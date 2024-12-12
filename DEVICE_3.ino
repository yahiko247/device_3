#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ESPDateTime.h>  // ESPDateTime Library

#define SOIL_MOISTURE_PIN 34      // Soil moisture sensor pin
#define DHTPIN 4                 // Pin connected to DHT11
#define DHTTYPE DHT11             // DHT 11 sensor
#define WIFI_SSID "DITO"       // Your hotspot SSID
#define WIFI_PASSWORD "gwapako123"  // Your hotspot password
#define MQTT_BROKER "192.168.254.147"  // Free broker

#define MQTT_PORT 1883
// #define MQTT_TOPIC_TEMPERATURE "home/sensor/temperature"
// #define MQTT_TOPIC_HUMIDITY "home/sensor/humidity"
// #define MQTT_TOPIC_SOILMOISTURE "home/sensor/soilMoisture"
#define MQTT_TOPIC_COMBINE "smmic/sensor/data"
#define SMMIC_IRRIGATION "smmic/sensor/triggers/irrigation/2d976174-d657-4f1d-9432-758996d36455"
#define ALERTS "smmic/sensor/alert"
#define INTERVAL "smmic/sensor/triggers/interval/2d976174-d657-4f1d-9432-758996d36455"
#define LED_PIN 15 // Check connection of MQTT
#define RELAY_PIN 23 // Pin for the relay
#define DEVICE_ID "2d976174-d657-4f1d-9432-758996d36455"
#define COMMAND_FEEDBACK "smmic/user/commands/feedback"
#include <NTPClient.h>
#include <WiFiUdp.h>

// NTP server
const char* ntpServer = "192.168.254.147";
const int timeOffset = 0;
const int updateInterval = 60000;

// create a UUP instance and NTP client
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer, timeOffset, updateInterval);

DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

int interval = 60;
int irrigation = 0;




const int min_moisture = 2000;  // Minimum soil moisture value (calibrated)
const int max_moisture = 4095;  // Maximum soil moisture value (calibrated)

void setup() {
  Serial.begin(115200);
  
  // Initialize DHT sensor
  dht.begin();

  // Setup LED pin
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT); // Set relay pin as output
  digitalWrite(RELAY_PIN, LOW); // Make sure relay is initially off

  // Create tasks
  xTaskCreate(connectWiFiTask, "ConnectWiFiTask", 2048, NULL, 1, NULL);
  xTaskCreate(mqttTask, "MqttTask", 4096, NULL, 1, NULL);
  xTaskCreate(blinkLEDTask, "BlinkLEDTask", 2048, NULL, 1, NULL);
  xTaskCreate(relayControlTask, "RelayControlTask", 2048, NULL, 1, NULL); // Add relay control task


}

void loop() {
  // FreeRTOS manages the tasks; no code needed here
}

void connectWiFiTask(void *pvParameters) {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int max_attempts = 20;  // Timeout after 20 seconds (20 attempts)
  int attempts = 0;

  while (WiFi.status() != WL_CONNECTED && attempts < max_attempts) {
    delay(1000);
    Serial.println("Attempting WiFi connection...");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
   
    timeClient.begin();  
     if (!DateTime.begin()) {
      Serial.println("Failed to get time from server.");
    } else {
      Serial.println("Time synchronized successfully.");
    }
  } else {
    Serial.println("WiFi connection failed!");
  }
  
  vTaskDelete(NULL); // Delete this task when done
}

void mqttTask(void *pvParameters) {
  client.setServer(MQTT_BROKER, MQTT_PORT);

  int counter = 0;
  bool alertSent = false;

  while (true) {
    if (!client.connected()) {
      reconnectMQTT();  
    }
    client.loop();

   

    // Read temperature and humidity
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    // Read soil moisture value and convert to percentage
    int soilMoistureValue = analogRead(SOIL_MOISTURE_PIN);
    int soilMoisturePercentage = map(soilMoistureValue, min_moisture, max_moisture, 100, 0);
    Serial.println(soilMoistureValue);  // Reverse to show percentage correctly

    // Check if any reading failed
    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }

    // Prepare payloads for temperature, humidity, and soil moisture
    // char tempPayload[50];
    // snprintf(tempPayload, sizeof(tempPayload), "Temperature: %.2f°C", temperature);

    // char humPayload[50];
    // snprintf(humPayload, sizeof(humPayload), "Humidity: %.2f%%", humidity);

    // char soilPayload[50];
    // snprintf(soilPayload, sizeof(soilPayload), "SoilMoisture: %d%%", soilMoisturePercentage);

    // Get the current time using ESPDateTime
     timeClient.update();  // e.g., "2024-09-24 10:15:30"
    //const char* node1 = "b4182319-ddfb-4f24-807e-8a6a464dca22";
    int id = 1;
    int batt_level = 0;

    char combinePayload[250];
    snprintf(combinePayload, sizeof(combinePayload), "soil_moisture;%s;%d;temperature:%.2f&humidity:%.2f%&soil_moisture:%d%&battery_level:%d", DEVICE_ID, timeClient.getEpochTime(), temperature, humidity, soilMoisturePercentage, batt_level);
    Serial.println(interval);
    // Publish temperature, humidity, and soil moisture data
    // client.publish(MQTT_TOPIC_TEMPERATURE, tempPayload);
    // client.publish(MQTT_TOPIC_HUMIDITY, humPayload);
    // client.publish(MQTT_TOPIC_SOILMOISTURE, soilPayload);

    char lowMoistureAlert[250];
    if( alertSent == false && soilMoisturePercentage < 20){
      snprintf(lowMoistureAlert, sizeof(lowMoistureAlert), "%s;%d;42;temperature:%.2f&humidity:%.2f%&soil_moisture:%d%&battery_level:%d", DEVICE_ID, timeClient.getEpochTime(), temperature, humidity, soilMoisturePercentage, batt_level);
        client.publish(ALERTS, lowMoistureAlert);
        alertSent = true;
    }
    if( alertSent == true && soilMoisturePercentage >= 70){
       alertSent = false;
    }

    if(counter == interval || counter > interval){
      client.publish(MQTT_TOPIC_COMBINE, combinePayload);
      counter = 0;
    }
    

    // Print data to serial monitor
    // Serial.println(tempPayload);
    // Serial.println(humPayload);
    // Serial.println(soilPayload);
    Serial.println(combinePayload);
  
    // Delay for 2 seconds before sending the next reading
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    counter = counter + 1;
  }
}

void subcribeCallBack( char* topic, byte* payload, unsigned int length){
  Serial.println("receive message");

  if(strcmp(topic, INTERVAL) == 0){
    String message = "";
    for (int i = 0; i < length; i++){
        message += (char)payload[i];
    }

    int payloadInt = message.toInt();
    interval = payloadInt/5;
  }
  if(strcmp (topic, SMMIC_IRRIGATION) == 0){
    String message = "";
    for (int i = 0; i < length; i++){
        message += (char)payload[i];
    }

    int payloadInt = message.toInt();
    irrigation = payloadInt;

    Serial.println(irrigation);
  }
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.println("Connecting to MQTT broker...");
    char combineAlert[250];
    snprintf(combineAlert, sizeof(combineAlert), "%s;%d;0", DEVICE_ID, timeClient.getEpochTime());

    //setcallback
    client.setCallback(subcribeCallBack);

    // Define the LWT
    const char* lwtTopic = "smmic/sensor/alert";     // Topic for LWT
    const char* lwtMessage = combineAlert;        // Message to indicate the ESP32 is offline
    bool lwtRetain = true;                     // Retain the message on the broker
    int lwtQos = 0;                            // QoS level (0: at most once)
    if (client.connect(DEVICE_ID, nullptr, nullptr, lwtTopic, lwtQos, lwtRetain, lwtMessage)) {
      Serial.println("Connected to MQTT broker");

      char combineAlert2[250];
      snprintf(combineAlert2, sizeof(combineAlert2), "%s;%d;1", DEVICE_ID, timeClient.getEpochTime());
       // Publish initial "online" message
      client.publish(lwtTopic, combineAlert2, true);  // Retain the "online" message
      digitalWrite(LED_PIN, HIGH); // Turn on LED

      client.subscribe(SMMIC_IRRIGATION);
      client.subscribe(INTERVAL);
    } else {
      Serial.print("Failed to connect to MQTT, retrying in 5 seconds. Error: ");
      Serial.println(client.state());
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }
}

void blinkLEDTask(void *pvParameters) {
  while (true) {
    if (!client.connected()) {
      // Blink LED if MQTT is disconnected
      digitalWrite(LED_PIN, HIGH);
      vTaskDelay(250 / portTICK_PERIOD_MS);
      digitalWrite(LED_PIN, LOW);
      vTaskDelay(250 / portTICK_PERIOD_MS);
    } else {
      digitalWrite(LED_PIN, HIGH); // Keep LED on when connected
      vTaskDelay(1000 / portTICK_PERIOD_MS); // Check every second
    }
  }
}

void relayControlTask(void *pvParameters) {
  while (true) {
    int soilMoistureValue1 = analogRead(SOIL_MOISTURE_PIN);
    int soilMoisturePercentage1 = map(soilMoistureValue1, min_moisture, max_moisture, 100, 0);

    // read temperature and humidity from DHT
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    // Checks for dht reading errors

    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Failed to read from DHT sensor!");
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      continue;
    }
    const char* ON = "1";
    const char* OFF = "0";
    timeClient.update();

    if (soilMoisturePercentage1 < 20 || (humidity < 55 && temperature) > 32 || irrigation == 1) {
      digitalWrite(RELAY_PIN, HIGH); // Open the relay (turn it on)
      Serial.println("Relay opened: Soil moisture below 20% or temperature > 32°C and humidity < 55%");

      char SendONsignal[100];
      snprintf(SendONsignal, sizeof(SendONsignal),"%s;%d;%s",DEVICE_ID,timeClient.getEpochTime(),ON);
      client.publish("smmic/irrigation", SendONsignal);
      Serial.println(SendONsignal);
     
      
      bool flag = true;
      irrigation = 1;
      do {
        int soilMoistureValue2 = analogRead(SOIL_MOISTURE_PIN);
        int soilMoisturePercentage2 = map(soilMoistureValue2, min_moisture, max_moisture, 100, 0);

         Serial.println(irrigation);

        float temperature2 = dht.readTemperature();
        float humidity2 = dht.readHumidity();

        if (soilMoisturePercentage2 > 70 || irrigation == 0 ) {
          digitalWrite(RELAY_PIN, LOW); // Close the relay (turn it off)
          Serial.println("Relay closed: Soil moisture sufficient");
          irrigation = 0;
          char SendOFFsignal[100];
          snprintf(SendOFFsignal, sizeof(SendOFFsignal), "%s;%d;%s",DEVICE_ID,timeClient.getEpochTime(),OFF);
          client.publish("smmic/irrigation", SendOFFsignal);
          Serial.println(SendOFFsignal);
          flag = false;
        } else {
          Serial.println("Relay open: Solenoid OPEN");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      } while(flag);

    } else {
      Serial.println("Relay open: Solenoid OFF");
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS); // Check every 2 seconds
  }
}


