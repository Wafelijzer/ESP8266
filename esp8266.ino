/**
 * ESP8266 MQTT Temperature Monitor
 * 
 * This program connects an ESP8266 to WiFi and MQTT, reads temperature from a MAX6675 thermocouple,
 * and publishes device information to an MQTT broker using a single topic.
 * 
 * Features:
 * - Secure MQTT connection over TLS
 * - Automatic reconnection for WiFi and MQTT
 * - JSON-formatted data publishing
 * - Device telemetry including uptime, signal strength, and IP
 * 
 * Hardware:
 * - ESP8266 module
 * - MAX6675 thermocouple interface
 * 
 * Version: 1.0.0
 * Date: March 27, 2025
 */

 #include <ESP8266WiFi.h>
 #include <PubSubClient.h>
 #include <WiFiClientSecure.h>
 #include <ArduinoJson.h>
 #include "max6675.h"
 
 /*****************
  * CONFIGURATION *
  *****************/
 
 // WiFi settings
 const char* WIFI_SSID = "CVTI-60144";
 const char* WIFI_PASSWORD = "mOS123@!";
 const uint8_t WIFI_MAX_RETRIES = 20;
 const unsigned long WIFI_RETRY_DELAY_MS = 500;
 
 // MQTT Broker settings
 const char* MQTT_SERVER = "a379f517b65b4ff8b4e739b7320b34c6.s1.eu.hivemq.cloud";
 const char* MQTT_USERNAME = "ESP8266";
 const char* MQTT_PASSWORD = "mOS123@!";
 const int MQTT_PORT = 8883;
 const char* MQTT_CLIENT_ID_PREFIX = "ESP8266_WafelIjzer";
 const unsigned long MQTT_RETRY_DELAY_MS = 5000;
 
 // Single topic for all data
 const char* MQTT_TOPIC = "esp8266/info";
 const char* MQTT_SUBSCRIBE_TOPIC = "esp8266/commands";
 
 // Sensor configuration
 const int THERMO_DO = 12;   // Data out pin
 const int THERMO_CS = 2;    // Chip select pin
 const int THERMO_CLK = 14;  // Clock pin
 const float TEMP_MIN_VALID = -20.0;  // Minimum valid temperature
 const float TEMP_MAX_VALID = 1000.0; // Maximum valid temperature
 
 // General settings
 const unsigned long PUBLISH_INTERVAL_MS = 5000; // How often to publish data
 
 /*************
  * VARIABLES *
  *************/
 
 WiFiClientSecure wifiClient;
 PubSubClient mqttClient(wifiClient);
 MAX6675 thermocouple(THERMO_CLK, THERMO_CS, THERMO_DO);
 
 unsigned long lastPublishTime = 0;
 unsigned long reconnectAttempts = 0;
 bool wifiConnected = false;
 bool sensorError = false;
 
 /*************
  * FUNCTIONS *
  *************/
 
 /**
  * Connect to WiFi network with retry mechanism
  * @return true if connected, false if failed after max retries
  */
 bool connectToWiFi() {
   Serial.print("Connecting to WiFi: ");
   Serial.println(WIFI_SSID);
 
   WiFi.mode(WIFI_STA);
   WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
 
   uint8_t retries = 0;
   while (WiFi.status() != WL_CONNECTED && retries < WIFI_MAX_RETRIES) {
     delay(WIFI_RETRY_DELAY_MS);
     Serial.print(".");
     retries++;
   }
 
   if (WiFi.status() == WL_CONNECTED) {
     Serial.println("\nWiFi connected!");
     Serial.print("IP address: ");
     Serial.println(WiFi.localIP());
     Serial.print("Signal strength (RSSI): ");
     Serial.print(WiFi.RSSI());
     Serial.println(" dBm");
     return true;
   } else {
     Serial.println("\nWiFi connection failed after maximum retries");
     return false;
   }
 }
 
 /**
  * Calculate and return device uptime in seconds
  * @return Uptime in seconds
  */
 unsigned long getUptime() {
   return millis() / 1000;
 }
 
 /**
  * Read temperature from the MAX6675 sensor with validation
  * @return Temperature in Celsius or NAN if reading is invalid
  */
 float readTemperature() {
   float temperature = thermocouple.readCelsius();
   
   // Check if reading is valid
   if (isnan(temperature) || temperature < TEMP_MIN_VALID || temperature > TEMP_MAX_VALID) {
     Serial.println("ERROR: Invalid temperature reading");
     sensorError = true;
     return NAN;
   }
   
   sensorError = false;
   return temperature;
 }
 
 /**
  * Publish a message to an MQTT topic
  * @param topic The topic to publish to
  * @param payload The message payload
  * @param retained Whether the message should be retained by the broker
  * @return true if published successfully, false otherwise
  */
 bool publishMessage(const char* topic, const char* payload, boolean retained) {
   if (!mqttClient.connected()) {
     return false;
   }
   
   bool success = mqttClient.publish(topic, payload, retained);
   if (success) {
     Serial.print("Published to ");
     Serial.print(topic);
     Serial.print(": ");
     Serial.println(payload);
   } else {
     Serial.print("Failed to publish to ");
     Serial.println(topic);
   }
   
   return success;
 }
 
 /**
  * Create and publish all device information to a single topic
  */
 void publishEspInfo() {
   // Create a JSON document
   StaticJsonDocument<384> doc;
   
   // Basic device info
   doc["identifier"] = "ESP8266 Info";
   doc["type"] = "ESP8266 Info";
   doc["device_id"] = WiFi.macAddress();
   doc["timestamp"] = getUptime();
   
   // Create data object for all metrics
   JsonObject data = doc.createNestedObject("data");
   
   // Add network information
   data["ip"] = WiFi.localIP().toString();
   data["rssi"] = WiFi.RSSI();
   data["uptime"] = getUptime();
   
   // Add sensor information
   data["sensor_type"] = "MAX6675";
   
   // Add system stats
   data["free_heap"] = ESP.getFreeHeap();
   data["reconnects"] = reconnectAttempts;
   
   // Add temperature data if available
   float temperature = readTemperature();
   if (!isnan(temperature)) {
     data["temperature"] = temperature;
     data["unit"] = "celsius";
     data["sensor_status"] = "ok";
   } else {
     data["sensor_status"] = "error";
   }
   
   // Serialize JSON document
   char jsonBuffer[384];
   serializeJson(doc, jsonBuffer);
   
   // Publish the JSON string
   publishMessage(MQTT_TOPIC, jsonBuffer, true);
 }
 
 /**
  * Callback for handling incoming MQTT messages
  */
 void mqttCallback(char* topic, byte* payload, unsigned int length) {
   // Convert payload to string
   String incomingMessage = "";
   for (int i = 0; i < length; i++) {
     incomingMessage += (char)payload[i];
   }
   
   Serial.print("Message arrived [");
   Serial.print(topic);
   Serial.print("]: ");
   Serial.println(incomingMessage);
   
   // Add command handling as needed
   // For example, if we receive a "restart" command:
   if (incomingMessage == "restart") {
     Serial.println("Restart command received. Restarting...");
     ESP.restart();
   }
 }
 
 /**
  * Connect to the MQTT broker with retry mechanism
  */
 bool connectToMqtt() {
   if (mqttClient.connected()) {
     return true;
   }
   
   Serial.print("Connecting to MQTT broker at ");
   Serial.print(MQTT_SERVER);
   Serial.print(":");
   Serial.print(MQTT_PORT);
   Serial.println("...");
   
   // Create a unique client ID
   String clientId = MQTT_CLIENT_ID_PREFIX;
   clientId += String(random(0xffff), HEX);
   
   // Attempt to connect
   if (mqttClient.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
     Serial.println("Connected to MQTT broker");
     
     // Subscribe to command topic
     mqttClient.subscribe(MQTT_SUBSCRIBE_TOPIC);
     Serial.print("Subscribed to topic: ");
     Serial.println(MQTT_SUBSCRIBE_TOPIC);
     
     // Publish a connection message
     StaticJsonDocument<128> doc;
     doc["identifier"] = "ESP8266 Info";
     doc["type"] = "ESP8266 Info";
     doc["data"]["status"] = "connected";
     doc["data"]["device_id"] = WiFi.macAddress();
     doc["data"]["ip"] = WiFi.localIP().toString();
     
     char jsonBuffer[128];
     serializeJson(doc, jsonBuffer);
     publishMessage(MQTT_TOPIC, jsonBuffer, true);
     
     reconnectAttempts = 0;
     return true;
   } else {
     reconnectAttempts++;
     Serial.print("MQTT connection failed, rc=");
     Serial.print(mqttClient.state());
     return false;
   }
 }
 
 /**
  * Initial setup function
  */
 void setup() {
   // Initialize serial communication
   Serial.begin(115200);
   while (!Serial && millis() < 5000); // Wait for serial to connect or timeout
   
   Serial.println("\n\nESP8266 MQTT Temperature Monitor");
   Serial.println("--------------------------------");
   
   // Connect to WiFi
   Serial.print("Connecting to ");
   Serial.println(WIFI_SSID);
 
   WiFi.mode(WIFI_STA);
   WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
 
   while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     Serial.print(".");
   }
   Serial.println("\nWiFi connected\nIP address: ");
   Serial.println(WiFi.localIP());
 
   wifiConnected = true;
   
   // Configure MQTT client
   wifiClient.setInsecure(); // Note: In production, consider using certificate validation
   mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
   mqttClient.setCallback(mqttCallback);
   mqttClient.setKeepAlive(60);
   
   // Initialize MAX6675 sensor
   Serial.println("Initializing MAX6675 temperature sensor...");
   delay(500); // Allow MAX6675 to stabilize
   
   // Test sensor
   float initialTemp = readTemperature();
   if (!isnan(initialTemp)) {
     Serial.print("Initial temperature reading: ");
     Serial.print(initialTemp);
     Serial.println("°C");
   } else {
     Serial.println("WARNING: Could not get initial temperature reading");
   }
   
   Serial.println("Setup complete!");
 }
 
 /**
  * Main program loop
  */
 void loop() {
   // Check WiFi connection
   if (WiFi.status() != WL_CONNECTED) {
     if (wifiConnected) {
       Serial.println("WiFi disconnected. Attempting to reconnect...");
       wifiConnected = false;
     }
     
     WiFi.reconnect();
     delay(WIFI_RETRY_DELAY_MS);
     
     if (WiFi.status() == WL_CONNECTED) {
       Serial.println("WiFi reconnected!");
       wifiConnected = true;
     }
     return; // Skip the rest of the loop if WiFi is disconnected
   }
   
   // Ensure MQTT connection
   if (!mqttClient.connected()) {
     connectToMqtt();
     // If connection fails, wait before trying again
     if (!mqttClient.connected()) {
       delay(MQTT_RETRY_DELAY_MS);
       return; // Skip the rest of the loop if MQTT is disconnected
     }
   }
   
   // Process MQTT messages
   mqttClient.loop();
   
   // Publish data at regular intervals
   unsigned long currentMillis = millis();
   
   if (currentMillis - lastPublishTime >= PUBLISH_INTERVAL_MS) {
     publishEspInfo();
     lastPublishTime = currentMillis;
   }
 }