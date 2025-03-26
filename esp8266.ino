#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

// WiFi settings
const char* ssid = "CVTI-60144";
const char* password = "mOS123@!";

// MQTT Broker settings
const char* mqtt_server = "a379f517b65b4ff8b4e739b7320b34c6.s1.eu.hivemq.cloud";
const char* mqtt_username = "ESP8266";
const char* mqtt_password = "mOS123@!";
const int mqtt_port = 8883;

WiFiClientSecure espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;

#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];

bool wifiConnected = false;

// Function to get uptime in seconds
unsigned long getUptime() {
  return millis() / 1000;
}

void publishEspInfo() {
  // Create a JSON document
  StaticJsonDocument<200> doc;

  // Add information to the JSON document
  doc["identifier"] = "ESP8266 Info";
  doc["type"] = "ESP8266 Info";
  doc["data"]["ip"] = WiFi.localIP().toString();
  doc["data"]["uptime"] = getUptime();
  doc["data"]["rssi"] = WiFi.RSSI();
  // Add more information as needed

  // Serialize JSON document to a string
  String jsonString;
  serializeJson(doc, jsonString);

  // Publish the JSON string
  publishMessage("esp8266/info", jsonString.c_str(), true);
}

void setup() {
  Serial.begin(9600);
  Serial.print("\nConnecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected\nIP address: ");
  Serial.println(WiFi.localIP());

  while (!Serial) delay(1);

  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  wifiConnected = true;
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    publishEspInfo();
    delay(5000);   
  } else {
    wifiConnected = false;
    Serial.println("WiFi disconnected. Attempting to reconnect...");
    WiFi.reconnect();
    delay(1000);
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi reconnected.");
      wifiConnected = true;
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection…");
    String clientId = "ESP32VANMATHIS";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");
      client.subscribe("HAMATHIS/Poort");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 1 seconds");
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String incomingMessage = "";
  for (int i = 0; i < length; i++) {
    incomingMessage += (char)payload[i];
  }
  Serial.println("Message arrived [" + String(topic) + "]" + incomingMessage);
}

void publishMessage(const char* topic, String payload, boolean retained) {
  if (client.publish(topic, payload.c_str(), retained)) {
    Serial.println("Published!");
  }
}
