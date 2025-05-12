#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include "max6675.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define MQTT_MAX_PACKET_SIZE 512

/*****************
 * CONFIGURATION *
 *****************/

// WiFi settings
const char* WIFI_SSID = "EnderDragon";
const char* WIFI_PASSWORD = "";
const uint8_t WIFI_MAX_RETRIES = 20;
const unsigned long WIFI_RETRY_DELAY_MS = 500;

// MQTT Broker settings
const char* MQTT_SERVER = "a379f517b65b4ff8b4e739b7320b34c6.s1.eu.hivemq.cloud";
const char* MQTT_USERNAME = "ESP8266";
const char* MQTT_PASSWORD = "mOS123@!";
const int MQTT_PORT = 8883;
const char* MQTT_CLIENT_ID_PREFIX = "ESP8266_WafelIjzer";
const char* MQTT_TOPIC = "esp8266/info";
const char* MQTT_SUBSCRIBE_TOPIC = "esp8266/commands";
const char* MQTT_TARGET_TOPIC = "esp8266/temp";
const char* MQTT_MODE_TOPIC = "esp8266/mode";
const unsigned long MQTT_RETRY_DELAY_MS = 5000;


// Sensor configuration
const int THERMO_DO = 12;
const int THERMO_CS = 2;
const int THERMO_CLK = 14;
const float TEMP_MIN_VALID = -20.0;
const float TEMP_MAX_VALID = 1000.0;

// Output pin control
const int RELAY_PIN1 = 13;
const int RELAY_PIN2 = 0;

// General settings
const unsigned long PUBLISH_INTERVAL_MS = 1000;
uint8_t relayMode = 0; // 0: both, 1: only RELAY_PIN1, 2: only RELAY_PIN2

/*************
 * VARIABLES *
 *************/

WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);
MAX6675 thermocouple(THERMO_CLK, THERMO_CS, THERMO_DO);
LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned long lastPublishTime = 0;
float targetTemperature = 0;
bool wifiConnected = false;
bool sensorError = false;

byte arrowUp[8] = {
  0b00100,
  0b01110,
  0b10101,
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b00000
};

byte arrowDown[8] = {
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b10101,
  0b01110,
  0b00100,
  0b00000
};

byte arrowBoth[8] = {
  0b00100,
  0b01110,
  0b10101,
  0b00100,
  0b10101,
  0b01110,
  0b00100,
  0b00000
};


/*************
 * FUNCTIONS *
 *************/

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
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }

  Serial.println("\nWiFi connection failed.");
  return false;
}

unsigned long getUptime() {
  return millis() / 1000;
}

float readTemperature() {
  float temperature = thermocouple.readCelsius();
  if (isnan(temperature) || temperature < TEMP_MIN_VALID || temperature > TEMP_MAX_VALID) {
    Serial.println("ERROR: Invalid temperature reading");
    sensorError = true;
    return NAN;
  }
  sensorError = false;
  return temperature;
}

bool publishMessage(const char* topic, const char* payload, boolean retained) {
  if (!mqttClient.connected()) return false;

  bool success = mqttClient.publish(topic, payload, retained);
  if (success) {
    Serial.printf("Published to %s: %s\n", topic, payload);
  } else {
    Serial.printf("Failed to publish to %s\n", topic);
  }
  return success;
}

void publishEspInfo(float currentTemp) {
  StaticJsonDocument<384> doc;
  doc["identifier"] = "ESP8266 Info";
  doc["type"] = "ESP8266 Info";

  JsonObject data = doc.createNestedObject("data");
  data["ip"] = WiFi.localIP().toString();
  data["rssi"] = WiFi.RSSI();
  data["uptime"] = getUptime();
  data["device_id"] = WiFi.macAddress();

  if (!isnan(currentTemp)) {
    data["temperature"] = currentTemp;
    data["sensor_status"] = "ok";
  } else {
    data["sensor_status"] = "error";
  }

  if (!isnan(targetTemperature)) {
    data["target_temperature"] = targetTemperature;
  }

  // Publish relay status
  /*data["relay_status1"] = digitalRead(RELAY_PIN1) == LOW;
  data["relay_status2"] = digitalRead(RELAY_PIN2) == LOW;*/

  char jsonBuffer[384];
  serializeJson(doc, jsonBuffer);
  publishMessage(MQTT_TOPIC, jsonBuffer, true);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.printf("Message arrived [%s]: %s\n", topic, msg.c_str());

  if (String(topic) == MQTT_SUBSCRIBE_TOPIC && msg == "restart") {
    ESP.restart();
  } else if (String(topic) == MQTT_TARGET_TOPIC) {
    targetTemperature = msg.toInt();
    Serial.print("Target Temperature is: ");
    Serial.println(targetTemperature);
  } else if (String(topic) == MQTT_MODE_TOPIC) {
    int mode = msg.toInt();
    if (mode >= 0 && mode <= 2) {
      relayMode = mode;
      Serial.printf("Relay mode set to: %d\n", relayMode);
    } else {
      Serial.println("Invalid mode received");
    }
  }
}


bool connectToMqtt() {
  if (mqttClient.connected()) return true;

  String clientId = MQTT_CLIENT_ID_PREFIX + String(random(0xffff), HEX);
  Serial.printf("Connecting to MQTT broker %s:%d...\n", MQTT_SERVER, MQTT_PORT);

  if (mqttClient.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
    mqttClient.subscribe(MQTT_SUBSCRIBE_TOPIC);
    mqttClient.subscribe(MQTT_TARGET_TOPIC);
    mqttClient.subscribe(MQTT_MODE_TOPIC);
    Serial.println("MQTT connected and subscribed");
    return true;
  }

  Serial.printf("MQTT connection failed, rc=%d\n", mqttClient.state());
  return false;
}

void handleRelay(float currentTemp) {
  bool turnOn = (!isnan(currentTemp) && !isnan(targetTemperature) && (currentTemp - targetTemperature <= -5));

  switch (relayMode) {
    case 0: // both relays
      digitalWrite(RELAY_PIN1, turnOn ? LOW : HIGH);
      digitalWrite(RELAY_PIN2, turnOn ? LOW : HIGH);
      break;
    case 1: // only RELAY_PIN1
      digitalWrite(RELAY_PIN1, turnOn ? LOW : HIGH);
      digitalWrite(RELAY_PIN2, HIGH);
      break;
    case 2: // only RELAY_PIN2
      digitalWrite(RELAY_PIN1, HIGH);
      digitalWrite(RELAY_PIN2, turnOn ? LOW : HIGH);
      break;
    default: // fallback to safe
      digitalWrite(RELAY_PIN1, HIGH);
      digitalWrite(RELAY_PIN2, HIGH);
      break;
  }
}


void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);

  Serial.println("\nESP8266 MQTT Temperature Monitor");
  // initialize LCD
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("WafelIjzer");
  lcd.setCursor(0, 1);
  lcd.print("Booting...");
  lcd.createChar(0, arrowUp);
  lcd.createChar(1, arrowDown);
  lcd.createChar(2, arrowBoth);


  pinMode(RELAY_PIN1, OUTPUT);
  digitalWrite(RELAY_PIN1, HIGH);
  pinMode(RELAY_PIN2, OUTPUT);
  digitalWrite(RELAY_PIN2, HIGH);

  wifiConnected = connectToWiFi();
  wifiClient.setInsecure(); // Use certificates in production
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(60);

  delay(500); // allow MAX6675 to stabilize
  float initialTemp = readTemperature();
  if (!isnan(initialTemp)) {
    Serial.printf("Initial temp: %.2f°C\n", initialTemp);
  } else {
    Serial.println("Initial temperature read failed.");
  }
  

  Serial.println("Setup complete");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiConnected) {
      Serial.println("WiFi disconnected. Reconnecting...");
      wifiConnected = false;
    }
    WiFi.reconnect();
    delay(WIFI_RETRY_DELAY_MS);
    return;
  } else if (!wifiConnected) {
    Serial.println("WiFi reconnected.");
    wifiConnected = true;
  }

  if (!mqttClient.connected()) {
    if (!connectToMqtt()) {
      delay(MQTT_RETRY_DELAY_MS);
      return;
    }
  }

  mqttClient.loop();

  unsigned long currentMillis = millis();
  if (currentMillis - lastPublishTime >= PUBLISH_INTERVAL_MS) {
    float currentTemp = readTemperature();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    if (!isnan(currentTemp)) {
      lcd.print(currentTemp);
      lcd.print((char)223); // degree symbol
      lcd.print("C");
    } else {
      lcd.print("Sensor Err");
    }

    lcd.setCursor(0, 1);
    lcd.print("Target: ");
    lcd.print(int(targetTemperature));
    lcd.print((char)223);  // degree symbol
    lcd.print("C ");

    if(digitalRead(RELAY_PIN1) == LOW || digitalRead(RELAY_PIN2) == LOW){
      switch (relayMode) {
        case 0: lcd.write(byte(2)); break; // ↕ Both
        case 1: lcd.write(byte(1)); break; // ↑ Up
        case 2: lcd.write(byte(0)); break; // ↓ Down
        default: lcd.print("?");
      }
    }

    handleRelay(currentTemp);
    publishEspInfo(currentTemp);
    lastPublishTime = currentMillis;
  }
}