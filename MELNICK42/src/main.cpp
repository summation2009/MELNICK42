#include <Arduino.h>
#include <SIM76xx.h>
#include <GSMClient.h>
#include <PubSubClient.h>
#include "DHT.h"
#include "ModbusMaster.h"
#include "esp_system.h"

ModbusMaster myModbus;

// ค่าเซนเซอร์ RS485
float Humi = 0.0;
float Temp = 0.0;

// DHT11
#define DHTPIN 25
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Relay Pins
#define Relay1_PIN26 26
#define Relay2_PIN27 27
#define Relay3_PIN33 33
#define Relay4_PIN32 32

// Input / Switch Pins
#define INPUT18 18
#define INPUT19 19
#define SW1 4
#define SW2 5

// MQTT Config
const char *mqtt_server = "35.236.157.133"; 
const int mqtt_port = 1883;
const char *mqtt_user = "TON_TON";
const char *mqtt_pass = "123456aaff";

String mqtt_client_id;
String mqtt_publish_topic;
String mqtt_subscribe_topic;

GSMClient gsm_client;
PubSubClient client(gsm_client);

// ดึง MAC Address
String getDeviceMAC() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);  
  char buf[13];
  sprintf(buf, "%02X%02X%02X%02X%02X%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

// Callback MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) Serial.print((char)payload[i]);
  Serial.println();
}

// Reconnect MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(mqtt_client_id.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      client.subscribe(mqtt_subscribe_topic.c_str());
      Serial.print("Subscribed to: ");
      Serial.println(mqtt_subscribe_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// อ่าน RS485
void read_sensor_RS485() {
  uint8_t result = myModbus.readHoldingRegisters(0, 4);
  if (result == myModbus.ku8MBSuccess) {
    Humi = myModbus.getResponseBuffer(0) / 10.0;
    Temp = myModbus.getResponseBuffer(1) / 10.0;
  }
}

// Publish MQTT
void publishData(float dhtH, float dhtT, float rsH, float rsT) {
  if (!client.connected()) reconnect();
  client.loop();

  char msg[250];
  sprintf(msg,
          "{\"DHT_Humi\":%.1f,\"DHT_Temp\":%.1f,"
          "\"RS485_Humi\":%.1f,\"RS485_Temp\":%.1f}",
          dhtH, dhtT, rsH, rsT);

  client.publish(mqtt_publish_topic.c_str(), msg);
  Serial.print("MQTT Publish [");
  Serial.print(mqtt_publish_topic);
  Serial.print("]: ");
  Serial.println(msg);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // RS485

  dht.begin();

  while (!GSM.begin()) {
    Serial.println("GSM setup fail");
    delay(2000);
  }

  // ตั้งค่ารีเลย์และสวิตช์
  pinMode(Relay1_PIN26, OUTPUT);
  pinMode(Relay2_PIN27, OUTPUT);
  pinMode(Relay3_PIN33, OUTPUT);
  pinMode(Relay4_PIN32, OUTPUT);

  pinMode(INPUT18, INPUT_PULLUP);
  pinMode(INPUT19, INPUT_PULLUP);
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);

  digitalWrite(Relay1_PIN26, HIGH);
  digitalWrite(Relay2_PIN27, HIGH);
  digitalWrite(Relay3_PIN33, HIGH);
  digitalWrite(Relay4_PIN32, HIGH);

  myModbus.begin(1, Serial2); // RS485 Slave ID = 1

  String mac = getDeviceMAC();
  mqtt_client_id     = "ESP32_" + mac;
  mqtt_publish_topic = "device/" + mac + "/data";
  mqtt_subscribe_topic = "device/" + mac + "/cmd";

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  Serial.println("Setup done");
  Serial.print("Client ID: "); Serial.println(mqtt_client_id);
  Serial.print("Publish Topic: "); Serial.println(mqtt_publish_topic);
  Serial.print("Subscribe Topic: "); Serial.println(mqtt_subscribe_topic);
}

unsigned long lastPublish = 0;
const unsigned long publishInterval = 5000;

unsigned long pressTimeInput18 = 0;
unsigned long pressTimeInput19 = 0;
unsigned long pressTimeSW1 = 0;
unsigned long pressTimeSW2 = 0;

void loop() {
  unsigned long now = millis();

  // 🔹 Group 1: INPUT18 + SW1 → Relay1 + Relay2
  bool group1Pressed = (digitalRead(INPUT18) == LOW) || (digitalRead(SW1) == LOW);
  if (group1Pressed) {
    if (pressTimeInput18 == 0) pressTimeInput18 = now;
    if (now - pressTimeInput18 >= 1000) {
      digitalWrite(Relay1_PIN26, LOW);
      digitalWrite(Relay2_PIN27, LOW);
    }
  } else {
    pressTimeInput18 = 0;
    digitalWrite(Relay1_PIN26, HIGH);
    digitalWrite(Relay2_PIN27, HIGH);
  }

  // 🔹 Group 2: INPUT19 + SW2 → Relay3 + Relay4
  bool group2Pressed = (digitalRead(INPUT19) == LOW) || (digitalRead(SW2) == LOW);
  if (group2Pressed) {
    if (pressTimeInput19 == 0) pressTimeInput19 = now;
    if (now - pressTimeInput19 >= 1000) {
      digitalWrite(Relay3_PIN33, LOW);
      digitalWrite(Relay4_PIN32, LOW);
    }
  } else {
    pressTimeInput19 = 0;
    digitalWrite(Relay3_PIN33, HIGH);
    digitalWrite(Relay4_PIN32, HIGH);
  }

  // ส่งข้อมูลทุก 5 วินาที
  if (now - lastPublish >= publishInterval) {
    lastPublish = now;

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (!isnan(h) && !isnan(t)) {
      read_sensor_RS485();
      publishData(h, t, Humi, Temp);
    } else {
      Serial.println("Failed to read from DHT11!");
    }
  }
}
