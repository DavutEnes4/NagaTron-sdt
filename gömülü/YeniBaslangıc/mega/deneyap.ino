#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>

// ---- WiFi Ayarları ----
const char* ssid = "nagatron-wifi";
const char* password = "nagatron";

// ---- MQTT Ayarları ----
const char* mqtt_server = "nagatron-sdt.local";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

// ---- MQTT Topicleri ----
const char* commandTopic = "robot/command";     // Gelen komutlar
const char* logTopic = "robot/log";             // Giden veriler

// ---- I2C Ayarları ----
#define SLAVE_ADDRESS 0x42
char incomingData[64]; // Slave'den gelen veri
#define INTERRUPT_PIN 26  // Slave'in sinyal gönderdiği pin

// === SETUP ===
void setup() {
  Serial.begin(115200);
  Wire.begin();  // Deneyap Kart I2C Master

  WiFi.begin(ssid, password);
  Serial.print("WiFi Bağlanıyor");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi Bağlandı!");

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
}

// === MQTT Geri Çağırma ===
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.println("MQTT mesajı alındı");

  payload[length] = '\0';
  String msgStr = String((char*)payload);
  Serial.println("Gelen JSON: " + msgStr);

  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, msgStr);
  if (error) {
    Serial.println("JSON ayrıştırma hatası!");
    return;
  }

  // === Komut kontrolü ===
  if (doc.containsKey("cmd")) {
    const char* cmd = doc["cmd"];
    sendCommand(cmd); // I2C ile slave'e gönder
  }

  // === PID ayar güncellemesi ===
  if (doc.containsKey("Kp") || doc.containsKey("Kd")) {
    char buffer[32];
    sprintf(buffer, "KP%.2fKD%d", doc["Kp"] | 0.4, doc["Kd"] | 2);
    sendCommand(buffer); // Örneğin: KP0.40KD2
  }
}

// === I2C Üzerinden Komut Gönderme ===
void sendCommand(const char* command) {
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(command);
  Wire.endTransmission();
  Serial.print("I2C ile gönderildi: ");
  Serial.println(command);
}

// === I2C Üzerinden Veri Okuma ===
void requestSlaveData() {
  Wire.requestFrom(SLAVE_ADDRESS, 64);
  int i = 0;
  while (Wire.available()) {
    incomingData[i++] = Wire.read();
  }
  incomingData[i] = '\0';

  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, incomingData);
  if (!err) {
    char buffer[128];
    serializeJson(doc, buffer);
    client.publish(logTopic, buffer);
    Serial.println("MQTT'ye log gönderildi: ");
    Serial.println(buffer);
  } else {
    Serial.println("Slave'den geçerli veri gelmedi.");
  }
}

// === MQTT Bağlantısı Sağlama ===
void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT bağlanıyor...");
    if (client.connect("RobotMasterClient")) {
      Serial.println("Bağlandı!");
      client.subscribe(commandTopic);
    } else {
      Serial.print("Bağlantı başarısız, tekrar deneniyor...");
      delay(2000);
    }
  }
}

// === LOOP ===
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  static unsigned long lastRequestTime = 0;
  if (millis() - lastRequestTime > 2000) {
    requestSlaveData();
    lastRequestTime = millis();
  }
}

