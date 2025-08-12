#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>

// ---- WiFi Ayarları ----
const char* ssid = "FenveTeknoloji";
const char* password = "fentek2024";

// ---- MQTT Ayarları ----
const char* mqtt_server = "nagatron-sdt.local";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

// ---- MQTT Topicleri ----
const char* commandTopic = "robot/command";   // Gelen komutlar
const char* logTopic     = "robot/log";       // Giden veriler

// ---- I2C Ayarları ----
#define SLAVE_ADDRESS 0x42
char incomingData[128]; // Slave'den gelen veri

// === SETUP ===
void setup() {
  Serial.begin(115200);
  Wire.begin();  // Deneyap Kart I2C Master

  // WiFi bağlantısı
  WiFi.begin(ssid, password);
  Serial.print("WiFi Bağlanıyor");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi Bağlandı!");

  // MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
}

// === MQTT Mesajı Alındığında ===
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
  if (doc.containsKey("Kp") || doc.containsKey("Kd") || doc.containsKey("Ki")) {
    float kp = doc["Kp"] | 0.4;
    float ki = doc["Ki"] | 0.0;
    float kd = doc["Kd"] | 2.0;

    char buffer[48];
    sprintf(buffer, "PID:%.2f,%.2f,%.2f", kp, ki, kd);  // Örn: PID:0.40,0.00,2.00
    sendCommand(buffer);
  }

  // === MODE değiştirme ===
  if (doc.containsKey("mode")) {
    const char* mode = doc["mode"]; // "LINE" veya "MANUAL"
    char buffer[16];
    sprintf(buffer, "MODE:%s", mode);
    sendCommand(buffer);
  }

  // === Hız değişikliği ===
  if (doc.containsKey("speed")) {
    int speed = doc["speed"];
    char buffer[16];
    sprintf(buffer, "SPD:%d", speed);
    sendCommand(buffer);
  }
}

// === I2C Üzerinden Komut Gönderme ===
void sendCommand(const char* command) {
  Serial.print("[I2C REQUEST] -> Komut gönderiliyor: ");
  Serial.println(command);

  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(command);
  byte error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("[I2C REQUEST] -> Komut başarıyla gönderildi.");
  } else {
    Serial.print("[I2C ERROR] -> Kod: ");
    Serial.println(error);
  }
}

// === I2C Üzerinden Veri Okuma ve MQTT Yayını ===
void requestSlaveData() {
  Serial.println("[I2C REQUEST] -> Slave'den veri talep ediliyor: GET:ALL");
  sendCommand("GET:ALL");
  delay(10);

  Wire.requestFrom(SLAVE_ADDRESS, 128);
  int byteCount = Wire.available();
  Serial.print("[I2C RECEIVE] -> Gelen byte sayısı: ");
  Serial.println(byteCount);

  int i = 0;
  while (Wire.available()) {
    incomingData[i++] = Wire.read();
    if (i >= 127) break;
  }
  incomingData[i] = '\0';

  if (byteCount > 0) {
    Serial.print("[I2C RECEIVE] -> Gelen veri: ");
    Serial.println(incomingData);
  }

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, incomingData);
  if (!err) {
    char buffer[256];
    serializeJson(doc, buffer);
    client.publish(logTopic, buffer);
    Serial.println("[MQTT PUBLISH] -> Log gönderildi: ");
    Serial.println(buffer);
  } else {
    Serial.print("[I2C ERROR] -> JSON ayrıştırılamadı: ");
    Serial.println(err.c_str());
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
      Serial.println("Bağlantı başarısız, tekrar deneniyor...");
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
    requestSlaveData();  // Her 2 saniyede bir verileri iste
    lastRequestTime = millis();
  }
}
