/*
 * 🔗 I²C Haberleşme Köprüsü
 * Deneyap Kart 2 (ESP32) - Arduino Mega ile MQTT arasında köprü
 * 
 * Fonksiyonlar:
 * - Arduino Mega'dan I²C ile veri alma
 * - MQTT üzerinden veri yayınlama
 * - MQTT'den gelen komutları Arduino'ya iletme
 * - Çoklu WiFi ağı desteği
 * - Otomatik yeniden bağlanma
 * - RadioLink R12DS v1.1 veri aktarımı
 * - Ortak konfigürasyon sistemi
 */

#include <WiFi.h>
#include <WiFiMulti.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include "../04_Dokumantasyon/Shared_Config.h"

// WiFi Ayarları
WiFiMulti wifiMulti;

WiFiClient espClient;
PubSubClient client(espClient);

// Veri Yapıları
struct IMUData {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
  float temperature;
  float pressure;
  float altitude;
  float heading;
} imuData;

struct QTR8AData {
  uint16_t sensor_values[8];
  uint16_t position;
  uint8_t calibrated;
  uint8_t line_detected;
} qtrData;

struct MotorData {
  uint8_t left_speed;
  uint8_t right_speed;
  uint8_t left_direction;
  uint8_t right_direction;
  float left_current;
  float right_current;
  float left_temperature;
  float right_temperature;
} motorData;

struct RadioLinkData {
  uint16_t throttle;
  uint16_t aileron;
  uint16_t elevator;
  uint16_t rudder;
  uint16_t gear;
  uint16_t aux1;
  uint16_t aux2;
  uint16_t aux3;
  uint8_t connected;
  uint8_t control_mode;
} radioData;

// Değişkenler
unsigned long lastDataTransmission = 0;
const unsigned long transmissionInterval = 100; // 100ms
bool newIMUData = false;
bool newQTRData = false;
bool newMotorData = false;
bool newRadioData = false;

// MQTT Konuları
const char* mqtt_topic_imu = MQTT_TOPIC_IMU;
const char* mqtt_topic_qtr = MQTT_TOPIC_QTR;
const char* mqtt_topic_motor = MQTT_TOPIC_MOTOR;
const char* mqtt_topic_radiolink = MQTT_TOPIC_RADIOLINK;
const char* mqtt_topic_command = MQTT_TOPIC_COMMAND;
const char* mqtt_topic_config = MQTT_TOPIC_CONFIG;
const char* mqtt_topic_pid = MQTT_TOPIC_PID;
const char* mqtt_topic_status = MQTT_TOPIC_STATUS;

void setup() {
  Serial.begin(115200);
  Serial.println("🔗 I²C Haberleşme Köprüsü Başlatılıyor...");
  
  // WiFi bağlantısı
  setupWiFi();
  
  // MQTT bağlantısı
  setupMQTT();
  
  // I²C Slave başlat
  setupI2C();
  
  // MQTT konularına abone ol
  subscribeToTopics();
  
  Serial.println("✅ I²C Köprü Sistemi Hazır!");
}

void loop() {
  // WiFi bağlantısını kontrol et
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ WiFi bağlantısı kesildi, yeniden bağlanılıyor...");
    setupWiFi();
  }
  
  // MQTT bağlantısını kontrol et
  if (!client.connected()) {
    Serial.println("❌ MQTT bağlantısı kesildi, yeniden bağlanılıyor...");
    reconnectMQTT();
  }
  
  // MQTT mesajlarını işle
  client.loop();
  
  // Yeni verileri MQTT'e gönder
  if (millis() - lastDataTransmission >= transmissionInterval) {
    sendSensorDataToMQTT();
    lastDataTransmission = millis();
  }
  
  delay(10);
}

void setupWiFi() {
  Serial.println("📡 WiFi bağlantısı kuruluyor...");
  
  wifiMulti.addAP(WIFI_SSID_1, WIFI_PASSWORD_1);
  wifiMulti.addAP(WIFI_SSID_2, WIFI_PASSWORD_2);
  
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  
  Serial.println();
  Serial.print("✅ WiFi Bağlandı: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP Adresi: ");
  Serial.println(WiFi.localIP());
}

void setupMQTT() {
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(mqttCallback);
}

void setupI2C() {
  // I²C Slave olarak başlat
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onReceive(receiveDataFromArduino);
  Wire.onRequest(sendDataToArduino);
  
  Serial.println("✅ I²C Slave Başlatıldı (Adres: 0x10)");
}

void subscribeToTopics() {
  client.subscribe(mqtt_topic_command);
  client.subscribe(mqtt_topic_config);
  client.subscribe(mqtt_topic_pid);
  Serial.println("✅ MQTT Konularına Abone Olundu");
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("🔗 MQTT Bağlantısı Kuruluyor...");
    
    if (client.connect(DEVICE_ID_BRIDGE, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("✅ MQTT Bağlandı");
      subscribeToTopics();
      
      // Durum mesajı gönder
      publishStatus("connected");
    } else {
      Serial.print("❌ MQTT Bağlantı Hatası, rc=");
      Serial.print(client.state());
      Serial.println(" 5 saniye sonra tekrar denenecek");
      delay(MQTT_RECONNECT_INTERVAL);
    }
  }
}

void receiveDataFromArduino() {
  if (Wire.available() >= 4) {
    uint8_t header1 = Wire.read();
    uint8_t header2 = Wire.read();
    
    if (header1 == 0xAA && header2 == 0x55) {
      uint8_t device_id = Wire.read();
      uint8_t data_type = Wire.read();
      
      if (device_id == ARDUINO_MEGA_ADDRESS) {
        processArduinoData(data_type);
      }
    }
  }
}

void processArduinoData(uint8_t data_type) {
  uint32_t timestamp;
  uint8_t data_length;
  
  // Timestamp ve data length oku
  Wire.readBytes((uint8_t*)&timestamp, 4);
  data_length = Wire.read();
  
  switch (data_type) {
    case 0x01: // IMU Data
      if (data_length == sizeof(IMUData)) {
        Wire.readBytes((uint8_t*)&imuData, sizeof(IMUData));
        newIMUData = true;
      }
      break;
      
    case 0x02: // QTR8A Data
      if (data_length == sizeof(QTR8AData)) {
        Wire.readBytes((uint8_t*)&qtrData, sizeof(QTR8AData));
        newQTRData = true;
      }
      break;
      
    case 0x03: // Motor Data
      if (data_length == sizeof(MotorData)) {
        Wire.readBytes((uint8_t*)&motorData, sizeof(MotorData));
        newMotorData = true;
      }
      break;
      
    case 0x04: // RadioLink Data
      if (data_length == sizeof(RadioLinkData)) {
        Wire.readBytes((uint8_t*)&radioData, sizeof(RadioLinkData));
        newRadioData = true;
      }
      break;
  }
}

void sendDataToArduino() {
  // Arduino'dan veri istendiğinde (şu an kullanılmıyor)
  // Gelecekte Arduino'ya veri göndermek için kullanılabilir
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("📨 MQTT Mesajı Alındı: ");
  Serial.println(topic);
  
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  if (strcmp(topic, mqtt_topic_command) == 0) {
    handleMotorCommand(message);
  } else if (strcmp(topic, mqtt_topic_config) == 0) {
    handlePinConfig(message);
  } else if (strcmp(topic, mqtt_topic_pid) == 0) {
    handlePIDConfig(message);
  }
}

void handleMotorCommand(String message) {
  Serial.println("🎛️ Motor Komutu Alındı: " + message);
  
  // JSON parse et
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.println("❌ JSON Parse Hatası");
    return;
  }
  
  // Motor komutunu Arduino'ya gönder
  sendMotorCommandToArduino(doc);
}

void handlePinConfig(String message) {
  Serial.println("🔧 Pin Konfigürasyonu Alındı: " + message);
  
  // JSON parse et
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.println("❌ JSON Parse Hatası");
    return;
  }
  
  // Pin konfigürasyonunu Arduino'ya gönder
  sendPinConfigToArduino(doc);
}

void handlePIDConfig(String message) {
  Serial.println("🎛️ PID Konfigürasyonu Alındı: " + message);
  
  // JSON parse et
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.println("❌ JSON Parse Hatası");
    return;
  }
  
  // PID konfigürasyonunu Arduino'ya gönder
  sendPIDConfigToArduino(doc);
}

void sendMotorCommandToArduino(DynamicJsonDocument& doc) {
  uint8_t left_speed = doc["left_speed"] | 0;
  uint8_t right_speed = doc["right_speed"] | 0;
  uint8_t left_direction = doc["left_direction"] | 0;
  uint8_t right_direction = doc["right_direction"] | 0;
  
  // Arduino'ya motor komutu gönder
  Wire.beginTransmission(ARDUINO_MEGA_ADDRESS);
  Wire.write(0xBB); // Header
  Wire.write(0x66);
  Wire.write(0x03); // Command type: Motor
  Wire.write(4); // Data length
  Wire.write(left_speed);
  Wire.write(right_speed);
  Wire.write(left_direction);
  Wire.write(right_direction);
  Wire.endTransmission();
  
  Serial.println("✅ Motor Komutu Arduino'ya Gönderildi");
}

void sendPinConfigToArduino(DynamicJsonDocument& doc) {
  // Pin konfigürasyonunu Arduino'ya gönder
  Wire.beginTransmission(ARDUINO_MEGA_ADDRESS);
  Wire.write(0xBB); // Header
  Wire.write(0x66);
  Wire.write(0x01); // Command type: Pin Config
  Wire.write(sizeof(Config)); // Data length
  
  // Config yapısını hazırla ve gönder
  // Bu kısım Arduino'daki Config yapısına uygun olmalı
  Wire.endTransmission();
  
  Serial.println("✅ Pin Konfigürasyonu Arduino'ya Gönderildi");
}

void sendPIDConfigToArduino(DynamicJsonDocument& doc) {
  float kp = doc["kp"] | 0.5;
  float ki = doc["ki"] | 0.0;
  float kd = doc["kd"] | 0.1;
  float setpoint = doc["setpoint"] | 3500;
  float output_limit = doc["output_limit"] | 255;
  
  // Arduino'ya PID konfigürasyonu gönder
  Wire.beginTransmission(ARDUINO_MEGA_ADDRESS);
  Wire.write(0xBB); // Header
  Wire.write(0x66);
  Wire.write(0x02); // Command type: PID Config
  Wire.write(20); // Data length (5 float * 4 bytes)
  
  // Float değerleri byte array'e çevir
  uint8_t* kp_bytes = (uint8_t*)&kp;
  uint8_t* ki_bytes = (uint8_t*)&ki;
  uint8_t* kd_bytes = (uint8_t*)&kd;
  uint8_t* setpoint_bytes = (uint8_t*)&setpoint;
  uint8_t* output_limit_bytes = (uint8_t*)&output_limit;
  
  for (int i = 0; i < 4; i++) {
    Wire.write(kp_bytes[i]);
    Wire.write(ki_bytes[i]);
    Wire.write(kd_bytes[i]);
    Wire.write(setpoint_bytes[i]);
    Wire.write(output_limit_bytes[i]);
  }
  
  Wire.endTransmission();
  
  Serial.println("✅ PID Konfigürasyonu Arduino'ya Gönderildi");
}

void sendSensorDataToMQTT() {
  // IMU verilerini gönder
  if (newIMUData) {
    publishIMUData();
    newIMUData = false;
  }
  
  // QTR8A verilerini gönder
  if (newQTRData) {
    publishQTRData();
    newQTRData = false;
  }
  
  // Motor verilerini gönder
  if (newMotorData) {
    publishMotorData();
    newMotorData = false;
  }
  
  if (newRadioData) {
    publishRadioLinkData();
    newRadioData = false;
  }
}

void publishIMUData() {
  DynamicJsonDocument doc(512);
  doc["device"] = "arduino_mega";
  doc["sensor"] = "gy89";
  doc["timestamp"] = millis();
  
  JsonObject accelerometer = doc.createNestedObject("accelerometer");
  accelerometer["x"] = imuData.accel_x;
  accelerometer["y"] = imuData.accel_y;
  accelerometer["z"] = imuData.accel_z;
  
  JsonObject gyroscope = doc.createNestedObject("gyroscope");
  gyroscope["x"] = imuData.gyro_x;
  gyroscope["y"] = imuData.gyro_y;
  gyroscope["z"] = imuData.gyro_z;
  
  JsonObject magnetometer = doc.createNestedObject("magnetometer");
  magnetometer["x"] = imuData.mag_x;
  magnetometer["y"] = imuData.mag_y;
  magnetometer["z"] = imuData.mag_z;
  
  doc["temperature"] = imuData.temperature;
  doc["pressure"] = imuData.pressure;
  doc["altitude"] = imuData.altitude;
  doc["heading"] = imuData.heading;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  client.publish(mqtt_topic_imu, jsonString.c_str());
  Serial.println("📤 IMU Verisi MQTT'e Gönderildi");
}

void publishQTRData() {
  DynamicJsonDocument doc(512);
  doc["device"] = "arduino_mega";
  doc["sensor"] = "qtr8a";
  doc["timestamp"] = millis();
  
  JsonArray values = doc.createNestedArray("values");
  for (int i = 0; i < 8; i++) {
    values.add(qtrData.sensor_values[i]);
  }
  
  doc["position"] = qtrData.position;
  doc["calibrated"] = qtrData.calibrated;
  doc["line_detected"] = qtrData.line_detected;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  client.publish(mqtt_topic_qtr, jsonString.c_str());
  Serial.println("📤 QTR8A Verisi MQTT'e Gönderildi");
}

void publishMotorData() {
  DynamicJsonDocument doc(512);
  doc["device"] = "arduino_mega";
  doc["timestamp"] = millis();
  
  JsonObject motors = doc.createNestedObject("motors");
  
  JsonObject left = motors.createNestedObject("left");
  left["speed"] = motorData.left_speed;
  left["direction"] = motorData.left_direction == 0 ? "forward" : "backward";
  left["current"] = motorData.left_current;
  left["temperature"] = motorData.left_temperature;
  
  JsonObject right = motors.createNestedObject("right");
  right["speed"] = motorData.right_speed;
  right["direction"] = motorData.right_direction == 0 ? "forward" : "backward";
  right["current"] = motorData.right_current;
  right["temperature"] = motorData.right_temperature;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  client.publish(mqtt_topic_motor, jsonString.c_str());
  Serial.println("📤 Motor Verisi MQTT'e Gönderildi");
}

void publishRadioLinkData() {
  StaticJsonDocument<512> doc;
  
  doc["device_id"] = "arduino_mega";
  doc["sensor_type"] = "radiolink_r12ds";
  doc["timestamp"] = millis();
  
  JsonObject data = doc.createNestedObject("data");
  data["throttle"] = radioData.throttle;
  data["aileron"] = radioData.aileron;
  data["elevator"] = radioData.elevator;
  data["rudder"] = radioData.rudder;
  data["gear"] = radioData.gear;
  data["aux1"] = radioData.aux1;
  data["aux2"] = radioData.aux2;
  data["aux3"] = radioData.aux3;
  data["connected"] = radioData.connected;
  data["control_mode"] = radioData.control_mode;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  if (client.publish(mqtt_topic_radiolink, jsonString.c_str())) {
    Serial.println("📡 RadioLink Verisi Yayınlandı");
  } else {
    Serial.println("❌ RadioLink Verisi Yayınlanamadı");
  }
}

void publishStatus(String status) {
  DynamicJsonDocument doc(256);
  doc["device"] = device_id;
  doc["status"] = status;
  doc["timestamp"] = millis();
  doc["wifi_rssi"] = WiFi.RSSI();
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  client.publish(mqtt_topic_status, jsonString.c_str());
} 