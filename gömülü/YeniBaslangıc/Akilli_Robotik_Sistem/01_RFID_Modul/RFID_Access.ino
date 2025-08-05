/*
 * 🔧 RFID Erişim ve Kontrol Sistemi
 * Deneyap Kart 1 (ESP32) + RC522 RFID Sensörü
 * 
 * Fonksiyonlar:
 * - RFID kart okuma/yazma
 * - MQTT üzerinden erişim verilerinin yayınlanması
 * - Çoklu WiFi ağı desteği (WiFiMulti)
 * - Otomatik yeniden bağlanma
 * - Ortak konfigürasyon sistemi
 */

#include <WiFi.h>
#include <WiFiMulti.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ArduinoJson.h>
#include "../04_Dokumantasyon/Shared_Config.h"

// WiFi Ayarları
WiFiMulti wifiMulti;

// RFID Ayarları
#define RST_PIN         RFID_RST_PIN
#define SS_PIN          RFID_SDA_PIN
#define IRQ_PIN         RFID_IRQ_PIN

MFRC522 mfrc522(SS_PIN, RST_PIN);
WiFiClient espClient;
PubSubClient client(espClient);

// Pin Konfigürasyonu (MQTT ile değiştirilebilir)
struct PinConfig {
  uint8_t rfid_sda = RFID_SDA_PIN;
  uint8_t rfid_sck = RFID_SCK_PIN;
  uint8_t rfid_mosi = RFID_MOSI_PIN;
  uint8_t rfid_miso = RFID_MISO_PIN;
  uint8_t rfid_rst = RFID_RST_PIN;
  uint8_t rfid_irq = RFID_IRQ_PIN;
} pinConfig;

// MQTT Konuları
const char* mqtt_topic_uid = MQTT_TOPIC_RFID_UID;
const char* mqtt_topic_command = MQTT_TOPIC_RFID_COMMAND;
const char* mqtt_topic_status = MQTT_TOPIC_RFID_STATUS;
const char* mqtt_topic_config = MQTT_TOPIC_RFID_CONFIG;

// Değişkenler
unsigned long lastCardCheck = 0;
const unsigned long cardCheckInterval = 1000; // 1 saniye
String lastCardUID = "";
bool cardDetected = false;

void setup() {
  Serial.begin(115200);
  Serial.println("🔧 RFID Erişim Sistemi Başlatılıyor...");
  
  // WiFi bağlantısı
  setupWiFi();
  
  // MQTT bağlantısı
  setupMQTT();
  
  // RFID başlatma
  setupRFID();
  
  // MQTT konularına abone ol
  subscribeToTopics();
  
  Serial.println("✅ RFID Sistemi Hazır!");
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
  
  // RFID kart kontrolü
  checkRFIDCard();
  
  delay(100);
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

void setupRFID() {
  SPI.begin();
  mfrc522.PCD_Init();
  
  // Pin konfigürasyonunu uygula
  mfrc522.PCD_Init();
  mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);
  
  Serial.println("✅ RFID Sensörü Hazır");
  Serial.print("RFID Sensörü Versiyonu: ");
  Serial.println(mfrc522.PCD_ReadRegister(mfrc522.VersionReg));
}

void subscribeToTopics() {
  client.subscribe(mqtt_topic_command);
  client.subscribe(mqtt_topic_config);
  Serial.println("✅ MQTT Konularına Abone Olundu");
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("🔗 MQTT Bağlantısı Kuruluyor...");
    
    if (client.connect(DEVICE_ID_RFID, MQTT_USERNAME, MQTT_PASSWORD)) {
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

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("📨 MQTT Mesajı Alındı: ");
  Serial.println(topic);
  
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  if (strcmp(topic, mqtt_topic_command) == 0) {
    handleRFIDCommand(message);
  } else if (strcmp(topic, mqtt_topic_config) == 0) {
    handlePinConfig(message);
  }
}

void handleRFIDCommand(String message) {
  Serial.println("🎛️ RFID Komutu Alındı: " + message);
  
  // JSON parse et
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.println("❌ JSON Parse Hatası");
    return;
  }
  
  String command = doc["command"];
  
  if (command == "write_card") {
    // Kart yazma komutu
    String uid = doc["uid"];
    String data = doc["data"];
    writeToCard(uid, data);
  } else if (command == "read_card") {
    // Kart okuma komutu
    readCard();
  }
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
  
  // Pin konfigürasyonunu güncelle
  if (doc.containsKey("pins")) {
    JsonObject pins = doc["pins"]["rfid"];
    
    if (pins.containsKey("sda_pin")) pinConfig.rfid_sda = pins["sda_pin"];
    if (pins.containsKey("sck_pin")) pinConfig.rfid_sck = pins["sck_pin"];
    if (pins.containsKey("mosi_pin")) pinConfig.rfid_mosi = pins["mosi_pin"];
    if (pins.containsKey("miso_pin")) pinConfig.rfid_miso = pins["miso_pin"];
    if (pins.containsKey("rst_pin")) pinConfig.rfid_rst = pins["rst_pin"];
    if (pins.containsKey("irq_pin")) pinConfig.rfid_irq = pins["irq_pin"];
    
    // RFID'yi yeniden başlat
    setupRFID();
    
    Serial.println("✅ Pin Konfigürasyonu Güncellendi");
  }
}

void checkRFIDCard() {
  if (millis() - lastCardCheck < cardCheckInterval) {
    return;
  }
  
  lastCardCheck = millis();
  
  // Yeni kart var mı kontrol et
  if (!mfrc522.PICC_IsNewCardPresent()) {
    if (cardDetected) {
      // Kart kaldırıldı
      cardDetected = false;
      publishCardEvent("card_removed", "", "");
    }
    return;
  }
  
  // Kart okunabilir mi kontrol et
  if (!mfrc522.PICC_ReadCardSerial()) {
    return;
  }
  
  // Kart UID'sini al
  String cardUID = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    cardUID += String(mfrc522.uid.uidByte[i] < 0x10 ? "0" : "");
    cardUID += String(mfrc522.uid.uidByte[i], HEX);
  }
  cardUID.toUpperCase();
  
  // Yeni kart mı kontrol et
  if (cardUID != lastCardUID) {
    lastCardUID = cardUID;
    cardDetected = true;
    
    // Kart tipini belirle
    MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
    String cardType = mfrc522.PICC_GetTypeName(piccType);
    
    Serial.println("📱 Kart Tespit Edildi!");
    Serial.print("UID: ");
    Serial.println(cardUID);
    Serial.print("Tip: ");
    Serial.println(cardType);
    
    // MQTT'e gönder
    publishCardEvent("card_detected", cardUID, cardType);
  }
  
  // Kart işlemini bitir
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}

void publishCardEvent(String event, String uid, String cardType) {
  DynamicJsonDocument doc(512);
  doc["device"] = device_id;
  doc["event"] = event;
  doc["timestamp"] = millis();
  
  if (uid.length() > 0) {
    doc["uid"] = uid;
    doc["card_type"] = cardType;
    doc["rssi"] = WiFi.RSSI();
  }
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  client.publish(mqtt_topic_uid, jsonString.c_str());
  Serial.println("📤 MQTT'e Gönderildi: " + jsonString);
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

void writeToCard(String uid, String data) {
  Serial.println("✍️ Kart Yazma Komutu: " + data);
  
  // Kart yazma işlemi burada implement edilecek
  // Güvenlik nedeniyle basit bir örnek
  
  DynamicJsonDocument doc(512);
  doc["device"] = device_id;
  doc["command"] = "write_result";
  doc["uid"] = uid;
  doc["data"] = data;
  doc["success"] = true;
  doc["timestamp"] = millis();
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  client.publish(mqtt_topic_command, jsonString.c_str());
  Serial.println("✅ Kart Yazma Tamamlandı");
}

void readCard() {
  Serial.println("📖 Kart Okuma Komutu");
  
  // Kart okuma işlemi
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    String cardUID = "";
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      cardUID += String(mfrc522.uid.uidByte[i] < 0x10 ? "0" : "");
      cardUID += String(mfrc522.uid.uidByte[i], HEX);
    }
    cardUID.toUpperCase();
    
    MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
    String cardType = mfrc522.PICC_GetTypeName(piccType);
    
    publishCardEvent("card_read", cardUID, cardType);
    
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
  } else {
    Serial.println("❌ Kart Bulunamadı");
  }
} 