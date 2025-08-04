#include "config.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ArduinoJson.h>

// ========================================
// RFID SİSTEMİ - DENEYAP KART
// ========================================
// Bu sistem RC522 sensörü kullanarak RFID kartları okur
// ve MQTT ağına bağlanarak veri gönderir/alır

// RFID Sensör
MFRC522 mfrc522(RFID_SS_PIN, RFID_RST_PIN);

// WiFi ve MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Sistem durumu
SystemState systemState = SYSTEM_INIT;
int currentWiFiIndex = 0;
int currentMQTTIndex = 0;

// RFID verileri
String lastCardID = "";
String lastCardData = "";
unsigned long lastReadTime = 0;
unsigned long lastWriteTime = 0;
int totalReads = 0;
int totalWrites = 0;

// Yazma modu değişkenleri
String pendingWriteData = "";
String pendingCardId = "";
bool writeMode = false;
unsigned long writeModeTimeout = 0;

// JSON dokümanları
DynamicJsonDocument rfidDoc(1024);
DynamicJsonDocument statusDoc(512);

void setup() {
    Serial.begin(DEBUG_SERIAL_BAUD);
    Serial.println("=== RFID SİSTEMİ BAŞLATILIYOR ===");
    
    // 3 saniye bekleme
    Serial.println("Sistem 3 saniye bekleniyor...");
    delay(SYSTEM_STARTUP_DELAY);
    
    // SPI ve RFID başlatma
    SPI.begin();
    mfrc522.PCD_Init();
    Serial.println("RFID sensör başlatıldı");
    
    // WiFi bağlantısı
    connectToWiFi();
    
    // MQTT bağlantısı
    connectToMQTT();
    
    Serial.println("RFID sistemi hazır!");
    systemState = SYSTEM_READY;
}

void loop() {
    // WiFi bağlantı kontrolü
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi bağlantısı kesildi, yeniden bağlanılıyor...");
        connectToWiFi();
    }
    
    // MQTT bağlantı kontrolü
    if (!client.connected()) {
        Serial.println("MQTT bağlantısı kesildi, yeniden bağlanılıyor...");
        connectToMQTT();
    }
    
    client.loop();
    
    // Yazma modu timeout kontrolü
    if (writeMode && millis() - writeModeTimeout > RFID_WRITE_TIMEOUT) {
        writeMode = false;
        Serial.println("Yazma modu zaman aşımı");
        sendWriteStatus("timeout", pendingCardId, pendingWriteData);
    }
    
    // RFID kart kontrolü
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
        String cardID = getCardID();
        
        if (writeMode) {
            // Yazma modu - karta veri yaz
            writeToCard(cardID);
        } else {
            // Okuma modu - kart bilgilerini oku
            readFromCard(cardID);
        }
        
        // Kart okumayı sonlandır
        mfrc522.PICC_HaltA();
        mfrc522.PCD_StopCrypto1();
        
        delay(1000); // Çoklu okumayı önle
    }
    
    // Sistem durumu gönder
    static unsigned long lastStatusTime = 0;
    if (millis() - lastStatusTime > 5000) {
        sendSystemStatus();
        lastStatusTime = millis();
    }
}

// ========================================
// WiFi BAĞLANTI FONKSİYONLARI
// ========================================

void connectToWiFi() {
    systemState = SYSTEM_CONNECTING_WIFI;
    Serial.println("WiFi ağlarına bağlanılıyor...");
    
    // Aktif ağları dene
    for (int i = 0; i < WIFI_NETWORK_COUNT; i++) {
        if (wifiNetworks[i].isActive) {
            Serial.print("Bağlanılıyor: ");
            Serial.println(wifiNetworks[i].ssid);
            
            WiFi.begin(wifiNetworks[i].ssid, wifiNetworks[i].password);
            
            int attempts = 0;
            while (WiFi.status() != WL_CONNECTED && attempts < 20) {
                delay(500);
                Serial.print(".");
                attempts++;
            }
            
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println();
                Serial.print("WiFi bağlandı! IP: ");
                Serial.println(WiFi.localIP());
                currentWiFiIndex = i;
                return;
            } else {
                Serial.println("Bağlantı başarısız");
            }
        }
    }
    
    Serial.println("Hiçbir WiFi ağına bağlanılamadı!");
    systemState = SYSTEM_ERROR;
}

// ========================================
// MQTT BAĞLANTI FONKSİYONLARI
// ========================================

void connectToMQTT() {
    systemState = SYSTEM_CONNECTING_MQTT;
    Serial.println("MQTT sunucusuna bağlanılıyor...");
    
    // MQTT sunucularını dene
    for (int i = 0; i < MQTT_SERVER_COUNT; i++) {
        Serial.print("MQTT Broker: ");
        Serial.println(mqttServers[i].broker);
        
        client.setServer(mqttServers[i].broker, mqttServers[i].port);
        client.setCallback(callback);
        
        if (client.connect(mqttServers[i].clientId, mqttServers[i].username, mqttServers[i].password)) {
            Serial.println("MQTT bağlandı!");
            
            // Topic'lere abone ol
            client.subscribe(MQTT_RFID_WRITE_TOPIC);
            
            currentMQTTIndex = i;
            systemState = SYSTEM_READY;
            return;
        } else {
            Serial.print("MQTT bağlantı hatası: ");
            Serial.println(client.state());
        }
    }
    
    Serial.println("Hiçbir MQTT sunucusuna bağlanılamadı!");
    systemState = SYSTEM_ERROR;
}

void callback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    
    Serial.println("MQTT mesajı alındı: " + String(topic) + " - " + message);
    
    if (String(topic) == MQTT_RFID_WRITE_TOPIC) {
        handleWriteCommand(message);
    }
}

// ========================================
// RFID FONKSİYONLARI
// ========================================

String getCardID() {
    String cardID = "";
    for (byte i = 0; i < mfrc522.uid.size; i++) {
        cardID += String(mfrc522.uid.uidByte[i] < 0x10 ? "0" : "");
        cardID += String(mfrc522.uid.uidByte[i], HEX);
    }
    cardID.toUpperCase();
    return cardID;
}

void readFromCard(String cardID) {
    Serial.println("Kart okundu: " + cardID);
    
    // Kart verilerini oku
    String cardData = readCardData();
    
    // Verileri güncelle
    lastCardID = cardID;
    lastCardData = cardData;
    lastReadTime = millis();
    totalReads++;
    
    // JSON formatında MQTT'ye gönder
    rfidDoc.clear();
    rfidDoc["card_id"] = cardID;
    rfidDoc["data"] = cardData;
    rfidDoc["timestamp"] = millis();
    rfidDoc["device"] = "deneyap_rfid";
    rfidDoc["action"] = "read";
    rfidDoc["total_reads"] = totalReads;
    
    String jsonMessage;
    serializeJson(rfidDoc, jsonMessage);
    
    client.publish(MQTT_RFID_READ_TOPIC, jsonMessage.c_str());
    Serial.println("Okuma verisi gönderildi");
}

void writeToCard(String cardID) {
    Serial.println("Karta yazılıyor: " + cardID);
    
    bool success = writeCardData(pendingWriteData);
    
    if (success) {
        Serial.println("Yazma başarılı!");
        totalWrites++;
        lastWriteTime = millis();
        sendWriteStatus("success", cardID, pendingWriteData);
    } else {
        Serial.println("Yazma başarısız!");
        sendWriteStatus("failed", cardID, pendingWriteData);
    }
    
    // Yazma modunu kapat
    writeMode = false;
    pendingWriteData = "";
    pendingCardId = "";
}

String readCardData() {
    // MIFARE karttan veri okuma (Sector 1, Block 4)
    byte sector = 1;
    byte blockAddr = 4;
    byte trailerBlock = 7;
    MFRC522::StatusCode status;
    byte buffer[18];
    byte size = sizeof(buffer);
    
    // Varsayılan anahtar kullan
    MFRC522::MIFARE_Key key;
    for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;
    
    // Sektörü doğrula
    status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, &(mfrc522.uid));
    if (status != MFRC522::STATUS_OK) {
        Serial.println("Doğrulama başarısız");
        return "";
    }
    
    // Veriyi oku
    status = mfrc522.MIFARE_Read(blockAddr, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.println("Okuma başarısız");
        return "";
    }
    
    // String'e çevir
    String data = "";
    for (int i = 0; i < 16; i++) {
        if (buffer[i] != 0) {
            data += (char)buffer[i];
        }
    }
    
    return data;
}

bool writeCardData(String data) {
    // MIFARE karta veri yazma (Sector 1, Block 4)
    byte sector = 1;
    byte blockAddr = 4;
    byte trailerBlock = 7;
    MFRC522::StatusCode status;
    byte dataBlock[16];
    
    // Veriyi byte array'e çevir
    memset(dataBlock, 0, 16);
    for (int i = 0; i < data.length() && i < 16; i++) {
        dataBlock[i] = data[i];
    }
    
    // Varsayılan anahtar kullan
    MFRC522::MIFARE_Key key;
    for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;
    
    // Sektörü doğrula
    status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, &(mfrc522.uid));
    if (status != MFRC522::STATUS_OK) {
        Serial.println("Doğrulama başarısız");
        return false;
    }
    
    // Veriyi yaz
    status = mfrc522.MIFARE_Write(blockAddr, dataBlock, 16);
    if (status != MFRC522::STATUS_OK) {
        Serial.println("Yazma başarısız");
        return false;
    }
    
    return true;
}

// ========================================
// MQTT KOMUT İŞLEME
// ========================================

void handleWriteCommand(String message) {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, message);
    
    String action = doc["action"];
    if (action == "write") {
        pendingWriteData = doc["data"].as<String>();
        pendingCardId = doc["card_id"].as<String>();
        writeMode = true;
        writeModeTimeout = millis();
        
        Serial.println("Yazma modu aktif - Kart bekleniliyor...");
        Serial.println("Yazılacak veri: " + pendingWriteData);
    }
}

void sendWriteStatus(String status, String cardId, String data) {
    DynamicJsonDocument doc(1024);
    doc["status"] = status;
    doc["card_id"] = cardId;
    doc["data"] = data;
    doc["timestamp"] = millis();
    doc["device"] = "deneyap_rfid";
    
    String jsonMessage;
    serializeJson(doc, jsonMessage);
    
    client.publish(MQTT_RFID_STATUS_TOPIC, jsonMessage.c_str());
    Serial.println("Yazma durumu gönderildi: " + status);
}

void sendSystemStatus() {
    statusDoc.clear();
    statusDoc["device"] = "deneyap_rfid";
    statusDoc["state"] = getSystemStateString(systemState);
    statusDoc["wifi_status"] = getWiFiStatus();
    statusDoc["wifi_ssid"] = WiFi.SSID();
    statusDoc["mqtt_connected"] = client.connected();
    statusDoc["total_reads"] = totalReads;
    statusDoc["total_writes"] = totalWrites;
    statusDoc["last_card_id"] = lastCardID;
    statusDoc["write_mode"] = writeMode;
    statusDoc["timestamp"] = millis();
    
    String jsonMessage;
    serializeJson(statusDoc, jsonMessage);
    
    client.publish(MQTT_SYSTEM_STATUS_TOPIC, jsonMessage.c_str());
} 