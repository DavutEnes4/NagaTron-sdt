#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ArduinoJson.h>

// WiFi ayarları
const char* ssid = "Furkan's Galaxy S20 FE";
const char* password = "12345678";

// MQTT ayarları
const char* mqtt_server = "nagatron-sdt.local";
const int mqtt_port = 1883;
const char* mqtt_read_topic = "rfid/data";     // Okuma verileri
const char* mqtt_write_topic = "rfid/write";   // Yazma komutları
const char* mqtt_status_topic = "rfid/status"; // Yazma durumu

// RFID pinleri (Deneyap kart için)
#define RST_PIN         D9
#define SS_PIN          D10

MFRC522 mfrc522(SS_PIN, RST_PIN);
WiFiClient espClient;
PubSubClient client(espClient);

// Yazma komutu için değişkenler
String pendingWriteData = "";
String pendingCardId = "";
bool writeMode = false;
unsigned long writeModeTimeout = 0;

void setup() {
    Serial.begin(115200);
    SPI.begin();
    mfrc522.PCD_Init();
    
    // WiFi bağlantısı
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi bağlandı!");
    
    // MQTT bağlantısı
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    
    Serial.println("RFID okuyucu/yazıcı hazır!");
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
    
    // Yazma modu timeout kontrolü (30 saniye)
    if (writeMode && millis() - writeModeTimeout > 30000) {
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
}

void callback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    
    Serial.println("MQTT mesajı alındı: " + String(topic) + " - " + message);
    
    if (String(topic) == mqtt_write_topic) {
        handleWriteCommand(message);
    }
}

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
    
    // Kart verilerini oku (Sector 1, Block 4'ten)
    String cardData = readCardData();
    
    // JSON formatında MQTT'ye gönder
    DynamicJsonDocument doc(1024);
    doc["card_id"] = cardID;
    doc["data"] = cardData;
    doc["timestamp"] = millis();
    doc["device"] = "deneyap_kart";
    
    String jsonMessage;
    serializeJson(doc, jsonMessage);
    
    client.publish(mqtt_read_topic, jsonMessage.c_str());
    Serial.println("Okuma verisi gönderildi");
}

void writeToCard(String cardID) {
    Serial.println("Karta yazılıyor: " + cardID);
    
    bool success = writeCardData(pendingWriteData);
    
    if (success) {
        Serial.println("Yazma başarılı!");
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

void sendWriteStatus(String status, String cardId, String data) {
    DynamicJsonDocument doc(1024);
    doc["status"] = status;
    doc["card_id"] = cardId;
    doc["data"] = data;
    doc["timestamp"] = millis();
    
    String jsonMessage;
    serializeJson(doc, jsonMessage);
    
    client.publish(mqtt_status_topic, jsonMessage.c_str());
    Serial.println("Yazma durumu gönderildi: " + status);
}

void reconnect() {
    while (!client.connected()) {
        Serial.print("MQTT bağlantısı deneniyor...");
        if (client.connect("DeneyapRFID")) {
            Serial.println("MQTT bağlandı!");
            client.subscribe(mqtt_write_topic);
        } else {
            Serial.print("Hata, kod: ");
            Serial.print(client.state());
            Serial.println(" 5 saniye sonra tekrar denenecek");
            delay(5000);
        }
    }
}