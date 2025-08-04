#include "config.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ArduinoJson.h>

// ========================================
// QTR/PID SİSTEMİ - DENEYAP KART MASTER
// ========================================
// Bu sistem I²C protokolü ile Arduino Mega'yı yönetir
// ve MQTT sistemine bağlanarak veri gönderir/alır

// WiFi ve MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// I²C Master
#define SLAVE_ADDRESS I2C_SLAVE_ADDRESS

// Sistem durumu
SystemState systemState = SYSTEM_INIT;
int currentWiFiIndex = 0;
int currentMQTTIndex = 0;

// QTR/PID verileri
int qtrPosition = 3500;
int rightMotorSpeed = 0;
int leftMotorSpeed = 0;
bool systemActive = false;

// PID parametreleri
int targetSpeed = DEFAULT_BASE_SPEED;
float pidKp = DEFAULT_KP;
float pidKi = DEFAULT_KI;
float pidKd = DEFAULT_KD;

// Timing kontrolü
unsigned long lastSendTime = 0;
unsigned long lastRequestTime = 0;
const unsigned long SEND_INTERVAL = PID_UPDATE_INTERVAL;
const unsigned long REQUEST_INTERVAL = QTR_UPDATE_INTERVAL;

// Buton ve potansiyometre pinleri
#define SPEED_POT A0      // Hız potansiyometresi
#define KP_POT A1         // Kp potansiyometresi
#define KD_POT A2         // Kd potansiyometresi
#define START_BUTTON 2    // Başlat/Durdur butonu
#define RESET_BUTTON 3    // Reset butonu

// Durum değişkenleri
bool lastButtonState = HIGH;
bool dataReceived = false;
unsigned long lastI2CResponse = 0;

// JSON dokümanları
DynamicJsonDocument qtrDoc(1024);
DynamicJsonDocument statusDoc(512);

void setup() {
    Serial.begin(DEBUG_SERIAL_BAUD);
    Serial.println("=== QTR/PID MASTER SİSTEMİ BAŞLATILIYOR ===");
    
    // 3 saniye bekleme
    Serial.println("Sistem 3 saniye bekleniyor...");
    delay(SYSTEM_STARTUP_DELAY);
    
    // I²C Master olarak başlat
    Wire.begin();
    Wire.setClock(I2C_CLOCK_SPEED);
    Serial.println("I2C Master başlatıldı");
    
    // Pin kurulumları
    pinMode(START_BUTTON, INPUT_PULLUP);
    pinMode(RESET_BUTTON, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);
    
    // Analog pinler otomatik olarak input
    Serial.println("Pinler kuruldu");
    
    // WiFi bağlantısı
    connectToWiFi();
    
    // MQTT bağlantısı
    connectToMQTT();
    
    Serial.println("QTR/PID Master sistemi hazır!");
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
    
    // Buton kontrolü
    handleButtons();
    
    // Potansiyometre değerlerini oku
    readPotentiometers();
    
    // Aktif durumdaysa parametreleri gönder
    if (systemActive && (millis() - lastSendTime > SEND_INTERVAL)) {
        sendParametersToSlave();
        lastSendTime = millis();
    }
    
    // Slave'den veri iste
    if (millis() - lastRequestTime > REQUEST_INTERVAL) {
        requestDataFromSlave();
        lastRequestTime = millis();
    }
    
    // Sistem durumu gönder
    static unsigned long lastStatusTime = 0;
    if (millis() - lastStatusTime > 5000) {
        sendSystemStatus();
        lastStatusTime = millis();
    }
    
    // Serial monitor çıktısı
    printStatus();
    
    delay(10);
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
            client.subscribe(MQTT_MOTOR_CONTROL_TOPIC);
            client.subscribe(MQTT_PID_CONFIG_TOPIC);
            
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
    
    if (String(topic) == MQTT_MOTOR_CONTROL_TOPIC) {
        handleMotorControl(message);
    } else if (String(topic) == MQTT_PID_CONFIG_TOPIC) {
        handlePIDConfig(message);
    }
}

// ========================================
// I²C FONKSİYONLARI
// ========================================

void sendParametersToSlave() {
    // Format: H:100;KP:1.2;KI:0.1;KD:0.2;ACT:1
    String data = "H:" + String(targetSpeed) + 
                  ";KP:" + String(pidKp, 2) + 
                  ";KI:" + String(pidKi, 2) + 
                  ";KD:" + String(pidKd, 2) + 
                  ";ACT:" + String(systemActive ? 1 : 0);
    
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write(data.c_str());
    byte error = Wire.endTransmission();
    
    if (error != 0) {
        Serial.print("I2C Send Error: ");
        Serial.println(error);
    }
}

void requestDataFromSlave() {
    Wire.requestFrom(SLAVE_ADDRESS, 64); // Maksimum 64 byte iste
    
    String response = "";
    while (Wire.available()) {
        char c = Wire.read();
        response += c;
    }
    
    if (response.length() > 0) {
        parseSlaveResponse(response);
        dataReceived = true;
        lastI2CResponse = millis();
    } else {
        dataReceived = false;
    }
}

void parseSlaveResponse(String data) {
    // Format: Q:3500;MR:50;ML:50;ACT:1
    int qIndex = data.indexOf("Q:");
    int mrIndex = data.indexOf("MR:");
    int mlIndex = data.indexOf("ML:");
    int actIndex = data.indexOf("ACT:");
    
    if (qIndex >= 0) {
        int end = data.indexOf(';', qIndex);
        if (end == -1) end = data.length();
        qtrPosition = data.substring(qIndex + 2, end).toInt();
    }
    
    if (mrIndex >= 0) {
        int end = data.indexOf(';', mrIndex);
        if (end == -1) end = data.length();
        rightMotorSpeed = data.substring(mrIndex + 3, end).toInt();
    }
    
    if (mlIndex >= 0) {
        int end = data.indexOf(';', mlIndex);
        if (end == -1) end = data.length();
        leftMotorSpeed = data.substring(mlIndex + 3, end).toInt();
    }
    
    if (actIndex >= 0) {
        int end = data.indexOf(';', actIndex);
        if (end == -1) end = data.length();
        int active = data.substring(actIndex + 4, end).toInt();
        systemActive = (active == 1);
    }
}

// ========================================
// KONTROL FONKSİYONLARI
// ========================================

void handleButtons() {
    // START/STOP butonu
    bool currentButtonState = digitalRead(START_BUTTON);
    if (lastButtonState == HIGH && currentButtonState == LOW) {
        systemActive = !systemActive;
        digitalWrite(LED_BUILTIN, systemActive ? HIGH : LOW);
        Serial.println(systemActive ? "Sistem BAŞLATILDI" : "Sistem DURDURULDU");
        
        if (!systemActive) {
            // Sistem durdurulduğunda hızı sıfırla
            targetSpeed = 0;
            sendParametersToSlave();
        }
    }
    lastButtonState = currentButtonState;
    
    // RESET butonu
    if (digitalRead(RESET_BUTTON) == LOW) {
        resetParameters();
        Serial.println("Parametreler SIFIRLANDI");
        delay(200); // Debounce
    }
}

void readPotentiometers() {
    if (systemActive) {
        // Hız potansiyometresi (0-150 aralığında)
        int speedRaw = analogRead(SPEED_POT);
        targetSpeed = map(speedRaw, 0, 4095, 20, 150);
        
        // Kp potansiyometresi (0.1 - 2.0 aralığında)
        int kpRaw = analogRead(KP_POT);
        pidKp = map(kpRaw, 0, 4095, 10, 200) / 100.0;
        
        // Kd potansiyometresi (0.5 - 5.0 aralığında)
        int kdRaw = analogRead(KD_POT);
        pidKd = map(kdRaw, 0, 4095, 50, 500) / 100.0;
    } else {
        targetSpeed = 0;
    }
}

void resetParameters() {
    targetSpeed = DEFAULT_BASE_SPEED;
    pidKp = DEFAULT_KP;
    pidKi = DEFAULT_KI;
    pidKd = DEFAULT_KD;
    systemActive = false;
    digitalWrite(LED_BUILTIN, LOW);
}

// ========================================
// MQTT KOMUT İŞLEME
// ========================================

void handleMotorControl(String message) {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, message);
    
    String action = doc["action"];
    int speed = doc["speed"] | targetSpeed;
    
    if (action == "start") {
        systemActive = true;
        targetSpeed = speed;
        Serial.println("Motor kontrolü: BAŞLAT");
    } else if (action == "stop") {
        systemActive = false;
        targetSpeed = 0;
        Serial.println("Motor kontrolü: DUR");
    } else if (action == "emergency_stop") {
        systemActive = false;
        targetSpeed = 0;
        Serial.println("Motor kontrolü: ACİL DURUM");
    }
    
    sendParametersToSlave();
}

void handlePIDConfig(String message) {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, message);
    
    if (doc.containsKey("kp")) pidKp = doc["kp"];
    if (doc.containsKey("ki")) pidKi = doc["ki"];
    if (doc.containsKey("kd")) pidKd = doc["kd"];
    if (doc.containsKey("speed")) targetSpeed = doc["speed"];
    
    Serial.println("PID konfigürasyonu güncellendi");
    sendParametersToSlave();
}

// ========================================
// VERİ GÖNDERME FONKSİYONLARI
// ========================================

void sendQTRData() {
    qtrDoc.clear();
    qtrDoc["position"] = qtrPosition;
    qtrDoc["right_speed"] = rightMotorSpeed;
    qtrDoc["left_speed"] = leftMotorSpeed;
    qtrDoc["system_active"] = systemActive;
    qtrDoc["timestamp"] = millis();
    qtrDoc["device"] = "deneyap_qtr_master";
    
    String jsonMessage;
    serializeJson(qtrDoc, jsonMessage);
    
    client.publish(MQTT_QTR_DATA_TOPIC, jsonMessage.c_str());
}

void sendSystemStatus() {
    statusDoc.clear();
    statusDoc["device"] = "deneyap_qtr_master";
    statusDoc["state"] = getSystemStateString(systemState);
    statusDoc["wifi_status"] = getWiFiStatus();
    statusDoc["wifi_ssid"] = WiFi.SSID();
    statusDoc["mqtt_connected"] = client.connected();
    statusDoc["i2c_connected"] = dataReceived;
    statusDoc["system_active"] = systemActive;
    statusDoc["target_speed"] = targetSpeed;
    statusDoc["pid_kp"] = pidKp;
    statusDoc["pid_ki"] = pidKi;
    statusDoc["pid_kd"] = pidKd;
    statusDoc["qtr_position"] = qtrPosition;
    statusDoc["timestamp"] = millis();
    
    String jsonMessage;
    serializeJson(statusDoc, jsonMessage);
    
    client.publish(MQTT_SYSTEM_STATUS_TOPIC, jsonMessage.c_str());
}

// ========================================
// DEBUG FONKSİYONLARI
// ========================================

void printStatus() {
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime > 1000) { // Her 1 saniyede bir yazdır
        Serial.println("=== QTR/PID MASTER SİSTEM DURUMU ===");
        Serial.print("Sistem: ");
        Serial.println(systemActive ? "AKTİF" : "DURDURULDU");
        Serial.print("Hedef Hız: ");
        Serial.println(targetSpeed);
        Serial.print("PID - Kp: ");
        Serial.print(pidKp, 2);
        Serial.print(", Ki: ");
        Serial.print(pidKi, 2);
        Serial.print(", Kd: ");
        Serial.println(pidKd, 2);
        
        if (dataReceived) {
            Serial.print("QTR Pozisyon: ");
            Serial.print(qtrPosition);
            Serial.print(" (Merkez: 3500)");
            
            // Pozisyon göstergesi
            int deviation = qtrPosition - 3500;
            if (abs(deviation) < 200) {
                Serial.println(" - ÇİZGİ ÜZERİNDE");
            } else if (deviation > 0) {
                Serial.println(" - SAĞA");
            } else {
                Serial.println(" - SOLA");
            }
            
            Serial.print("Motor Hızları - Sağ: ");
            Serial.print(rightMotorSpeed);
            Serial.print(", Sol: ");
            Serial.println(leftMotorSpeed);
        } else {
            Serial.println("Slave'den veri alınamıyor!");
        }
        Serial.println("=====================================");
        lastPrintTime = millis();
    }
} 