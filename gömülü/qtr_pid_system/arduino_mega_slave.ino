#include "config.h"
#include <Wire.h>
#include <QTRSensors.h>

// ========================================
// QTR/PID SİSTEMİ - ARDUINO MEGA SLAVE
// ========================================
// Bu sistem QTR8A sensörü ile çizgi takibi yapar
// ve BTS7960B motor sürücüleri ile PID kontrolü yapar

// QTR Sensör
QTRSensors qtr;
const uint8_t SensorCount = QTR_SENSOR_COUNT;
uint16_t sensorValues[SensorCount];

// PID Değişkenleri
float Kp = DEFAULT_KP;
float Ki = DEFAULT_KI;
float Kd = DEFAULT_KD;
int rightMaxSpeed = DEFAULT_MAX_SPEED;
int leftMaxSpeed = DEFAULT_MAX_SPEED;
int rightBaseSpeed = DEFAULT_BASE_SPEED;
int leftBaseSpeed = DEFAULT_BASE_SPEED;

// PID Hesaplama Değişkenleri
int lastError = 0;
float integral = 0;

// I²C'den gelen değerler
volatile int i2cTargetSpeed = DEFAULT_BASE_SPEED;
volatile float i2cKp = DEFAULT_KP;
volatile float i2cKi = DEFAULT_KI;
volatile float i2cKd = DEFAULT_KD;
volatile bool newDataReceived = false;

// Son motor hızları (I²C response için)
int lastRightSpeed = 0;
int lastLeftSpeed = 0;

// Sistem durumu
SystemState systemState = SYSTEM_INIT;
unsigned long lastPingTime = 0;
bool systemActive = false;

// ========================================
// I²C EVENT FONKSİYONLARI
// ========================================

void receiveEvent(int howMany) {
    String incoming = "";
    while (Wire.available()) {
        char c = Wire.read();
        incoming += c;
    }

    // Format: H:100;KP:1.2;KI:0.1;KD:0.2;ACT:1
    if (incoming.length() > 0) {
        parseI2CData(incoming);
        newDataReceived = true;
        lastPingTime = millis();
    }
}

void parseI2CData(String data) {
    int hIndex = data.indexOf("H:");
    int kpIndex = data.indexOf("KP:");
    int kiIndex = data.indexOf("KI:");
    int kdIndex = data.indexOf("KD:");
    int actIndex = data.indexOf("ACT:");

    if (hIndex >= 0) {
        int end = data.indexOf(';', hIndex);
        if (end == -1) end = data.length();
        i2cTargetSpeed = data.substring(hIndex + 2, end).toInt();
        i2cTargetSpeed = constrain(i2cTargetSpeed, 0, 255);
    }
    
    if (kpIndex >= 0) {
        int end = data.indexOf(';', kpIndex);
        if (end == -1) end = data.length();
        i2cKp = data.substring(kpIndex + 3, end).toFloat();
    }
    
    if (kiIndex >= 0) {
        int end = data.indexOf(';', kiIndex);
        if (end == -1) end = data.length();
        i2cKi = data.substring(kiIndex + 3, end).toFloat();
    }
    
    if (kdIndex >= 0) {
        int end = data.indexOf(';', kdIndex);
        if (end == -1) end = data.length();
        i2cKd = data.substring(kdIndex + 3, end).toFloat();
    }
    
    if (actIndex >= 0) {
        int end = data.indexOf(';', actIndex);
        if (end == -1) end = data.length();
        int active = data.substring(actIndex + 4, end).toInt();
        systemActive = (active == 1);
    }
}

void requestEvent() {
    // QTR sensör pozisyonu ve motor hızlarını gönder
    uint16_t position = qtr.readLineBlack(sensorValues);
    String response = "Q:" + String(position) + 
                     ";MR:" + String(lastRightSpeed) + 
                     ";ML:" + String(lastLeftSpeed) + 
                     ";ACT:" + String(systemActive ? 1 : 0);
    Wire.write(response.c_str());
}

// ========================================
// SETUP FONKSİYONU
// ========================================

void setup() {
    Serial.begin(DEBUG_SERIAL_BAUD);
    Serial.println("=== QTR/PID SİSTEMİ BAŞLATILIYOR ===");
    
    // 3 saniye bekleme
    Serial.println("Sistem 3 saniye bekleniyor...");
    delay(SYSTEM_STARTUP_DELAY);
    
    // QTR sensör kurulumu
    qtr.setTypeAnalog();
    qtr.setSensorPins(QTR_SENSOR_PINS, SensorCount);
    Serial.println("QTR sensör başlatıldı");

    // Motor pinleri kurulumu
    pinMode(RPWM_R, OUTPUT);
    pinMode(LPWM_R, OUTPUT);
    pinMode(R_EN_R, OUTPUT);
    pinMode(L_EN_R, OUTPUT);

    pinMode(RPWM_L, OUTPUT);
    pinMode(LPWM_L, OUTPUT);
    pinMode(R_EN_L, OUTPUT);
    pinMode(L_EN_L, OUTPUT);

    enableMotors();
    stopMotors();
    Serial.println("Motor sürücüleri başlatıldı");

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    
    Serial.println("Sensörler kalibre ediliyor...");
    
    // Kalibrasyon - sağa sola dönerek
    for (int i = 0; i < 200; i++) {
        if (i < 50) {
            // İlk 50 döngü sağa dön
            setMotorSpeeds(60, -60);
        } else if (i < 100) {
            // Sonraki 50 döngü sola dön
            setMotorSpeeds(-60, 60);
        } else if (i < 150) {
            // Tekrar sağa
            setMotorSpeeds(60, -60);
        } else {
            // Son olarak sola
            setMotorSpeeds(-60, 60);
        }
        
        qtr.calibrate();
        delay(20);
    }
    
    stopMotors();
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Kalibrasyon tamamlandı!");
    
    delay(1000);

    // I²C başlatma
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    Wire.setClock(I2C_CLOCK_SPEED);

    Serial.println("I2C başlatıldı. Sistem hazır!");
    systemState = SYSTEM_READY;
}

// ========================================
// ANA LOOP
// ========================================

void loop() {
    // I²C ping kontrolü (5 saniye timeout)
    if (millis() - lastPingTime > 5000 && lastPingTime > 0) {
        systemActive = false;
        stopMotors();
        Serial.println("I2C bağlantısı kesildi - sistem durduruldu");
    }
    
    // QTR sensörleri oku
    uint16_t position = qtr.readLineBlack(sensorValues);
    
    // Hata hesapla (0-7000 aralığında, 3500 merkez)
    int error = position - 3500;
    
    // PID hesaplama
    if (systemActive) {
        integral += error;
        integral = constrain(integral, -5000, 5000); // Integral windup önleme
        
        int derivative = error - lastError;
        
        float motorSpeed = i2cKp * error + i2cKi * integral + i2cKd * derivative;
        
        lastError = error;
        
        // Motor hızlarını hesapla
        int rightMotorSpeed = i2cTargetSpeed + motorSpeed;
        int leftMotorSpeed = i2cTargetSpeed - motorSpeed;
        
        // Hız limitlerini uygula
        rightMotorSpeed = constrain(rightMotorSpeed, -rightMaxSpeed, rightMaxSpeed);
        leftMotorSpeed = constrain(leftMotorSpeed, -leftMaxSpeed, leftMaxSpeed);
        
        // Motorları çalıştır
        setMotorSpeeds(rightMotorSpeed, leftMotorSpeed);
        
        // Son hızları kaydet (I²C için)
        lastRightSpeed = rightMotorSpeed;
        lastLeftSpeed = leftMotorSpeed;
    } else {
        stopMotors();
        lastRightSpeed = 0;
        lastLeftSpeed = 0;
        lastError = 0;
        integral = 0;
    }
    
    // Debug çıktısı
    if (newDataReceived) {
        Serial.print("Yeni I2C verisi - Hız: ");
        Serial.print(i2cTargetSpeed);
        Serial.print(", Kp: ");
        Serial.print(i2cKp);
        Serial.print(", Ki: ");
        Serial.print(i2cKi);
        Serial.print(", Kd: ");
        Serial.print(i2cKd);
        Serial.print(", Aktif: ");
        Serial.println(systemActive ? "EVET" : "HAYIR");
        newDataReceived = false;
    }
    
    // QTR sensör değerlerini yazdır (opsiyonel)
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 1000) {
        Serial.print("QTR Pozisyon: ");
        Serial.print(position);
        Serial.print(" (Merkez: 3500)");
        
        // Pozisyon göstergesi
        int deviation = position - 3500;
        if (abs(deviation) < 200) {
            Serial.println(" - ÇİZGİ ÜZERİNDE");
        } else if (deviation > 0) {
            Serial.println(" - SAĞA");
        } else {
            Serial.println(" - SOLA");
        }
        
        Serial.print("Motor Hızları - Sağ: ");
        Serial.print(lastRightSpeed);
        Serial.print(", Sol: ");
        Serial.println(lastLeftSpeed);
        
        lastDebugTime = millis();
    }
    
    delay(10); // Kısa bekleme
}

// ========================================
// MOTOR FONKSİYONLARI
// ========================================

void enableMotors() {
    digitalWrite(R_EN_R, HIGH);
    digitalWrite(L_EN_R, HIGH);
    digitalWrite(R_EN_L, HIGH);
    digitalWrite(L_EN_L, HIGH);
}

void stopMotors() {
    analogWrite(RPWM_R, 0);
    analogWrite(LPWM_R, 0);
    analogWrite(RPWM_L, 0);
    analogWrite(LPWM_L, 0);
}

void setMotorSpeeds(int rightSpeed, int leftSpeed) {
    // Sağ motor kontrolü
    if (rightSpeed >= 0) {
        analogWrite(RPWM_R, rightSpeed);
        analogWrite(LPWM_R, 0);
    } else {
        analogWrite(RPWM_R, 0);
        analogWrite(LPWM_L, -rightSpeed);
    }
    
    // Sol motor kontrolü
    if (leftSpeed >= 0) {
        analogWrite(RPWM_L, leftSpeed);
        analogWrite(LPWM_L, 0);
    } else {
        analogWrite(RPWM_L, 0);
        analogWrite(LPWM_L, -leftSpeed);
    }
}

// ========================================
// ACİL DURUM FONKSİYONLARI
// ========================================

void emergencyStop() {
    systemActive = false;
    stopMotors();
    lastError = 0;
    integral = 0;
    Serial.println("ACİL DURUM - MOTORLAR DURDURULDU!");
}

// ========================================
// SİSTEM DURUMU FONKSİYONLARI
// ========================================

bool isSystemHealthy() {
    // I²C bağlantısı kontrolü
    if (millis() - lastPingTime > 10000) {
        return false;
    }
    
    // QTR sensör kontrolü
    uint16_t position = qtr.readLineBlack(sensorValues);
    if (position == 0 || position == 7000) {
        return false;
    }
    
    return true;
}

String getSystemStatus() {
    String status = "QTR/PID System - ";
    status += systemActive ? "ACTIVE" : "STOPPED";
    status += " | I2C: ";
    status += (millis() - lastPingTime < 5000) ? "OK" : "TIMEOUT";
    status += " | Motors: ";
    status += (lastRightSpeed != 0 || lastLeftSpeed != 0) ? "RUNNING" : "STOPPED";
    return status;
} 