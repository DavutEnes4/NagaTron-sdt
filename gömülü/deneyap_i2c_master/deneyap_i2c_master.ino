#include <Wire.h>

// I²C Slave adresi (Arduino Mega)
#define SLAVE_ADDRESS 8

// Kontrol parametreleri
int targetSpeed = 50;
float pidKp = 0.4;
float pidKi = 0.0;
float pidKd = 2.0;

// Alınan veriler
int qtrPosition = 3500;
int rightMotorSpeed = 0;
int leftMotorSpeed = 0;

// Timing kontrolü
unsigned long lastSendTime = 0;
unsigned long lastRequestTime = 0;
const unsigned long SEND_INTERVAL = 100;    // 100ms'de bir parametre gönder
const unsigned long REQUEST_INTERVAL = 50;  // 50ms'de bir veri iste

// Buton ve potansiyometre pinleri
#define SPEED_POT A0      // Hız potansiyometresi
#define KP_POT A1         // Kp potansiyometresi
#define KD_POT A2         // Kd potansiyometresi
#define START_BUTTON 2    // Başlat/Durdur butonu
#define RESET_BUTTON 3    // Reset butonu

// Durum değişkenleri
bool systemActive = false;
bool lastButtonState = HIGH;
bool dataReceived = false;

void setup() {
  Serial.begin(115200);
  Serial.println("Deneyap Kart - QTR Controller Starting...");
  
  // I²C Master olarak başlat
  Wire.begin();
  Wire.setClock(100000); // 100kHz I²C hızı
  
  // Pin kurulumları
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(RESET_BUTTON, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Analog pinler otomatik olarak input
  
  Serial.println("System ready. Press START button to begin.");
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
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
  
  // Serial monitor çıktısı
  printStatus();
  
  delay(10);
}

void handleButtons() {
  // START/STOP butonu
  bool currentButtonState = digitalRead(START_BUTTON);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    systemActive = !systemActive;
    digitalWrite(LED_BUILTIN, systemActive ? HIGH : LOW);
    Serial.println(systemActive ? "System STARTED" : "System STOPPED");
    
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
    Serial.println("Parameters RESET");
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

void sendParametersToSlave() {
  // Format: H:100;KP:1.2;KI:0.1;KD:0.2
  String data = "H:" + String(targetSpeed) + 
                ";KP:" + String(pidKp, 2) + 
                ";KI:" + String(pidKi, 2) + 
                ";KD:" + String(pidKd, 2);
  
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(data.c_str());
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.print("I2C Send Error: ");
    Serial.println(error);
  }
}

void requestDataFromSlave() {
  Wire.requestFrom(SLAVE_ADDRESS, 32); // Maksimum 32 byte iste
  
  String response = "";
  while (Wire.available()) {
    char c = Wire.read();
    response += c;
  }
  
  if (response.length() > 0) {
    parseSlaveResponse(response);
    dataReceived = true;
  } else {
    dataReceived = false;
  }
}

void parseSlaveResponse(String data) {
  // Format: Q:3500;MR:50;ML:50
  int qIndex = data.indexOf("Q:");
  int mrIndex = data.indexOf("MR:");
  int mlIndex = data.indexOf("ML:");
  
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
}

void resetParameters() {
  targetSpeed = 50;
  pidKp = 0.4;
  pidKi = 0.0;
  pidKd = 2.0;
  systemActive = false;
  digitalWrite(LED_BUILTIN, LOW);
}

void printStatus() {
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 500) { // Her 500ms'de bir yazdır
    Serial.println("=== QTR LINE FOLLOWER STATUS ===");
    Serial.print("System: ");
    Serial.println(systemActive ? "ACTIVE" : "STOPPED");
    Serial.print("Target Speed: ");
    Serial.println(targetSpeed);
    Serial.print("PID - Kp: ");
    Serial.print(pidKp, 2);
    Serial.print(", Ki: ");
    Serial.print(pidKi, 2);
    Serial.print(", Kd: ");
    Serial.println(pidKd, 2);
    
    if (dataReceived) {
      Serial.print("QTR Position: ");
      Serial.print(qtrPosition);
      Serial.print(" (Center: 3500)");
      
      // Pozisyon göstergesi
      int deviation = qtrPosition - 3500;
      if (abs(deviation) < 200) {
        Serial.println(" - ON LINE");
      } else if (deviation > 0) {
        Serial.println(" - RIGHT");
      } else {
        Serial.println(" - LEFT");
      }
      
      Serial.print("Motor Speeds - Right: ");
      Serial.print(rightMotorSpeed);
      Serial.print(", Left: ");
      Serial.println(leftMotorSpeed);
    } else {
      Serial.println("No data from slave!");
    }
    Serial.println("=================================");
    lastPrintTime = millis();
  }
}

// Interrupt fonksiyonları (gelecekte kullanılabilir)
void emergencyStop() {
  systemActive = false;
  targetSpeed = 0;
  digitalWrite(LED_BUILTIN, LOW);
  sendParametersToSlave();
}