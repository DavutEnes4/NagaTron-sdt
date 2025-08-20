#include <Wire.h>
#include <QTRSensors.h>

// ---- PID Ayarları ----
float Kp = 0.4;
float Kd = 2.0;
float Ki = 0.0;
int rightMaxSpeed = 120;   // Maksimum hız
int leftMaxSpeed = 120;
int rightBaseSpeed = 50;   // Temel hız
int leftBaseSpeed = 50;

// ---- Motor Pinleri (BTS7960B) ----
// Sağ Motor
#define RPWM_R 10   // Sağ motor ileri PWM
#define LPWM_R 9  // Sağ motor geri PWM
#define R_EN_R 7   // Sağ motor R_EN
#define L_EN_R 8   // Sağ motor L_EN

// Sol Motor
#define RPWM_L 6   // Sol motor ileri PWM
#define LPWM_L 5   // Sol motor geri PWM
#define R_EN_L 3   // Sol motor R_EN
#define L_EN_L 4   // Sol motor L_EN

// ---- QTR Sensör Ayarları ----
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int lastError = 0;
float integral = 0;

// I²C'den gelen değerler
volatile int i2cTargetSpeed = 50;
volatile float i2cKp = 0.4;
volatile float i2cKi = 0.0;
volatile float i2cKd = 2.0;
volatile bool newDataReceived = false;

// Son motor hızları (I²C response için)
int lastRightSpeed = 0;
int lastLeftSpeed = 0;

// ---------------- I²C Event Fonksiyonları ----------------
void receiveEvent(int howMany) {
  String incoming = "";
  while (Wire.available()) {
    char c = Wire.read();
    incoming += c;
  }

  // Format: H:100;KP:1.2;KI:0.1;KD:0.2
  if (incoming.length() > 0) {
    parseI2CData(incoming);
    newDataReceived = true;
  }
}

void parseI2CData(String data) {
  int hIndex = data.indexOf("H:");
  int kpIndex = data.indexOf("KP:");
  int kiIndex = data.indexOf("KI:");
  int kdIndex = data.indexOf("KD:");

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
}

void requestEvent() {
  // QTR sensör pozisyonu ve motor hızlarını gönder
  uint16_t position = qtr.readLineBlack(sensorValues);
  String response = "Q:" + String(position) + ";MR:" + String(lastRightSpeed) + ";ML:" + String(lastLeftSpeed);
  Wire.write(response.c_str());
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(9600);
  Serial.println("QTR Line Follower Starting...");

  // QTR sensör kurulumu
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

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

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println("Calibrating sensors...");
  
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
  Serial.println("Calibration complete!");
  
  delay(1000);

  // I²C başlatma
  Wire.begin(8);  // Slave adresi 8
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  Serial.println("I2C initialized. Ready to start!");
}

// ---------------- Ana Loop ----------------
void loop() {
  // QTR sensörleri oku
  uint16_t position = qtr.readLineBlack(sensorValues);
  
  // Hata hesapla (0-7000 aralığında, 3500 merkez)
  int error = position - 3500;
  
  // PID hesaplama
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
  
  // Debug çıktısı (opsiyonel)
  if (newDataReceived) {
    Serial.print("New I2C data - Speed: ");
    Serial.print(i2cTargetSpeed);
    Serial.print(", Kp: ");
    Serial.print(i2cKp);
    Serial.print(", Ki: ");
    Serial.print(i2cKi);
    Serial.print(", Kd: ");
    Serial.println(i2cKd);
    newDataReceived = false;
  }
  
  delay(10); // Kısa bekleme
}

// ---------------- Motor Fonksiyonları ----------------
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
    analogWrite(LPWM_R, -rightSpeed);
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