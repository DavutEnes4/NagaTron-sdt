#include <Wire.h>
#include <SoftwareWire.h>
#include <QTRSensors.h>
#include <MPU6050.h>
#include <ArduinoJson.h>

// I²C Adresleri
#define I2C_SLAVE_ADDRESS 0x42
#define MPU6050_ADDRESS 0x68

#define INT_PIN 2

// ---- PID Ayarları ----
float Kp = 0.4;
int Kd = 2;
int rightMaxSpeed 120;   // Maksimum hız daha da düşürüldü
int leftMaxSpeed 120;
int rightBaseSpeed = 50;  // Temel hız düşürüldü
int leftBaseSpeed = 50;
int lastPosition = 0;
int lastPidOutput = 0;

// Kontrol Modları (Ortak konfigürasyondan alınır)
enum ControlMode {
  MODE_AUTONOMOUS = MODE_AUTONOMOUS,  // Otonom çizgi takibi
  MODE_MANUAL = MODE_MANUAL,          // Manuel RadioLink kontrolü
  MODE_MQTT = MODE_MQTT,              // MQTT komut kontrolü
  MODE_CALIBRATION = MODE_CALIBRATION // Kalibrasyon modu
};

// ---- Motor Pinleri (BTS7960B) ----
// Sağ Motor
#define RPWM_R 9
#define LPWM_R 10
#define R_EN_R 7
#define L_EN_R 8 

// Sol Motor
#define RPWM_L 5
#define LPWM_L 6
#define R_EN_L 3
#define L_EN_L 4

// ---- Sensör Nesneleri ----
QTRSensors qtr;
MPU6050 mpu;

// ---- MPU6050 ----
SoftwareWire mpuWire(4, 5); // SDA, SCL pinleri
float yaw, pitch, roll;
int yawOffset = 0;

void calibrateMPU() {
  int tempYaw, tempPitch, tempRoll;
  mpu.getRotation(&tempYaw, &tempPitch, &tempRoll);
  yawOffset = tempYaw;
}

int getCurrentAngle() {
  int y, p, r;
  mpu.getRotation(&y, &p, &r);
  return y - yawOffset;
}

void normalizeAngle(int &angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
}

bool angleReached(int targetAngle) {
  int currentAngle = getCurrentAngle();
  int delta = targetAngle - currentAngle;
  normalizeAngle(delta);
  return abs(delta) < 5;  // 5 derece tolerans
}


// ---- QTR Sensör Ayarları ----
const uint8_t SensorCount = 8;
unsigned int sensorValues[SensorCount];

bool running = false;

int lastError = 0;

#define I2C_BUFFER_SIZE 32
char i2cBuffer[I2C_BUFFER_SIZE];
volatile bool newMessage = false;

char cmd_status[32] = "none";        // Komut işlenme durumu
char lastRequestedData[8] = "ALL";   // Master hangi veriyi istiyor
char log[64] = "empty";              // En son olay logu

void notifyMaster() {
  digitalWrite(INT_PIN, LOW);
  delay(1);
  digitalWrite(INT_PIN, HIGH);
}

void receiveEvent(int howMany) {
  if (howMany >= I2C_BUFFER_SIZE) howMany = I2C_BUFFER_SIZE - 1;

  int i = 0;
  while (Wire.available() && i < I2C_BUFFER_SIZE - 1) {
    i2cBuffer[i++] = Wire.read();
  }
  i2cBuffer[i] = '\0';  // Null-terminator (C-string sonu)

  newMessage = true;
}

void handleI2CCommand(const char* cmd){
  strcpy(cmd_status, "success");  // Varsayılan
  if (strcmp(cmd, "START",5) == 0) {
    running = true;
    digitalWrite(LED_BUILTIN, HIGH);
    strcpy(log, "START command received");
  } else if (strcmp(cmd, "STOP",4) == 0) {
    running = false;
    stopMotors();
    digitalWrite(LED_BUILTIN, LOW);
    strcpy(log, "STOP command received");
  } else if (strcmp(cmd, "RESTART",7) == 0) {
    calibrateQTR();
    digitalWrite(LED_BUILTIN, HIGH);
    running = true;
  } else if (strncmp(cmd, "SPD:", 4) == 0) {
    int speed = atoi(cmd + 4);
    rightBaseSpeed = speed;
    leftBaseSpeed = speed;
    sprintf(log, "Speed changed to %d", speed);

  } else if (strncmp(cmd, "KP:", 3) == 0) {
    Kp = atof(cmd + 3);
    sprintf(log, "Kp changed to %.2f", Kp);

  } else if (strncmp(cmd, "KD:", 3) == 0) {
    Kd = atof(cmd + 3);
    sprintf(log, "Kd changed to %.2f", Kd);
  } else if (strncmp(cmd, "FORWARD", 7) == 0) {
    moveForward();
    strcpy(log, "Moving forward");
  } else if (strncmp(cmd, "BACK", 4) == 0) {
    moveBackward();
    strcpy(log, "Moving backward");
  } else if (strncmp(cmd, "LEFT", 4) == 0) {
    turn_left();
    strcpy(log, "Turning left");
  } else if (strncmp(cmd, "RIGHT", 5) == 0) {
    turn_right();
    strcpy(log, "Turning right");
  } else if (strncmp(cmd, "TURNL90", 7) == 0) {
    turnLeft90();
    strcpy(log, "Turned left 90");
  } else if (strncmp(cmd, "TURNR90", 7) == 0) {
    turnRight90();
    strcpy(log, "Turned right 90");
  } else if (strncmp(cmd, "TURNL180", 8) == 0) {
    turnLeft180();
    strcpy(log, "Turned left 180");
  } else if (strncmp(cmd, "TURNR180", 8) == 0) {
    turnRight180();
    strcpy(log, "Turned right 180");
  }else {
    strcpy(cmd_status, "unsuccessful");
    strcpy(log, "Unknown command received");
  }
  notifyMaster();
}

void requestEvent() {
  StaticJsonDocument<64> doc;

  if (strcmp(lastRequestedData, "POS") == 0) {
    doc["pos"] = lastPosition;
  } else if (strcmp(lastRequestedData, "ERR") == 0) {
    doc["err"] = lastError;
  } else if (strcmp(lastRequestedData, "OUT") == 0) {
    doc["out"] = lastPidOutput;
  } else if (strcmp(lastRequestedData,"LOG") == 0) {
    doc["log"] = log;
  } else {
    doc["err"] = lastError;
    doc["pos"] = lastPosition;
    doc["out"] = lastPidOutput;
  }
  char buffer[64];
  serializeJson(doc, buffer);
  Wire.write(buffer);  // Master’a gönder
}

void setupQTR(){
  // QTR sensör
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);
}

void setupMotors(){
  pinMode(RPWM_R, OUTPUT);
  pinMode(LPWM_R, OUTPUT);
  pinMode(R_EN_R, OUTPUT);
  pinMode(L_EN_R, OUTPUT);

  pinMode(RPWM_L, OUTPUT);
  pinMode(LPWM_L, OUTPUT);
  pinMode(R_EN_L, OUTPUT);
  pinMode(L_EN_L, OUTPUT);

  enableMotors();
}

void calibrateQTR(){
  for (int i = 0; i < 100; i++) {
    if (i % 2 == 0) {
      turn_right();
    } else {
      turn_left();
    }
    qtr.calibrate();
    delay(250);
  }
  stopMotors();
}

void setup(){
  Wire.begin(I2C_SLAVE_ADDRESS);  // I2C slave adresin 0x42
  Wire.onReceive(receiveEvent);   // Master'dan veri geldiğinde çağrılır
  Wire.onRequest(requestEvent);   // Master veri istediğinde çağrılır

  mpu.setWire(&mpuWire);  
  mpuWire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 bağlantısı başarısız!");
    while (1);
  } else {
    Serial.println("MPU6050 bağlandı.");
  }

  setupQTR();
  setupMotors();
  

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 
  
  calibrateQTR();
  
  digitalWrite(LED_BUILTIN, LOW); 
  delay(1000);
}

void loop(){
   if (newMessage) {
    newMessage = false;
    handleI2CCommand(i2cBuffer);
  }

  if (running) {
    lineFollow();
  } else {
    stopMotors();  // Gereksiz tekrarlar için değil, güvenlik için
    delay(100);    // CPU boşa çalışmasın
  }
}

// ---- Çizgi Takip Fonksiyonu ----
void lineFollow(){
  unsigned int position = qtr.readLineBlack(sensorValues);
  lastPosition = position;
  int error = position - 3500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed  = leftBaseSpeed - motorSpeed;
  lastPidOutput = motorSpeed;

  if (rightMotorSpeed > rightMaxSpeed) rightMotorSpeed = rightMaxSpeed;
  if (leftMotorSpeed > leftMaxSpeed) leftMotorSpeed = leftMaxSpeed;
  if (rightMotorSpeed < 0) rightMotorSpeed = 0;
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;

  setMotorSpeeds(rightMotorSpeed, leftMotorSpeed);
}

// ---- Motor Fonksiyonları ----
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
  // Sağ motor
  if (rightSpeed >= 0) {
    analogWrite(RPWM_R, rightSpeed);
    analogWrite(LPWM_R, 0);
  } else {
    analogWrite(RPWM_R, 0);
    analogWrite(LPWM_R, -rightSpeed);
  }

  // Sol motor
  if (leftSpeed >= 0) {
    analogWrite(RPWM_L, leftSpeed);
    analogWrite(LPWM_L, 0);
  } else {
    analogWrite(RPWM_L, 0);
    analogWrite(LPWM_L, -leftSpeed);
  }
}

void moveForward() {
  setMotorSpeeds(rightBaseSpeed, leftBaseSpeed);
}

void moveBackward() {
  setMotorSpeeds(-rightBaseSpeed, -leftBaseSpeed);
}

void turn_left() {
  analogWrite(RPWM_R, rightBaseSpeed);
  analogWrite(LPWM_R, 0);
  analogWrite(RPWM_L, 0);
  analogWrite(LPWM_L, leftBaseSpeed);
}

void turn_right() {
  analogWrite(RPWM_R, 0);
  analogWrite(LPWM_R, rightBaseSpeed);
  analogWrite(RPWM_L, leftBaseSpeed);
  analogWrite(LPWM_L, 0);
}

void turnRight90() {
  int startAngle = getCurrentAngle();
  int targetAngle = startAngle + 90;
  normalizeAngle(targetAngle);

  while (!angleReached(targetAngle)) {
    turnRight();  // Motorları sağa döndür
    delay(10);
  }
  stopMotors();
}

void turnLeft90() {
  int startAngle = getCurrentAngle();
  int targetAngle = startAngle - 90;
  normalizeAngle(targetAngle);

  while (!angleReached(targetAngle)) {
    turnLeft();  // Motorları sola döndür
    delay(10);
  }
  stopMotors();
}

void turnRight180() {
  int startAngle = getCurrentAngle();
  int targetAngle = startAngle + 180;
  normalizeAngle(targetAngle);

  while (!angleReached(targetAngle)) {
    turnRight();
    delay(10);
  }
  stopMotors();
}

void turnLeft180() {
  int startAngle = getCurrentAngle();
  int targetAngle = startAngle - 180;
  normalizeAngle(targetAngle);

  while (!angleReached(targetAngle)) {
    turnLeft();
    delay(10);
  }
  stopMotors();
}
