#include <Wire.h>
#include <SoftwareWire.h>
#include <QTRSensors.h>
#include <MPU6050.h>
#include <ArduinoJson.h>
#include <math.h>

// ================== Sabitler & Pinler ==================
#define I2C_SLAVE_ADDRESS 0x42
#define MPU6050_ADDRESS   0x68
#define INT_PIN           2

// --- QTR yapılandırma ---
#define QTR_EMITTER_PIN  -1   // LEDON dijital pine bağlıysa pin no, Vcc'ye sabitse -1
#define QTR_SAMPLES      4
#define AUTO_CAL_MS      3000 // <<< 3 saniye kalibrasyon

// PID (çizgi takip)
float Kp = 0.346, Ki = 0.0, Kd = 2.2;

int rightMaxSpeed  = 85;
int leftMaxSpeed   = 85;
int rightBaseSpeed = 50;
int leftBaseSpeed  = 50;
int lastPosition   = 0;
int lastPidOutput  = 0;
int lastError      = 0;

// Motor Pinleri
#define RPWM_R 10
#define LPWM_R 9
#define R_EN_R 8
#define L_EN_R 7
#define RPWM_L 5
#define LPWM_L 6
#define R_EN_L 3
#define L_EN_L 4

// MZ80 & Buzzer
#define MZ80_PIN         12
#define MZ80_ACTIVE_LOW  1
#define BUZZER_PIN       11

// ================== Nesneler ==================
QTRSensors qtr;
MPU6050 mpu;
SoftwareWire mpuWire(20, 21);

// ================== Global Değişkenler ==================
const float GYRO_SENS = 131.0f;
float yawDeg = 0.0f;
float gyroZBias = 0.0f;
unsigned long lastIMUus = 0;

// Modlar
enum Mode { MODE_NONE, MODE_MANUAL, MODE_LINE, MODE_CALIBRATION };
Mode currentMode = MODE_LINE;
bool running = false;

const uint8_t SensorCount = 8;
unsigned int sensorValues[SensorCount];

// ---------- I2C komut kuyruğu (ISR-safe) ----------
#define I2C_BUFFER_SIZE 32
#define CMD_Q_SIZE 4
volatile uint8_t qHead = 0, qTail = 0;
char cmdQ[CMD_Q_SIZE][I2C_BUFFER_SIZE];

char cmd_status[32] = "none";
char lastRequestedData[8] = "ALL";
char logMessage[64] = "empty";

// MANUAL sürüş
int manualRightDir = 0; // -1,0,1
int manualLeftDir  = 0; // -1,0,1

// -------- Teker yön çarpanları --------
int DIR_R = +1;
int DIR_L = +1;

// -------- Manuel dönüş kontrol (PD - şu an pasif) --------
volatile bool turningActive = false;
float turnTargetDeg = 0.0f;
int   minTurnSpeed  = 50;
int   maxTurnSpeed  = 110;
float angleKp       = 0.1f;
const float ANGLE_TOL = 3.2f;
int   settleNeeded   = 6;
int   settleCount    = 0;
unsigned long turnStartMs = 0;
unsigned long turnTimeoutMs = 0;
float angleKd = 0.6f;
float prevErr = 0.0f;
unsigned long lastErrMs = 0;
unsigned long brakeUntilMs = 0;
const int brakePulseMs = 80;
const int brakePulseSp = 120;

// -------- Çizgi takibi seçenekleri --------
bool qtrCalibrated = false;
int  LINE_DIR = +1;      // tersse -1
bool READ_BLACK = true;  // siyah çizgi true / beyaz çizgi false

// ================== İleri Bildirimler ==================
void handleI2CCommand(const char* cmd);
void stopMotors();
void setMotorSpeeds(int rightSpeed, int leftSpeed);
void turnRight90();
void turnLeft90();

// ================== Yardımcılar ==================
inline void buzzerOn()  { if (BUZZER_PIN >= 0) digitalWrite(BUZZER_PIN, HIGH); }
inline void buzzerOff() { if (BUZZER_PIN >= 0) digitalWrite(BUZZER_PIN, LOW);  }

inline bool isObstacle() {
  int v = digitalRead(MZ80_PIN);
#if MZ80_ACTIVE_LOW
  return (v == LOW);
#else
  return (v == HIGH);
#endif
}

// ---------- Kuyruk işlemleri ----------
void enqueueCmd_isrSafe(const char* src) {
  uint8_t next = (uint8_t)((qHead + 1) % CMD_Q_SIZE);
  // doluysa en eskiyi düşür (qTail'i ilerlet)
  if (next == qTail) {
    qTail = (uint8_t)((qTail + 1) % CMD_Q_SIZE);
  }
  // kopyala
  uint8_t i=0;
  for (; i<I2C_BUFFER_SIZE-1 && src[i]; ++i) cmdQ[qHead][i] = src[i];
  cmdQ[qHead][i] = '\0';
  qHead = next;
}

bool dequeueCmd(char* out) {
  noInterrupts();
  bool empty = (qTail == qHead);
  if (!empty) {
    // kopyala
    for (uint8_t i=0; i<I2C_BUFFER_SIZE; ++i) {
      out[i] = cmdQ[qTail][i];
      if (out[i] == '\0') break;
    }
    qTail = (uint8_t)((qTail + 1) % CMD_Q_SIZE);
  }
  interrupts();
  return !empty;
}

// ================== IMU (SoftwareWire) ==================
bool mpuWakeUp_SW() {
  mpuWire.beginTransmission(MPU6050_ADDRESS);
  mpuWire.write(0x6B);
  mpuWire.write(0x00);
  return (mpuWire.endTransmission() == 0);
}

bool mpuReadGyroZRaw_SW(int16_t &gzRaw) {
  mpuWire.beginTransmission(MPU6050_ADDRESS);
  mpuWire.write(0x47);
  uint8_t err = mpuWire.endTransmission(false);
  if (err != 0) {
    mpuWire.beginTransmission(MPU6050_ADDRESS);
    mpuWire.write(0x47);
    err = mpuWire.endTransmission();
    if (err != 0) return false;
  }
  int n = mpuWire.requestFrom(MPU6050_ADDRESS, 2);
  if (n != 2) return false;
  uint8_t hi = mpuWire.read();
  uint8_t lo = mpuWire.read();
  gzRaw = (int16_t)((hi << 8) | lo);
  return true;
}

void imuUpdate() {
  if (lastIMUus == 0) { lastIMUus = micros(); return; }
  int16_t gzRaw;
  if (!mpuReadGyroZRaw_SW(gzRaw)) return;

  unsigned long now = micros();
  float dt = (now - lastIMUus) / 1e6f;
  lastIMUus = now;

  float gz_dps = (float)gzRaw / GYRO_SENS;
  gz_dps -= gyroZBias;
  yawDeg += gz_dps * dt;

  while (yawDeg >  180.f) yawDeg -= 360.f;
  while (yawDeg < -180.f) yawDeg += 360.f;
}

void imuCalibrateBias(unsigned samples = 200) {
  delay(50);
  long sum = 0;
  for (unsigned i=0; i<samples; ++i) {
    int16_t gz;
    if (mpuReadGyroZRaw_SW(gz)) sum += gz;
    delay(2);
  }
  float gz_avg = (float)sum / (float)samples;
  gyroZBias = gz_avg / GYRO_SENS;
}

// ================== Motor Fonksiyonları ==================
void setupMotors() {
  pinMode(R_EN_R, OUTPUT); pinMode(L_EN_R, OUTPUT);
  pinMode(R_EN_L, OUTPUT); pinMode(L_EN_L, OUTPUT);
  pinMode(RPWM_R, OUTPUT); pinMode(LPWM_R, OUTPUT);
  pinMode(RPWM_L, OUTPUT); pinMode(LPWM_L, OUTPUT);
}

void enableMotors() {
  digitalWrite(R_EN_R, HIGH); digitalWrite(L_EN_R, HIGH);
  digitalWrite(R_EN_L, HIGH); digitalWrite(L_EN_L, HIGH);
}

void stopMotors() {
  analogWrite(RPWM_R, 0); analogWrite(LPWM_R, 0);
  analogWrite(RPWM_L, 0); analogWrite(LPWM_L, 0);
}

void setMotorSpeeds(int rightSpeed, int leftSpeed) {
  rightSpeed *= DIR_R;
  leftSpeed  *= DIR_L;

  if (rightSpeed >= 0) { analogWrite(RPWM_R, rightSpeed); analogWrite(LPWM_R, 0); }
  else                 { analogWrite(RPWM_R, 0);          analogWrite(LPWM_R, -rightSpeed); }

  if (leftSpeed  >= 0) { analogWrite(RPWM_L, leftSpeed);  analogWrite(LPWM_L, 0); }
  else                 { analogWrite(RPWM_L, 0);          analogWrite(LPWM_L, -leftSpeed); }
}

void moveForward()  { setMotorSpeeds(rightBaseSpeed,  leftBaseSpeed); }
void moveBackward() { setMotorSpeeds(-rightBaseSpeed, -leftBaseSpeed); }
void turn_left()    { setMotorSpeeds(-rightBaseSpeed,  rightBaseSpeed); }
void turn_right()   { setMotorSpeeds( rightBaseSpeed, -leftBaseSpeed ); }

void setMotorDirections(int right, int left) {
  setMotorSpeeds(right * rightBaseSpeed, left * leftBaseSpeed);
  if (right == -1 && left == -1) buzzerOn();
  else                           buzzerOff();
}

// ================== QTR ==================
void setupQTR() {
  qtr.setTypeAnalog();
  //qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);
  qtr.setSamplesPerSensor(QTR_SAMPLES);
  if (QTR_EMITTER_PIN >= 0) qtr.setEmitterPin(QTR_EMITTER_PIN);
}

void autoCalibrateQTRSimple(uint16_t ms = AUTO_CAL_MS) {
  int savedR = rightBaseSpeed, savedL = leftBaseSpeed;
  rightBaseSpeed = 60; leftBaseSpeed = 60;

  unsigned long t0 = millis();
  int dir = +1;
  while (millis() - t0 < ms) {
    if (dir > 0) turn_right(); else turn_left();
    qtr.calibrate();
    if (((millis() - t0) % 250) < 5) dir = -dir; // ~250ms’de bir yön değiş
    delay(2);
  }

  rightBaseSpeed = savedR; leftBaseSpeed = savedL;
  stopMotors();
  qtrCalibrated = true;
}

void calibrateQTR() {
  int savedR = rightBaseSpeed, savedL = leftBaseSpeed;
  rightBaseSpeed = 65; leftBaseSpeed = 65;

  for (int i = 0; i < 400 && running && currentMode == MODE_CALIBRATION; i++) {
    if (i % 18 > 6) turn_right(); else turn_left();
    qtr.calibrate();
    if (!running) break;
    delay(2);
  }

  rightBaseSpeed = savedR; leftBaseSpeed = savedL;
  stopMotors();
  qtrCalibrated = true;
}

void manuelCalibrateQTR(int wait = 15000) {
  int now = millis();
  int waitTime = 0;
  buzzerOn();
  while(waitTime < wait) {
    if (waitTime % 1000 == 0) {
      Serial.print("Calibrating QTR... ");
      Serial.println(waitTime / 1000);
    }
    qtr.calibrate();
    qtrSerial();
    waitTime = millis() - now;
  }
  qtrCalibrated = true;
  buzzerOff();
  delay(50);
  buzzerOn();
  delay(50);
  buzzerOff();
  strcpy(logMessage, "QTR Calibrated");
  notifyMaster();
}

// ================== I2C (Slave) ==================
void notifyMaster() {
  digitalWrite(INT_PIN, LOW); delay(1); digitalWrite(INT_PIN, HIGH);
}

// NOT: ISR içinde Serial KULLANMIYORUZ!
void receiveEvent(int howMany) {
  if (howMany <= 0) return;
  char tmp[I2C_BUFFER_SIZE];
  int i = 0;
  while (Wire.available() && i < I2C_BUFFER_SIZE - 1) {
    tmp[i++] = Wire.read();
  }
  tmp[i] = '\0';
  enqueueCmd_isrSafe(tmp);  // komutu kuyruğa al
}

// NOT: ISR içinde Serial KULLANMIYORUZ!
void requestEvent() {
  StaticJsonDocument<128> doc;
  if      (!strcmp(lastRequestedData, "POS"))  doc["pos"]  = lastPosition;
  else if (!strcmp(lastRequestedData, "ERR"))  doc["err"]  = lastError;
  else if (!strcmp(lastRequestedData, "OUT"))  doc["out"]  = lastPidOutput;
  else if (!strcmp(lastRequestedData, "LOG"))  doc["log"]  = logMessage;
  else if (!strcmp(lastRequestedData, "MODE")) doc["mode"] = (currentMode==MODE_MANUAL?"MANUAL":currentMode==MODE_LINE?"LINE":currentMode==MODE_CALIBRATION?"CALIBRATION":"NONE");
  else {
    doc["err"]  = lastError;
    doc["pos"]  = lastPosition;
    doc["out"]  = lastPidOutput;
    doc["mode"] = (currentMode==MODE_MANUAL?"MANUAL":currentMode==MODE_LINE?"LINE":currentMode==MODE_CALIBRATION?"CALIBRATION":"NONE");
    doc["log"]  = logMessage;
  }
  serializeJson(doc, Wire);
}

const char* getModeName() {
  switch (currentMode) {
    case MODE_MANUAL: return "MANUAL";
    case MODE_LINE:   return "LINE";
    case MODE_CALIBRATION: return "CALIBRATION";
    default: return "NONE";
  }
}

// ================== Komut İşleme ==================
void handleI2CCommand(const char* cmd) {
  strcpy(cmd_status, "success");

  if (!strncmp(cmd, "START", 5)) {
    running = true;
    digitalWrite(LED_BUILTIN, HIGH);

    currentMode = MODE_LINE;
    strcpy(logMessage, "START - LINE");

  } else if (!strncmp(cmd, "STOP", 4)) {
    running = false;
    turningActive = false;
    stopMotors();
    digitalWrite(LED_BUILTIN, LOW);
    strcpy(logMessage , "STOP");

  } else if (!strncmp(cmd, "RESTART", 7)) {
    running = true;
    digitalWrite(LED_BUILTIN, HIGH);
    currentMode = MODE_CALIBRATION;
    strcpy(logMessage , "RESTART - Cal");
    manuelCalibrateQTR();
    qtrCalibrated = true;
    currentMode = MODE_LINE;
    running = false;

  } else if (!strncmp(cmd, "MODE:", 5)) {
    if (strstr(cmd + 5, "MANUAL")) { currentMode = MODE_MANUAL; running = false; strcpy(logMessage , "Mod: MANUAL"); }
    else if (strstr(cmd + 5, "LINE")) { currentMode = MODE_LINE; strcpy(logMessage , "Mod: LINE"); }
    else if (strstr(cmd + 5, "CALIBRATION")) { currentMode = MODE_CALIBRATION; strcpy(logMessage , "Mod: CAL"); }
    else { strcpy(cmd_status, "invalid MODE"); strcpy(logMessage , "Bilinmeyen mod"); }

  } else if (!strncmp(cmd, "SPD:", 4)) {
    int speed = atoi(cmd + 4);
    rightBaseSpeed = constrain(speed, 0, 255);
    leftBaseSpeed  = constrain(speed, 0, 255);
    sprintf(logMessage , "Speed=%d", speed);

  } else if (!strncmp(cmd, "PID:", 4)) {
    float kp, ki, kd;
    if (sscanf(cmd + 4, "%f,%f,%f", &kp, &ki, &kd) == 3) {
      Kp = kp; Ki = ki; Kd = kd;
      sprintf(logMessage , "PID Kp=%.2f Ki=%.2f Kd=%.2f", Kp, Ki, Kd);
    } else { strcpy(cmd_status, "invalid PID"); strcpy(logMessage , "PID format"); }

  } else if (!strncmp(cmd, "MOVE:", 5)) {
    if (currentMode == MODE_MANUAL) {
      int right, left;
      if (sscanf(cmd + 5, "%d,%d", &left, &right) == 2) {
        manualRightDir = constrain(right, -1, 1);
        manualLeftDir  = constrain(left , -1, 1);
        turningActive = false;
        running = true;
        setMotorDirections(manualRightDir, manualLeftDir);
        sprintf(logMessage , "MANUAL MOVE: R=%d L=%d", manualRightDir, manualLeftDir);
      } else { strcpy(cmd_status, "invalid MOVE"); strcpy(logMessage , "MOVE format"); }
    } else { strcpy(cmd_status, "invalid in LINE"); strcpy(logMessage , "MANUAL disi MOVE yok"); }

  } else if (!strncmp(cmd, "ANGLE:", 6)) {
    // PD turn şu an pasif tutuluyor
    strcpy(logMessage , "TURN fx pasif");

  } else if (!strncmp(cmd, "GET:", 4)) {
    strncpy(lastRequestedData, cmd + 4, 7);
    lastRequestedData[7] = '\0';
    strcpy(logMessage , "GET updated");

  } else if (!strncmp(cmd, "TURNLEFT", 8)) {
    turnLeft90(); strcpy(logMessage , "TURN LEFT 90");

  } else if (!strncmp(cmd, "TURNRIGHT", 9)) {
    turnRight90(); strcpy(logMessage , "TURN RIGHT 90");

  } else if (!strncmp(cmd, "DIR:", 4)) {
    int dr, dl;
    if (sscanf(cmd + 4, "%d,%d", &dr, &dl) == 2) {
      DIR_R = (dr >= 0) ? +1 : -1;
      DIR_L = (dl >= 0) ? +1 : -1;
      sprintf(logMessage, "DIR set R=%d L=%d", DIR_R, DIR_L);
    } else { strcpy(cmd_status, "invalid DIR"); strcpy(logMessage, "DIR:1,-1"); }

  } else if (!strncmp(cmd, "LINEDIR:", 8)) {
    int d = atoi(cmd + 8);
    LINE_DIR = (d >= 0) ? +1 : -1;
    sprintf(logMessage, "LINE_DIR=%d", LINE_DIR);

  } else if (!strncmp(cmd, "READMODE:", 9)) {
    if (strstr(cmd + 9, "BLACK")) { READ_BLACK = true;  strcpy(logMessage, "READMODE=BLACK"); }
    else if (strstr(cmd + 9, "WHITE")) { READ_BLACK = false; strcpy(logMessage, "READMODE=WHITE"); }
    else { strcpy(cmd_status, "invalid READMODE"); strcpy(logMessage, "BLACK/WHITE"); }

  } else if (!strncmp(cmd, "QTRCAL", 6)) {
    bool wasRunning = running;
    running = true;
    autoCalibrateQTRSimple(AUTO_CAL_MS); // 3 sn
    running = wasRunning;
    strcpy(logMessage, "QTR AutoCal done");

  } else {
    strcpy(cmd_status, "unsuccessful");
    strcpy(logMessage , "Unknown command");
  }

  notifyMaster();
}

// ================== Çizgi Takip ==================

void lineFollow() {
  //if (!qtrCalibrated) autoCalibrateQTRSimple(AUTO_CAL_MS);

  if (isObstacle()) { stopMotors(); buzzerOn(); return; }

  unsigned int position = READ_BLACK ? qtr.readLineBlack(sensorValues)
                                     : qtr.readLineWhite(sensorValues);
  lastPosition = position;

  int error = (int)position - 3500;   // merkez 3500
  int steer = (int)(Kp * error/2 + Kd * (error - lastError));
  lastError = error;

  steer *= LINE_DIR/15;
  
  int rightMotorSpeed = constrain(rightBaseSpeed + steer, 0, rightMaxSpeed);
  int leftMotorSpeed  = constrain(leftBaseSpeed  - steer, 0, leftMaxSpeed);
  Serial.print(steer);
  Serial.print("\t");
  lastPidOutput = steer;
  setMotorSpeeds(rightMotorSpeed,leftMotorSpeed);
}

/*
void lineFollow() {
  unsigned int position = qtr.readLineBlack(sensorValues);
  lastPosition = position;
  int error = position - 3500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed  = leftBaseSpeed  - motorSpeed;
  lastPidOutput = motorSpeed;

  rightMotorSpeed = constrain(rightMotorSpeed, 0, rightMaxSpeed);
  leftMotorSpeed  = constrain(leftMotorSpeed , 0, leftMaxSpeed);

  setMotorSpeeds(rightMotorSpeed, leftMotorSpeed);
}
*/
void qtrSerial(){
  // print the sensor values as numbers from 0 to 2500, where 0 means maximum
  // reflectance and 2500 means minimum reflectance
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.print(qtr.readLineBlack(sensorValues));
    Serial.println();
}

// ================== Setup & Loop ==================
void setup() {
  Serial.begin(115200);
  Serial.println("[SETUP] Basliyor...");
  Serial.print("[Mod] "); Serial.println(getModeName());

  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  mpuWire.begin();
  mpuWakeUp_SW();
  delay(50);
  imuCalibrateBias(200);
  lastIMUus = micros();

  setupQTR();
  setupMotors();
  enableMotors();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(INT_PIN, OUTPUT); digitalWrite(INT_PIN, HIGH);

  pinMode(MZ80_PIN, INPUT_PULLUP);
  if (BUZZER_PIN >= 0) { pinMode(BUZZER_PIN, OUTPUT); buzzerOff(); }

  // Açılışta otomatik QTR kalibrasyonu (3 sn)
  //autoCalibrateQTRSimple(AUTO_CAL_MS);
}

void loop() {
  // Kuyruktaki TÜM komutları işle
  char cmdBuf[I2C_BUFFER_SIZE];
  while (dequeueCmd(cmdBuf)) {
    handleI2CCommand(cmdBuf);
  }

  imuUpdate();

  if (isObstacle()) { stopMotors(); buzzerOn(); delay(5); return; }

  if (running) {
    if (currentMode == MODE_LINE) {
      buzzerOff();
      lineFollow();
      qtrSerial();
    } else if (currentMode == MODE_MANUAL) {
      setMotorDirections(manualRightDir, manualLeftDir);
      delay(5);
    } else {
      stopMotors(); buzzerOff();
    }
  } else {
    stopMotors(); buzzerOff(); delay(5);
  }
}

// ================== 90 DERECE SAĞA/SOLA DÖNME ==================
void turnRight90() {
  setMotorSpeeds(100, -100);
  delay(500); // sahada ayarla
  stopMotors();
}
void turnLeft90() {
  setMotorSpeeds(-100, 100);
  delay(500); // sahada ayarla
  stopMotors();
}
