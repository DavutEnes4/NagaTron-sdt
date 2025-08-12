#include <Wire.h>
#include <SoftwareWire.h>
#include <QTRSensors.h>
#include <MPU6050.h>
#include <ArduinoJson.h>

// I2C Adresleri
#define I2C_SLAVE_ADDRESS 0x42
#define MPU6050_ADDRESS 0x68
#define INT_PIN 2

// PID Ayarları
float Kp = 0.8, Ki = 0.0, Kd = 2.0;

int rightMaxSpeed = 220;
int leftMaxSpeed = 220;
int rightBaseSpeed = 75;
int leftBaseSpeed = 75;
int lastPosition = 0;
int lastPidOutput = 0;
int lastError = 0;

// Motor Pinleri
#define RPWM_R 10
#define LPWM_R 9
#define R_EN_R 8
#define L_EN_R 7
#define RPWM_L 5
#define LPWM_L 6
#define R_EN_L 3
#define L_EN_L 4

// Sensörler
QTRSensors qtr;
MPU6050 mpu;
SoftwareWire mpuWire(18, 19); // SDA, SCL

float yaw, pitch, roll;
int yawOffset = 0;
const uint8_t SensorCount = 8;
unsigned int sensorValues[SensorCount];

// Çalışma Modları
enum Mode { MODE_NONE, MODE_MANUAL, MODE_LINE, MODE_CALIBRATION };
Mode currentMode = MODE_LINE;
bool running = false;

#define I2C_BUFFER_SIZE 32
char i2cBuffer[I2C_BUFFER_SIZE];
volatile bool newMessage = false;

char cmd_status[32] = "none";
char lastRequestedData[8] = "ALL";
char logMessage[64] = "empty";

// ----------------- MPU ------------------
void calibrateMPU() {
  Serial.println("[MPU] Kalibrasyon yapılıyor...");
  int tempYaw, tempPitch, tempRoll;
  if (mpu.testConnection()) {
    mpu.getRotation(&tempYaw, &tempPitch, &tempRoll);
    yawOffset = tempYaw;
  }
}

int getCurrentAngle() {
  int y, p, r;
  if (!mpu.testConnection()) return 0;
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
  return abs(delta) < 5;
}

void turnAngle(int angle) {
  int startAngle = getCurrentAngle();
  int targetAngle = startAngle + angle;
  normalizeAngle(targetAngle);
  int dir = angle > 0 ? 1 : -1;

  while (!angleReached(targetAngle)) {
    if (dir > 0) turn_right();
    else turn_left();
    delay(10);
  }
  stopMotors();
}

// ----------------- Motor ------------------
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
  if (rightSpeed >= 0) {
    analogWrite(RPWM_R, rightSpeed);
    analogWrite(LPWM_R, 0);
  } else {
    analogWrite(RPWM_R, 0);
    analogWrite(LPWM_R, -rightSpeed);
  }

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
  setMotorSpeeds(-rightBaseSpeed, rightBaseSpeed);
}

void turn_right() {
  setMotorSpeeds(rightBaseSpeed, -leftBaseSpeed);
}

void setMotorDirections(int right, int left) {
  setMotorSpeeds(right * rightBaseSpeed, left * leftBaseSpeed);
}

// ----------------- QTR ------------------
void setupQTR() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);
}

void calibrateQTR() {
  Serial.println("[QTR] Kalibrasyon başlatıldı...");
  int temp = rightBaseSpeed;
  rightBaseSpeed = 65;
  leftBaseSpeed = 65;
  for (int i = 0; i < 1000; i++) {
    if (i % 36 > 18) turn_right();
    else turn_left();
    qtr.calibrate();
  }
  rightBaseSpeed = temp;
  leftBaseSpeed = temp;
  stopMotors();
  Serial.println("[QTR] Kalibrasyon tamamlandı.");
}

// ----------------- I2C ------------------
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
  i2cBuffer[i] = '\0';
  newMessage = true;

  Serial.print("[I2C RECEIVE] Master'dan veri alındı → '");
  Serial.print(i2cBuffer);
  Serial.print("'  (byte sayısı: ");
  Serial.print(howMany);
  Serial.println(")");
}


const char* getModeName() {
  switch (currentMode) {
    case MODE_MANUAL:      return "MANUAL";
    case MODE_LINE:        return "LINE";
    case MODE_CALIBRATION: return "CALIBRATION";
    default:               return "NONE";
  }
}

void requestEvent() {
  StaticJsonDocument<128> doc;
  if (strcmp(lastRequestedData, "POS") == 0) doc["pos"] = lastPosition;
  else if (strcmp(lastRequestedData, "ERR") == 0) doc["err"] = lastError;
  else if (strcmp(lastRequestedData, "OUT") == 0) doc["out"] = lastPidOutput;
  else if (strcmp(lastRequestedData, "LOG") == 0) doc["log"] = logMessage;
  else if (strcmp(lastRequestedData, "MODE") == 0) doc["mode"] = getModeName();
  else {
    doc["err"]  = lastError;
    doc["pos"]  = lastPosition;
    doc["out"]  = lastPidOutput;
    doc["mode"] = getModeName();
    doc["log"]  = logMessage;
  }

  Serial.print("[I2C REQUEST] Master veri istedi → Tip: ");
  Serial.print(lastRequestedData);
  Serial.print(" | Gönderilen JSON: ");
  serializeJson(doc, Serial);
  Serial.println();

  serializeJson(doc, Wire);  // Cevabı I²C üzerinden gönder
}

// ----------------- Komut İşleme ------------------
void handleI2CCommand(const char* cmd) {
  strcpy(cmd_status, "success");

  if (strncmp(cmd, "START", 5) == 0) {
    if (currentMode == MODE_LINE) {
      running = true;
      digitalWrite(LED_BUILTIN, HIGH);
      strcpy(logMessage , "START - Çizgi Takip Başlatıldı");
    } else {
      strcpy(logMessage , "START komutu çizgi modunda geçerli");
      strcpy(cmd_status, "invalid in MANUAL");
    }
  } else if (strncmp(cmd, "STOP", 4) == 0) {
    running = false;
    stopMotors();
    digitalWrite(LED_BUILTIN, LOW);
    strcpy(logMessage , "STOP - Motorlar durdu");

  } else if (strncmp(cmd, "RESTART", 7) == 0) {
    calibrateQTR();
    digitalWrite(LED_BUILTIN, HIGH);
    if (currentMode == MODE_LINE) {
      running = true;
      strcpy(logMessage , "RESTART - Kalibrasyon sonrası çizgi takip aktif");
    } else {
      running = false;
      strcpy(logMessage , "RESTART - Kalibrasyon yapıldı, MANUAL modda çizgi takip çalışmaz");
    }

  } else if (strncmp(cmd, "MODE:", 5) == 0) {
    if (strstr(cmd + 5, "MANUAL")) {
      currentMode = MODE_MANUAL;
      running = false;
      strcpy(logMessage , "Mod: MANUAL");
    } else if (strstr(cmd + 5, "LINE")) {
      currentMode = MODE_LINE;
      strcpy(logMessage , "Mod: LINE");
    } else if (strstr(cmd + 5, "CALIBRATION")) {
      currentMode = MODE_LINE;
      strcpy(logMessage , "Mod: CALIBRATION");
    } else {
      strcpy(cmd_status, "invalid MODE");
      strcpy(logMessage , "Bilinmeyen mod");
    }
  } else if (strncmp(cmd, "SPD:", 4) == 0) {
    int speed = atoi(cmd + 4);
    rightBaseSpeed = speed;
    leftBaseSpeed = speed;
    sprintf(logMessage , "Speed changed to %d", speed);

  } else if (strncmp(cmd, "PID:", 4) == 0) {
    float kp, ki, kd;
    if (sscanf(cmd + 4, "%f,%f,%f", &kp, &ki, &kd) == 3) {
      Kp = kp;
      Ki = ki;
      Kd = kd;
      sprintf(logMessage , "PID set to Kp=%.2f Ki=%.2f Kd=%.2f", Kp, Ki, Kd);
    } else {
      strcpy(cmd_status, "invalid PID");
      strcpy(logMessage , "Invalid PID format");
    }

  } else if (strncmp(cmd, "MOVE:", 5) == 0) {
    if (currentMode == MODE_MANUAL) {
      int right, left;
      if (sscanf(cmd + 5, "%d,%d", &right, &left) == 2) {
        setMotorDirections(right, left);
        sprintf(logMessage , "MANUAL MOVE: R=%d, L=%d", right, left);
      } else {
        strcpy(cmd_status, "invalid MOVE");
        strcpy(logMessage , "Invalid MOVE format");
      }
    } else {
      strcpy(cmd_status, "invalid in LINE");
      strcpy(logMessage , "MOVE komutu sadece MANUAL modda geçerli");
    }

  } else if (strncmp(cmd, "ANGLE:", 6) == 0) {
    if (currentMode == MODE_MANUAL) {
      int angle = atoi(cmd + 6);
      turnAngle(angle);
      sprintf(logMessage , "MANUAL TURN: %d derece", angle);
    } else {
      strcpy(cmd_status, "invalid in LINE");
      strcpy(logMessage , "ANGLE komutu sadece MANUAL modda geçerli");
    }

  } else if (strncmp(cmd, "GET:", 4) == 0) {
    strncpy(lastRequestedData, cmd + 4, 7);
    lastRequestedData[7] = '\0';
    strcpy(logMessage , "Data request updated");

  } else {
    strcpy(cmd_status, "unsuccessful");
    strcpy(logMessage , "Unknown command received");
  }

  notifyMaster();
  Serial.print("[I2C Komut] → ");
  Serial.println(cmd);
  Serial.print("[LOG] → ");
  Serial.println(logMessage);
}

// ----------------- Çizgi Takip ------------------
void lineFollow() {
  unsigned int position = qtr.readLineBlack(sensorValues);
  lastPosition = position;
  int error = position - 3500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed  = leftBaseSpeed - motorSpeed;
  lastPidOutput = motorSpeed;

  rightMotorSpeed = constrain(rightMotorSpeed, 0, rightMaxSpeed);
  leftMotorSpeed = constrain(leftMotorSpeed, 0, leftMaxSpeed);

  setMotorSpeeds(rightMotorSpeed, leftMotorSpeed);

  Serial.print("[Çizgi Takip] Pos: ");
  Serial.print(lastPosition);
  Serial.print("  Err: ");
  Serial.print(lastError);
  Serial.print("  Out: ");
  Serial.println(lastPidOutput);
}

// ----------------- Setup & Loop ------------------
void setup() {
  Serial.begin(115200);
  Serial.println("[SETUP] Başlatılıyor...");
  Serial.print("[Mod] Başlangıç modu: ");
  Serial.println(getModeName());

  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  mpuWire.begin();
  mpu.initialize();

  setupQTR();
  setupMotors();
  enableMotors();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(INT_PIN, OUTPUT);
  digitalWrite(INT_PIN, HIGH);

}

void loop() {
  if (newMessage) {
    newMessage = false;
    handleI2CCommand(i2cBuffer);
  }

  if (running) {
    if(currentMode == MODE_LINE){
      lineFollow();
    } else if(currentMode = MODE_CALIBRATION){
        calibrateMPU();
        calibrateQTR();
        running = false;
      } else {
        stopMotors();
      }
    }
  else {
    stopMotors();
  }
}
