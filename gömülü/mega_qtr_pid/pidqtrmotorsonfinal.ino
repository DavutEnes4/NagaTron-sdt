#include <QTRSensors.h>

// ---- PID Ayarları ----
#define Kp 0.4
#define Kd 2
#define rightMaxSpeed 120   // Maksimum hız daha da düşürüldü
#define leftMaxSpeed 120
int rightBaseSpeed = 50;  // Temel hız düşürüldü
int leftBaseSpeed = 50;

// ---- Motor Pinleri (BTS7960B) ----
// Sağ Motor
#define RPWM_R 9
#define LPWM_R 10
#define R_EN_R 7
#define L_EN_R 8 

// Sol Motor
#define RPWM_L 6
#define LPWM_L 5
#define R_EN_L 4
#define L_EN_L 3

// ---- QTR Sensör Ayarları ----
QTRSensors qtr;
const uint8_t SensorCount = 8;
unsigned int sensorValues[SensorCount];

int lastError = 0;

void setup()
{
  // QTR sensör
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);

  // Motor pinleri
  pinMode(RPWM_R, OUTPUT);
  pinMode(LPWM_R, OUTPUT);
  pinMode(R_EN_R, OUTPUT);
  pinMode(L_EN_R, OUTPUT);

  pinMode(RPWM_L, OUTPUT);
  pinMode(LPWM_L, OUTPUT);
  pinMode(R_EN_L, OUTPUT);
  pinMode(L_EN_L, OUTPUT);

  enableMotors();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 

  // SADECE SAĞA-SOLA DÖNEREK KALİBRASYON
  int tempRight = rightBaseSpeed;
  rightBaseSpeed = 50;
  int tempLeft = leftBaseSpeed;
  leftBaseSpeed = 50;
  for (int i = 0; i < 100; i++) {
    if (i % 2 == 0) {
      turn_right();
    } else {
      turn_left();
    }
    qtr.calibrate();
    delay(250);
  }
  rightBaseSpeed = tempRight;
  leftBaseSpeed = tempLeft;
  stopMotors();
  digitalWrite(LED_BUILTIN, LOW); 
  delay(1000);
}

void loop()
{
  unsigned int position = qtr.readLineBlack(sensorValues);
  int error = position - 3500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed  = leftBaseSpeed - motorSpeed;

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
  analogWrite(RPWM_R, rightSpeed);
  analogWrite(LPWM_R, 0);
  analogWrite(RPWM_L, leftSpeed);
  analogWrite(LPWM_L, 0);
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
