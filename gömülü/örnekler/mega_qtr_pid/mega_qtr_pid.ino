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
#define RPWM_R 10
#define LPWM_R 9
#define R_EN_R 7
#define L_EN_R 8 

// Sol Motor
#define RPWM_L 5
#define LPWM_L 6
#define R_EN_L 3
#define L_EN_L 4

// ---- QTR Sensör Ayarları ----
QTRSensors qtr;
const uint8_t SensorCount = 8;
unsigned int sensorValues[SensorCount];

int lastError = 0;

void setup()
{
  Serial.begin(115200);
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
  for (int i = 0; i < 1000; i++) {
    if (i % 10 < 5) {
      Serial.println("Sag");
      turn_right();
    } else {
      Serial.println("Sol");
      turn_left();
    }
    qtr.calibrate();
    delay(25);
  }
  rightBaseSpeed = tempRight;
  leftBaseSpeed = tempLeft;
  stopMotors();
  digitalWrite(LED_BUILTIN, LOW); 
  delay(1000);
}

void loop()
{
  uint16_t position = qtr.readLineBlack(sensorValues);
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

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
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
