/*
 * 🤖 Arduino Mega Robotik Kontrol Sistemi
 * Arduino Mega 2560 + QTR-8A + GY-89 + BTS7960B Motorlar + RadioLink R12DS
 * 
 * Fonksiyonlar:
 * - Çizgi takibi ve PID kontrol
 * - 9 eksenli IMU verisi (ivme, jiroskop, yön, pusula)
 * - Basınç ve yükseklik ölçümü
 * - I²C üzerinden veri aktarımı
 * - EEPROM tabanlı konfigürasyon
 * - RadioLink R12DS v1.1 uzaktan kumanda desteği
 * - QTR kalibrasyonu sırasında araç hareketi
 * - Ortak konfigürasyon sistemi
 */

#include <Wire.h>
#include <QTRSensors.h>
#include <PID_v1.h>
#include <MPU6050.h>
#include <HMC5883L.h>
#include <BMP180.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <Servo.h>
#include "../04_Dokumantasyon/Shared_Config.h"

// Pin Tanımlamaları (Ortak konfigürasyondan alınır)
#define QTR_EMITTER_PIN QTR_EMITTER_PIN
#define MOTOR_LEFT_PWM MOTOR_LEFT_RPWM_PIN
#define MOTOR_LEFT_DIR MOTOR_LEFT_LPWM_PIN
#define MOTOR_LEFT_BRAKE MOTOR_LEFT_R_EN_PIN
#define MOTOR_RIGHT_PWM MOTOR_RIGHT_RPWM_PIN
#define MOTOR_RIGHT_DIR MOTOR_RIGHT_LPWM_PIN
#define MOTOR_RIGHT_BRAKE MOTOR_RIGHT_R_EN_PIN

// RadioLink R12DS v1.1 Pin Tanımlamaları (Ortak konfigürasyondan alınır)
#define RADIOLINK_CH1_PIN RADIOLINK_CH1_PIN
#define RADIOLINK_CH2_PIN RADIOLINK_CH2_PIN
#define RADIOLINK_CH3_PIN RADIOLINK_CH3_PIN
#define RADIOLINK_CH4_PIN RADIOLINK_CH4_PIN
#define RADIOLINK_CH5_PIN RADIOLINK_CH5_PIN
#define RADIOLINK_CH6_PIN RADIOLINK_CH6_PIN
#define RADIOLINK_CH7_PIN RADIOLINK_CH7_PIN
#define RADIOLINK_CH8_PIN RADIOLINK_CH8_PIN

// I²C Adresleri
#define I2C_SLAVE_ADDRESS I2C_SLAVE_ADDRESS
#define MPU6050_ADDRESS 0x68
#define HMC5883L_ADDRESS 0x1E
#define BMP180_ADDRESS 0x77

// Kontrol Modları (Ortak konfigürasyondan alınır)
enum ControlMode {
  MODE_AUTONOMOUS = MODE_AUTONOMOUS,  // Otonom çizgi takibi
  MODE_MANUAL = MODE_MANUAL,          // Manuel RadioLink kontrolü
  MODE_MQTT = MODE_MQTT,              // MQTT komut kontrolü
  MODE_CALIBRATION = MODE_CALIBRATION // Kalibrasyon modu
};

// Sensör Nesneleri
QTRSensors qtr;
MPU6050 mpu;
HMC5883L compass;
BMP180 bmp;

// PID Kontrolcüleri (Ortak konfigürasyondan alınır)
double setpoint = PID_SETPOINT_DEFAULT; // Çizgi pozisyonu
double input, output;
double kp = PID_KP_DEFAULT, ki = PID_KI_DEFAULT, kd = PID_KD_DEFAULT;
PID linePID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// Konfigürasyon Yapısı
struct Config {
  uint8_t qtr_pins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
  uint8_t qtr_emitter_pin = QTR_EMITTER_PIN;
  uint8_t motor_left_pwm = MOTOR_LEFT_PWM;
  uint8_t motor_left_dir = MOTOR_LEFT_DIR;
  uint8_t motor_left_brake = MOTOR_LEFT_BRAKE;
  uint8_t motor_right_pwm = MOTOR_RIGHT_PWM;
  uint8_t motor_right_dir = MOTOR_RIGHT_DIR;
  uint8_t motor_right_brake = MOTOR_RIGHT_BRAKE;
  uint8_t i2c_sda = 20;
  uint8_t i2c_scl = 21;
  
  // RadioLink Pin Konfigürasyonu
  uint8_t radiolink_ch1_pin = RADIOLINK_CH1_PIN;
  uint8_t radiolink_ch2_pin = RADIOLINK_CH2_PIN;
  uint8_t radiolink_ch3_pin = RADIOLINK_CH3_PIN;
  uint8_t radiolink_ch4_pin = RADIOLINK_CH4_PIN;
  uint8_t radiolink_ch5_pin = RADIOLINK_CH5_PIN;
  uint8_t radiolink_ch6_pin = RADIOLINK_CH6_PIN;
  uint8_t radiolink_ch7_pin = RADIOLINK_CH7_PIN;
  uint8_t radiolink_ch8_pin = RADIOLINK_CH8_PIN;
  
  // PID Parametreleri
  double pid_kp = 0.5;
  double pid_ki = 0.0;
  double pid_kd = 0.1;
  double pid_setpoint = 3500;
  double pid_output_limit = 255;
  
  // Sensör Kalibrasyonu
  uint16_t qtr_min_values[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint16_t qtr_max_values[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
  float accel_offset[3] = {0.0, 0.0, 0.0};
  float gyro_offset[3] = {0.0, 0.0, 0.0};
  
  // RadioLink Kalibrasyon Değerleri
  uint16_t radiolink_min_values[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
  uint16_t radiolink_max_values[8] = {2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000};
  uint16_t radiolink_center_values[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
} config;

// Değişkenler
unsigned long lastSensorRead = 0;
const unsigned long sensorInterval = 100; // 100ms
unsigned long lastI2CTransmission = 0;
const unsigned long i2cInterval = 50; // 50ms
unsigned long lastRadioLinkRead = 0;
const unsigned long radiolinkInterval = 20; // 20ms (50Hz)

// Kontrol Modu
ControlMode currentMode = MODE_AUTONOMOUS;

// Motor Durumu
struct MotorStatus {
  uint8_t left_speed = 0;
  uint8_t right_speed = 0;
  uint8_t left_direction = 0; // 0: forward, 1: backward
  uint8_t right_direction = 0;
  float left_current = 0.0;
  float right_current = 0.0;
  float left_temperature = 25.0;
  float right_temperature = 25.0;
} motorStatus;

// RadioLink Durumu
struct RadioLinkStatus {
  uint16_t throttle = 1500;    // CH1
  uint16_t aileron = 1500;     // CH2
  uint16_t elevator = 1500;    // CH3
  uint16_t rudder = 1500;      // CH4
  uint16_t gear = 1500;        // CH5
  uint16_t aux1 = 1500;        // CH6
  uint16_t aux2 = 1500;        // CH7
  uint16_t aux3 = 1500;        // CH8
  bool connected = false;
  unsigned long lastSignal = 0;
} radioStatus;

// QTR Kalibrasyon Durumu
struct CalibrationStatus {
  bool is_calibrating = false;
  unsigned long calibration_start = 0;
  unsigned long movement_start = 0;
  int movement_direction = 1; // 1: sağa, -1: sola
  int movement_speed = 100;
  unsigned long movement_duration = 2000; // 2 saniye
} calibStatus;

void setup() {
  Serial.begin(115200);
  Serial.println("🤖 Arduino Mega Robotik Kontrol Sistemi Başlatılıyor...");
  
  // EEPROM'dan konfigürasyonu yükle
  loadConfigFromEEPROM();
  
  // Pin modlarını ayarla
  setupPins();
  
  // Sensörleri başlat
  setupSensors();
  
  // PID kontrolcüsünü başlat
  setupPID();
  
  // I²C Master başlat
  setupI2C();
  
  // RadioLink alıcısını başlat
  setupRadioLink();
  
  // QTR sensörünü kalibre et
  calibrateQTR();
  
  Serial.println("✅ Arduino Mega Sistemi Hazır!");
}

void loop() {
  // RadioLink sinyallerini oku
  if (millis() - lastRadioLinkRead >= radiolinkInterval) {
    readRadioLink();
    lastRadioLinkRead = millis();
  }
  
  // Kontrol modunu belirle
  determineControlMode();
  
  // Sensör verilerini oku
  if (millis() - lastSensorRead >= sensorInterval) {
    readSensors();
    lastSensorRead = millis();
  }
  
  // I²C üzerinden veri gönder
  if (millis() - lastI2CTransmission >= i2cInterval) {
    sendDataToESP32();
    lastI2CTransmission = millis();
  }
  
  // I²C'den gelen komutları kontrol et
  checkI2CCommands();
  
  // Kontrol moduna göre hareket et
  switch (currentMode) {
    case MODE_AUTONOMOUS:
      lineFollow();
      break;
    case MODE_MANUAL:
      manualControl();
      break;
    case MODE_MQTT:
      // MQTT komutları zaten I²C üzerinden geliyor
      break;
    case MODE_CALIBRATION:
      calibrationMovement();
      break;
  }
  
  delay(10);
}

void setupPins() {
  // QTR sensör pinleri
  qtr.setTypeAnalog();
  qtr.setSensorPins(config.qtr_pins, 8);
  qtr.setEmitterPin(config.qtr_emitter_pin);
  
  // Motor pinleri
  pinMode(config.motor_left_pwm, OUTPUT);
  pinMode(config.motor_left_dir, OUTPUT);
  pinMode(config.motor_left_brake, OUTPUT);
  pinMode(config.motor_right_pwm, OUTPUT);
  pinMode(config.motor_right_dir, OUTPUT);
  pinMode(config.motor_right_brake, OUTPUT);
  
  // RadioLink pinleri
  pinMode(config.radiolink_ch1_pin, INPUT);
  pinMode(config.radiolink_ch2_pin, INPUT);
  pinMode(config.radiolink_ch3_pin, INPUT);
  pinMode(config.radiolink_ch4_pin, INPUT);
  pinMode(config.radiolink_ch5_pin, INPUT);
  pinMode(config.radiolink_ch6_pin, INPUT);
  pinMode(config.radiolink_ch7_pin, INPUT);
  pinMode(config.radiolink_ch8_pin, INPUT);
  
  // Motorları durdur
  stopMotors();
}

void setupRadioLink() {
  Serial.println("📡 RadioLink R12DS v1.1 Alıcısı Başlatılıyor...");
  
  // RadioLink kalibrasyonu
  calibrateRadioLink();
  
  Serial.println("✅ RadioLink Alıcısı Hazır!");
}

void calibrateRadioLink() {
  Serial.println("🔧 RadioLink Kalibrasyonu Başlıyor...");
  Serial.println("Kumandayı açın ve tüm kanalları merkeze getirin");
  delay(3000);
  
  // Merkez değerlerini oku
  for (int i = 0; i < 8; i++) {
    uint16_t value = readRadioLinkChannel(i);
    config.radiolink_center_values[i] = value;
    Serial.print("Kanal "); Serial.print(i + 1); Serial.print(": "); Serial.println(value);
  }
  
  Serial.println("✅ RadioLink Kalibrasyonu Tamamlandı");
}

uint16_t readRadioLinkChannel(int channel) {
  int pin;
  switch (channel) {
    case 0: pin = config.radiolink_ch1_pin; break;
    case 1: pin = config.radiolink_ch2_pin; break;
    case 2: pin = config.radiolink_ch3_pin; break;
    case 3: pin = config.radiolink_ch4_pin; break;
    case 4: pin = config.radiolink_ch5_pin; break;
    case 5: pin = config.radiolink_ch6_pin; break;
    case 6: pin = config.radiolink_ch7_pin; break;
    case 7: pin = config.radiolink_ch8_pin; break;
    default: return 1500;
  }
  
  return pulseIn(pin, HIGH, 25000); // 25ms timeout
}

void readRadioLink() {
  // Tüm kanalları oku
  radioStatus.throttle = readRadioLinkChannel(0);
  radioStatus.aileron = readRadioLinkChannel(1);
  radioStatus.elevator = readRadioLinkChannel(2);
  radioStatus.rudder = readRadioLinkChannel(3);
  radioStatus.gear = readRadioLinkChannel(4);
  radioStatus.aux1 = readRadioLinkChannel(5);
  radioStatus.aux2 = readRadioLinkChannel(6);
  radioStatus.aux3 = readRadioLinkChannel(7);
  
  // Sinyal kontrolü
  if (radioStatus.throttle > 0 && radioStatus.throttle < 3000) {
    radioStatus.connected = true;
    radioStatus.lastSignal = millis();
  } else {
    // 1 saniye sinyal yoksa bağlantıyı kes
    if (millis() - radioStatus.lastSignal > 1000) {
      radioStatus.connected = false;
    }
  }
}

void determineControlMode() {
  // RadioLink sinyali varsa ve AUX1 yüksekse manuel mod
  if (radioStatus.connected && radioStatus.aux1 > 1700) {
    currentMode = MODE_MANUAL;
  }
  // Kalibrasyon modunda ise
  else if (calibStatus.is_calibrating) {
    currentMode = MODE_CALIBRATION;
  }
  // Varsayılan otonom mod
  else {
    currentMode = MODE_AUTONOMOUS;
  }
}

void manualControl() {
  // Throttle (CH1) - İleri/Geri hız
  int throttle = map(radioStatus.throttle, 1000, 2000, -255, 255);
  
  // Rudder (CH4) - Dönüş
  int rudder = map(radioStatus.rudder, 1000, 2000, -255, 255);
  
  // Motor hızlarını hesapla
  int leftSpeed = throttle + rudder;
  int rightSpeed = throttle - rudder;
  
  // Hızları sınırla
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // Motorları sür
  setMotorSpeed(0, leftSpeed);
  setMotorSpeed(1, rightSpeed);
  
  // Motor durumunu güncelle
  motorStatus.left_speed = abs(leftSpeed);
  motorStatus.right_speed = abs(rightSpeed);
  motorStatus.left_direction = (leftSpeed < 0) ? 1 : 0;
  motorStatus.right_direction = (rightSpeed < 0) ? 1 : 0;
}

void calibrationMovement() {
  unsigned long currentTime = millis();
  
  // Hareket süresi doldu mu?
  if (currentTime - calibStatus.movement_start >= calibStatus.movement_duration) {
    // Yön değiştir
    calibStatus.movement_direction *= -1;
    calibStatus.movement_start = currentTime;
    
    // Hareket yönünü değiştir
    if (calibStatus.movement_direction == 1) {
      // Sağa hareket
      setMotorSpeed(0, calibStatus.movement_speed);
      setMotorSpeed(1, -calibStatus.movement_speed);
    } else {
      // Sola hareket
      setMotorSpeed(0, -calibStatus.movement_speed);
      setMotorSpeed(1, calibStatus.movement_speed);
    }
  }
}

void setupSensors() {
  // I²C başlat
  Wire.begin();
  Wire.setClock(400000); // 400kHz
  
  // MPU6050 başlat
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("❌ MPU6050 bulunamadı!");
    delay(1000);
  }
  Serial.println("✅ MPU6050 başlatıldı");
  
  // HMC5883L başlat
  while (!compass.begin()) {
    Serial.println("❌ HMC5883L bulunamadı!");
    delay(1000);
  }
  Serial.println("✅ HMC5883L başlatıldı");
  
  // BMP180 başlat
  while (!bmp.begin()) {
    Serial.println("❌ BMP180 bulunamadı!");
    delay(1000);
  }
  Serial.println("✅ BMP180 başlatıldı");
  
  // Sensör kalibrasyonu
  calibrateSensors();
}

void setupPID() {
  // PID parametrelerini ayarla
  linePID.SetMode(AUTOMATIC);
  linePID.SetOutputLimits(-config.pid_output_limit, config.pid_output_limit);
  linePID.SetTunings(config.pid_kp, config.pid_ki, config.pid_kd);
  
  Serial.println("✅ PID Kontrolcüsü Başlatıldı");
}

void setupI2C() {
  // I²C Master olarak başlat
  Wire.begin();
  Wire.setClock(400000);
  
  Serial.println("✅ I²C Master Başlatıldı");
}

void calibrateQTR() {
  Serial.println("🔧 QTR Sensörü Kalibre Ediliyor...");
  Serial.println("Araç sağa ve sola hareket ederek sensörü kalibre edecek");
  
  // Kalibrasyon modunu başlat
  calibStatus.is_calibrating = true;
  calibStatus.calibration_start = millis();
  calibStatus.movement_start = millis();
  calibStatus.movement_direction = 1; // Sağa başla
  calibStatus.movement_speed = 100;
  calibStatus.movement_duration = 2000; // 2 saniye
  
  // İlk hareketi başlat
  setMotorSpeed(0, calibStatus.movement_speed);
  setMotorSpeed(1, -calibStatus.movement_speed);
  
  // 400 kalibrasyon döngüsü (yaklaşık 8 saniye)
  for (int i = 0; i < 400; i++) {
    // Kalibrasyon hareketini kontrol et
    calibrationMovement();
    
    // QTR kalibrasyonu
    qtr.calibrate();
    
    delay(20);
  }
  
  // Motorları durdur
  stopMotors();
  
  // Kalibrasyon modunu bitir
  calibStatus.is_calibrating = false;
  
  // Kalibrasyon değerlerini kaydet
  for (int i = 0; i < 8; i++) {
    config.qtr_min_values[i] = qtr.calibratedMinimumOn[i];
    config.qtr_max_values[i] = qtr.calibratedMaximumOn[i];
  }
  
  // Konfigürasyonu kaydet
  saveConfigToEEPROM();
  
  Serial.println("✅ QTR Kalibrasyonu Tamamlandı");
}

void calibrateSensors() {
  Serial.println("🔧 IMU Sensörleri Kalibre Ediliyor...");
  
  // MPU6050 kalibrasyonu
  mpu.calibrateGyro();
  mpu.setThreshold(3);
  
  // HMC5883L kalibrasyonu
  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);
  
  Serial.println("✅ IMU Kalibrasyonu Tamamlandı");
}

void readSensors() {
  // QTR sensör değerlerini oku
  uint16_t sensorValues[8];
  qtr.read(sensorValues);
  
  // Çizgi pozisyonunu hesapla
  uint32_t position = qtr.readLine(sensorValues);
  input = position;
  
  // IMU verilerini oku
  Vector rawAccel = mpu.readRawAccel();
  Vector normAccel = mpu.readNormalizeAccel();
  Vector rawGyro = mpu.readRawGyro();
  Vector normGyro = mpu.readNormalizeGyro();
  
  // Magnetometre verilerini oku
  Vector mag = compass.readNormalize();
  
  // Basınç ve sıcaklık verilerini oku
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure();
  float altitude = bmp.readAltitude();
  
  // Pusula yönünü hesapla
  float heading = atan2(mag.YAxis, mag.XAxis);
  if (heading < 0) heading += 2 * PI;
  heading = heading * 180 / PI;
  
  // Motor durumunu güncelle
  updateMotorStatus();
}

void lineFollow() {
  // PID hesapla
  linePID.Compute();
  
  // Motor hızlarını hesapla
  int leftSpeed = 150 + output;
  int rightSpeed = 150 - output;
  
  // Hızları sınırla
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  // Motorları sür
  setMotorSpeed(0, leftSpeed);  // Sol motor
  setMotorSpeed(1, rightSpeed); // Sağ motor
  
  // Motor durumunu güncelle
  motorStatus.left_speed = leftSpeed;
  motorStatus.right_speed = rightSpeed;
  motorStatus.left_direction = 0;  // İleri
  motorStatus.right_direction = 0; // İleri
}

void setMotorSpeed(int motor, int speed) {
  if (motor == 0) { // Sol motor
    if (speed > 0) {
      digitalWrite(config.motor_left_dir, LOW);  // İleri
      digitalWrite(config.motor_left_brake, LOW);
      analogWrite(config.motor_left_pwm, speed);
    } else if (speed < 0) {
      digitalWrite(config.motor_left_dir, HIGH); // Geri
      digitalWrite(config.motor_left_brake, LOW);
      analogWrite(config.motor_left_pwm, -speed);
    } else {
      digitalWrite(config.motor_left_brake, HIGH);
      analogWrite(config.motor_left_pwm, 0);
    }
  } else { // Sağ motor
    if (speed > 0) {
      digitalWrite(config.motor_right_dir, LOW);  // İleri
      digitalWrite(config.motor_right_brake, LOW);
      analogWrite(config.motor_right_pwm, speed);
    } else if (speed < 0) {
      digitalWrite(config.motor_right_dir, HIGH); // Geri
      digitalWrite(config.motor_right_brake, LOW);
      analogWrite(config.motor_right_pwm, -speed);
    } else {
      digitalWrite(config.motor_right_brake, HIGH);
      analogWrite(config.motor_right_pwm, 0);
    }
  }
}

void stopMotors() {
  digitalWrite(config.motor_left_brake, HIGH);
  digitalWrite(config.motor_right_brake, HIGH);
  analogWrite(config.motor_left_pwm, 0);
  analogWrite(config.motor_right_pwm, 0);
}

void updateMotorStatus() {
  // Motor akımı ve sıcaklık simülasyonu
  motorStatus.left_current = motorStatus.left_speed * 0.003; // Amper
  motorStatus.right_current = motorStatus.right_speed * 0.003;
  motorStatus.left_temperature = 25.0 + (motorStatus.left_speed * 0.04);
  motorStatus.right_temperature = 25.0 + (motorStatus.right_speed * 0.04);
}

void sendDataToESP32() {
  // Sensör verilerini hazırla
  struct SensorData {
    uint8_t header[2];        // 0xAA, 0x55
    uint8_t device_id;        // 0x01 = Arduino Mega
    uint8_t data_type;        // 0x01 = IMU, 0x02 = QTR8A, 0x03 = Motor, 0x04 = RadioLink
    uint32_t timestamp;
    uint8_t data_length;
    uint8_t data[64];
    uint8_t checksum;
  } sensorData;
  
  // Header
  sensorData.header[0] = 0xAA;
  sensorData.header[1] = 0x55;
  sensorData.device_id = 0x01;
  sensorData.timestamp = millis();
  
  // IMU verilerini gönder
  sendIMUData();
  
  // QTR verilerini gönder
  sendQTRData();
  
  // Motor verilerini gönder
  sendMotorData();
  
  // RadioLink verilerini gönder
  sendRadioLinkData();
}

void sendIMUData() {
  Vector normAccel = mpu.readNormalizeAccel();
  Vector normGyro = mpu.readNormalizeGyro();
  Vector mag = compass.readNormalize();
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure();
  float altitude = bmp.readAltitude();
  
  struct IMUData {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
    float temperature;
    float pressure;
    float altitude;
    float heading;
  } imuData;
  
  imuData.accel_x = normAccel.XAxis;
  imuData.accel_y = normAccel.YAxis;
  imuData.accel_z = normAccel.ZAxis;
  imuData.gyro_x = normGyro.XAxis;
  imuData.gyro_y = normGyro.YAxis;
  imuData.gyro_z = normGyro.ZAxis;
  imuData.mag_x = mag.XAxis;
  imuData.mag_y = mag.YAxis;
  imuData.mag_z = mag.ZAxis;
  imuData.temperature = temperature;
  imuData.pressure = pressure;
  imuData.altitude = altitude;
  
  // I²C üzerinden gönder
  Wire.beginTransmission(I2C_SLAVE_ADDRESS);
  Wire.write(0xAA); // Header
  Wire.write(0x55);
  Wire.write(0x01); // Device ID
  Wire.write(0x01); // Data type: IMU
  Wire.write((uint8_t*)&millis(), 4); // Timestamp
  Wire.write(sizeof(IMUData)); // Data length
  Wire.write((uint8_t*)&imuData, sizeof(IMUData));
  Wire.endTransmission();
}

void sendQTRData() {
  uint16_t sensorValues[8];
  qtr.read(sensorValues);
  uint32_t position = qtr.readLine(sensorValues);
  
  struct QTR8AData {
    uint16_t sensor_values[8];
    uint16_t position;
    uint8_t calibrated;
    uint8_t line_detected;
  } qtrData;
  
  for (int i = 0; i < 8; i++) {
    qtrData.sensor_values[i] = sensorValues[i];
  }
  qtrData.position = position;
  qtrData.calibrated = 1;
  qtrData.line_detected = (position > 0) ? 1 : 0;
  
  // I²C üzerinden gönder
  Wire.beginTransmission(I2C_SLAVE_ADDRESS);
  Wire.write(0xAA); // Header
  Wire.write(0x55);
  Wire.write(0x01); // Device ID
  Wire.write(0x02); // Data type: QTR8A
  Wire.write((uint8_t*)&millis(), 4); // Timestamp
  Wire.write(sizeof(QTR8AData)); // Data length
  Wire.write((uint8_t*)&qtrData, sizeof(QTR8AData));
  Wire.endTransmission();
}

void sendMotorData() {
  struct MotorData {
    uint8_t left_speed;
    uint8_t right_speed;
    uint8_t left_direction;
    uint8_t right_direction;
    float left_current;
    float right_current;
    float left_temperature;
    float right_temperature;
  } motorData;
  
  motorData.left_speed = motorStatus.left_speed;
  motorData.right_speed = motorStatus.right_speed;
  motorData.left_direction = motorStatus.left_direction;
  motorData.right_direction = motorStatus.right_direction;
  motorData.left_current = motorStatus.left_current;
  motorData.right_current = motorStatus.right_current;
  motorData.left_temperature = motorStatus.left_temperature;
  motorData.right_temperature = motorStatus.right_temperature;
  
  // I²C üzerinden gönder
  Wire.beginTransmission(I2C_SLAVE_ADDRESS);
  Wire.write(0xAA); // Header
  Wire.write(0x55);
  Wire.write(0x01); // Device ID
  Wire.write(0x03); // Data type: Motor
  Wire.write((uint8_t*)&millis(), 4); // Timestamp
  Wire.write(sizeof(MotorData)); // Data length
  Wire.write((uint8_t*)&motorData, sizeof(MotorData));
  Wire.endTransmission();
}

void sendRadioLinkData() {
  struct RadioLinkData {
    uint16_t throttle;
    uint16_t aileron;
    uint16_t elevator;
    uint16_t rudder;
    uint16_t gear;
    uint16_t aux1;
    uint16_t aux2;
    uint16_t aux3;
    uint8_t connected;
    uint8_t control_mode;
  } radioData;
  
  radioData.throttle = radioStatus.throttle;
  radioData.aileron = radioStatus.aileron;
  radioData.elevator = radioStatus.elevator;
  radioData.rudder = radioStatus.rudder;
  radioData.gear = radioStatus.gear;
  radioData.aux1 = radioStatus.aux1;
  radioData.aux2 = radioStatus.aux2;
  radioData.aux3 = radioStatus.aux3;
  radioData.connected = radioStatus.connected ? 1 : 0;
  radioData.control_mode = currentMode;
  
  // I²C üzerinden gönder
  Wire.beginTransmission(I2C_SLAVE_ADDRESS);
  Wire.write(0xAA); // Header
  Wire.write(0x55);
  Wire.write(0x01); // Device ID
  Wire.write(0x04); // Data type: RadioLink
  Wire.write((uint8_t*)&millis(), 4); // Timestamp
  Wire.write(sizeof(RadioLinkData)); // Data length
  Wire.write((uint8_t*)&radioData, sizeof(RadioLinkData));
  Wire.endTransmission();
}

void checkI2CCommands() {
  // I²C'den gelen komutları kontrol et
  if (Wire.available() >= 4) {
    uint8_t header1 = Wire.read();
    uint8_t header2 = Wire.read();
    
    if (header1 == 0xBB && header2 == 0x66) {
      uint8_t command_type = Wire.read();
      uint8_t data_length = Wire.read();
      
      uint8_t data[32];
      for (int i = 0; i < data_length; i++) {
        data[i] = Wire.read();
      }
      
      processConfigCommand(command_type, data, data_length);
    }
  }
}

void processConfigCommand(uint8_t command_type, uint8_t* data, uint8_t length) {
  switch (command_type) {
    case 0x01: // Pin Config
      handlePinConfig(data, length);
      break;
    case 0x02: // PID Config
      handlePIDConfig(data, length);
      break;
    case 0x03: // Motor Command
      handleMotorCommand(data, length);
      break;
  }
}

void handlePinConfig(uint8_t* data, uint8_t length) {
  // Pin konfigürasyonunu güncelle
  if (length >= sizeof(config)) {
    memcpy(&config, data, sizeof(config));
    setupPins();
    saveConfigToEEPROM();
    Serial.println("✅ Pin Konfigürasyonu Güncellendi");
  }
}

void handlePIDConfig(uint8_t* data, uint8_t length) {
  // PID parametrelerini güncelle
  if (length >= 20) {
    config.pid_kp = *(float*)&data[0];
    config.pid_ki = *(float*)&data[4];
    config.pid_kd = *(float*)&data[8];
    config.pid_setpoint = *(float*)&data[12];
    config.pid_output_limit = *(float*)&data[16];
    
    linePID.SetTunings(config.pid_kp, config.pid_ki, config.pid_kd);
    setpoint = config.pid_setpoint;
    linePID.SetOutputLimits(-config.pid_output_limit, config.pid_output_limit);
    
    saveConfigToEEPROM();
    Serial.println("✅ PID Konfigürasyonu Güncellendi");
  }
}

void handleMotorCommand(uint8_t* data, uint8_t length) {
  // Motor komutlarını işle
  if (length >= 4) {
    uint8_t left_speed = data[0];
    uint8_t right_speed = data[1];
    uint8_t left_dir = data[2];
    uint8_t right_dir = data[3];
    
    // MQTT komutları sadece otonom modda çalışır
    if (currentMode == MODE_AUTONOMOUS || currentMode == MODE_MQTT) {
      setMotorSpeed(0, left_speed);
      setMotorSpeed(1, right_speed);
      
      motorStatus.left_speed = left_speed;
      motorStatus.right_speed = right_speed;
      motorStatus.left_direction = left_dir;
      motorStatus.right_direction = right_dir;
      
      Serial.println("✅ Motor Komutu Uygulandı");
    }
  }
}

void loadConfigFromEEPROM() {
  // EEPROM'dan konfigürasyonu yükle
  if (EEPROM.read(0) == 0xDE && EEPROM.read(1) == 0xAD) {
    EEPROM.get(2, config);
    Serial.println("✅ EEPROM'dan Konfigürasyon Yüklendi");
  } else {
    Serial.println("⚠️ EEPROM'da Konfigürasyon Bulunamadı, Varsayılan Kullanılıyor");
  }
}

void saveConfigToEEPROM() {
  // Konfigürasyonu EEPROM'a kaydet
  EEPROM.write(0, 0xDE);
  EEPROM.write(1, 0xAD);
  EEPROM.put(2, config);
  Serial.println("✅ Konfigürasyon EEPROM'a Kaydedildi");
} 