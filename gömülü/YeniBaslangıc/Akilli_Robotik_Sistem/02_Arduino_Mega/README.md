# 🤖 Arduino Mega Robotik Kontrol Sistemi

## 📋 Genel Bakış

Arduino Mega 2560 tabanlı robotik kontrol sistemi. QTR-8A çizgi sensörü, GY-89 9-eksenli IMU, BTS7960B motor sürücüleri ve RadioLink R12DS v1.1 uzaktan kumanda desteği ile çalışır.

## 🛠️ Donanım Gereksinimleri

### Ana Bileşenler
- **Arduino Mega 2560** - Ana kontrolcü
- **QTR-8A** - Analog çizgi sensörü (8 sensör)
- **GY-89 Sensör Modülü**:
  - MPU6050 (ivmeölçer + jiroskop)
  - HMC5883L (magnetometre)
  - BMP180 (basınç + sıcaklık)
- **BTS7960B Motor Sürücüleri** (x2)
- **RadioLink R12DS v1.1** - Uzaktan kumanda ve alıcı

### Pin Bağlantıları

#### QTR-8A Sensörü
| QTR Pin | Arduino Pin | Açıklama |
|---------|-------------|----------|
| VCC | 5V | Güç |
| GND | GND | Toprak |
| EMIT | Pin 2 | Emisyon kontrolü |
| S1 | A0 | Sensör 1 |
| S2 | A1 | Sensör 2 |
| S3 | A2 | Sensör 3 |
| S4 | A3 | Sensör 4 |
| S5 | A4 | Sensör 5 |
| S6 | A5 | Sensör 6 |
| S7 | A6 | Sensör 7 |
| S8 | A7 | Sensör 8 |

#### BTS7960B Motor Sürücüleri
| Motor | PWM Pin | DIR Pin | BRAKE Pin | Açıklama |
|-------|---------|---------|-----------|----------|
| Sol | Pin 9 | Pin 8 | Pin 7 | Sol motor kontrolü |
| Sağ | Pin 10 | Pin 11 | Pin 12 | Sağ motor kontrolü |

#### RadioLink R12DS v1.1 Alıcısı
| Kanal | Arduino Pin | Açıklama |
|-------|-------------|----------|
| CH1 (Throttle) | Pin 22 | İleri/Geri hız |
| CH2 (Aileron) | Pin 23 | Roll kontrolü |
| CH3 (Elevator) | Pin 24 | Pitch kontrolü |
| CH4 (Rudder) | Pin 25 | Yaw kontrolü |
| CH5 (Gear) | Pin 26 | Gear kontrolü |
| CH6 (Aux1) | Pin 27 | Mod değiştirme |
| CH7 (Aux2) | Pin 28 | Ek fonksiyon |
| CH8 (Aux3) | Pin 29 | Ek fonksiyon |

#### I²C Bağlantıları
| Sinyal | Arduino Pin | Açıklama |
|--------|-------------|----------|
| SDA | Pin 20 | I²C Data |
| SCL | Pin 21 | I²C Clock |

## 🔧 Kontrol Modları

### 1. Otonom Mod (MODE_AUTONOMOUS)
- PID tabanlı çizgi takibi
- QTR-8A sensörü ile çizgi algılama
- Otomatik hız ve yön kontrolü

### 2. Manuel Mod (MODE_MANUAL)
- RadioLink R12DS v1.1 ile manuel kontrol
- AUX1 kanalı ile mod değiştirme
- Throttle (CH1) ve Rudder (CH4) ile hareket

### 3. MQTT Mod (MODE_MQTT)
- MQTT komutları ile uzaktan kontrol
- I²C üzerinden Deneyap Kart 2'den komut alma

### 4. Kalibrasyon Mod (MODE_CALIBRATION)
- QTR sensörü kalibrasyonu sırasında araç hareketi
- Sağa-sola hareket ile sensör kalibrasyonu

## 📊 I²C Veri Formatları

### Arduino Mega → Deneyap Kart 2 (Gönderilen Veriler)

#### IMU Verisi (Data Type: 0x01)
```cpp
struct IMUData {
  float accel_x, accel_y, accel_z;    // İvme (m/s²)
  float gyro_x, gyro_y, gyro_z;       // Açısal hız (rad/s)
  float mag_x, mag_y, mag_z;          // Manyetik alan (μT)
  float temperature;                   // Sıcaklık (°C)
  float pressure;                     // Basınç (Pa)
  float altitude;                     // Yükseklik (m)
  float heading;                      // Yön (derece)
};
```

#### QTR8A Verisi (Data Type: 0x02)
```cpp
struct QTR8AData {
  uint16_t sensor_values[8];          // Sensör değerleri
  uint16_t position;                  // Çizgi pozisyonu
  uint8_t calibrated;                 // Kalibrasyon durumu
  uint8_t line_detected;              // Çizgi algılama
};
```

#### Motor Verisi (Data Type: 0x03)
```cpp
struct MotorData {
  uint8_t left_speed;                 // Sol motor hızı
  uint8_t right_speed;                // Sağ motor hızı
  uint8_t left_direction;             // Sol motor yönü
  uint8_t right_direction;            // Sağ motor yönü
  float left_current;                 // Sol motor akımı
  float right_current;                // Sağ motor akımı
  float left_temperature;             // Sol motor sıcaklığı
  float right_temperature;            // Sağ motor sıcaklığı
};
```

#### RadioLink Verisi (Data Type: 0x04)
```cpp
struct RadioLinkData {
  uint16_t throttle;                  // CH1 - Throttle
  uint16_t aileron;                   // CH2 - Aileron
  uint16_t elevator;                  // CH3 - Elevator
  uint16_t rudder;                    // CH4 - Rudder
  uint16_t gear;                      // CH5 - Gear
  uint16_t aux1;                      // CH6 - Aux1
  uint16_t aux2;                      // CH7 - Aux2
  uint16_t aux3;                      // CH8 - Aux3
  uint8_t connected;                  // Bağlantı durumu
  uint8_t control_mode;               // Kontrol modu
};
```

### Deneyap Kart 2 → Arduino Mega (Alınan Komutlar)

#### Pin Konfigürasyonu (Command Type: 0x01)
```cpp
// Tüm pin konfigürasyonu
struct Config {
  // QTR pinleri
  uint8_t qtr_pins[8];
  uint8_t qtr_emitter_pin;
  
  // Motor pinleri
  uint8_t motor_left_pwm, motor_left_dir, motor_left_brake;
  uint8_t motor_right_pwm, motor_right_dir, motor_right_brake;
  
  // I²C pinleri
  uint8_t i2c_sda, i2c_scl;
  
  // RadioLink pinleri
  uint8_t radiolink_ch1_pin, radiolink_ch2_pin, radiolink_ch3_pin, radiolink_ch4_pin;
  uint8_t radiolink_ch5_pin, radiolink_ch6_pin, radiolink_ch7_pin, radiolink_ch8_pin;
  
  // PID parametreleri
  double pid_kp, pid_ki, pid_kd, pid_setpoint, pid_output_limit;
  
  // Kalibrasyon değerleri
  uint16_t qtr_min_values[8], qtr_max_values[8];
  float accel_offset[3], gyro_offset[3];
  uint16_t radiolink_min_values[8], radiolink_max_values[8], radiolink_center_values[8];
};
```

#### PID Konfigürasyonu (Command Type: 0x02)
```cpp
struct PIDConfig {
  float kp, ki, kd;                  // PID parametreleri
  float setpoint;                     // Hedef değer
  float output_limit;                 // Çıkış sınırı
};
```

#### Motor Komutu (Command Type: 0x03)
```cpp
struct MotorCommand {
  uint8_t left_speed;                 // Sol motor hızı
  uint8_t right_speed;                // Sağ motor hızı
  uint8_t left_direction;             // Sol motor yönü
  uint8_t right_direction;            // Sağ motor yönü
};
```

## 🚀 Özellikler

### ✅ Mevcut Özellikler
- **PID Tabanlı Çizgi Takibi**: Hassas çizgi takibi
- **9-Eksenli IMU**: Tam hareket algılama
- **Çoklu Kontrol Modu**: Otonom, Manuel, MQTT
- **RadioLink R12DS v1.1 Desteği**: Uzaktan kumanda
- **QTR Kalibrasyon Hareketi**: Otomatik sağa-sola hareket
- **EEPROM Konfigürasyonu**: Kalıcı ayarlar
- **I²C Haberleşme**: Deneyap Kart 2 ile veri aktarımı
- **Gerçek Zamanlı Veri**: 50Hz sensör okuma
- **Güvenli Motor Kontrolü**: Akım ve sıcaklık izleme

### 🔧 Kalibrasyon Özellikleri
- **QTR Kalibrasyonu**: Otomatik sağa-sola hareket ile
- **RadioLink Kalibrasyonu**: Merkez değerleri otomatik ayarlama
- **IMU Kalibrasyonu**: Sensör offset değerleri
- **Motor Kalibrasyonu**: Hız ve yön kalibrasyonu

## ⚙️ Kurulum

### 1. Kütüphaneler
```cpp
#include <Wire.h>           // I²C haberleşme
#include <QTRSensors.h>     // QTR-8A sensör
#include <PID_v1.h>         // PID kontrol
#include <MPU6050.h>        // MPU6050 IMU
#include <HMC5883L.h>       // HMC5883L magnetometre
#include <BMP180.h>         // BMP180 barometre
#include <EEPROM.h>         // EEPROM depolama
#include <ArduinoJson.h>    // JSON işleme
#include <Servo.h>          // Servo kontrol (gelecek)
```

### 2. Pin Konfigürasyonu
- Tüm pinler `config` yapısında tanımlanmıştır
- MQTT üzerinden dinamik olarak değiştirilebilir
- EEPROM'da kalıcı olarak saklanır

### 3. Başlangıç Ayarları
```cpp
void setup() {
  loadConfigFromEEPROM();    // EEPROM'dan ayarları yükle
  setupPins();               // Pin modlarını ayarla
  setupSensors();            // Sensörleri başlat
  setupPID();                // PID kontrolcüsünü ayarla
  setupI2C();                // I²C Master başlat
  setupRadioLink();          // RadioLink alıcısını başlat
  calibrateQTR();            // QTR kalibrasyonu
}
```

## 🧪 Test ve Doğrulama

### QTR Kalibrasyon Testi
1. Aracı çizgi üzerine yerleştirin
2. Sistemi başlatın
3. Araç otomatik olarak sağa-sola hareket edecek
4. Kalibrasyon tamamlandığında motorlar durur

### RadioLink Testi
1. RadioLink kumandasını açın
2. AUX1 kanalını yüksek konuma getirin
3. Throttle ve Rudder ile hareket testi yapın
4. Bağlantı durumunu Serial monitörden kontrol edin

### I²C Haberleşme Testi
1. Deneyap Kart 2 ile bağlantıyı kontrol edin
2. MQTT broker'a bağlanın
3. Sensör verilerinin yayınlandığını doğrulayın

## 🐛 Hata Ayıklama

### Yaygın Sorunlar
- **QTR Kalibrasyon Hatası**: Sensör bağlantılarını kontrol edin
- **RadioLink Bağlantı Sorunu**: Alıcı pinlerini ve kumanda ayarlarını kontrol edin
- **I²C Haberleşme Hatası**: SDA/SCL bağlantılarını ve adresleri kontrol edin
- **Motor Çalışmıyor**: Motor sürücü bağlantılarını ve güç kaynağını kontrol edin

### Debug Mesajları
```cpp
Serial.println("🔧 QTR Sensörü Kalibre Ediliyor...");
Serial.println("📡 RadioLink R12DS v1.1 Alıcısı Başlatılıyor...");
Serial.println("✅ Arduino Mega Sistemi Hazır!");
```

## 📈 Performans

### Zamanlama
- **Sensör Okuma**: 100ms (10Hz)
- **I²C Veri Aktarımı**: 50ms (20Hz)
- **RadioLink Okuma**: 20ms (50Hz)
- **PID Hesaplama**: 10ms (100Hz)

### Bellek Kullanımı
- **Program Belleği**: ~15KB
- **RAM**: ~2KB
- **EEPROM**: ~512 byte konfigürasyon

## 🔒 Güvenlik

### Motor Güvenliği
- Akım sınırlama
- Sıcaklık izleme
- Acil durdurma fonksiyonu

### RadioLink Güvenliği
- Sinyal kaybı algılama
- Otomatik güvenli mod
- Bağlantı durumu izleme

## 📝 Lisans

Bu proje eğitim amaçlıdır. Donanım yapılandırmaları açık kaynaklıdır ve geliştirilebilir.

---

**Son Güncelleme**: RadioLink R12DS v1.1 desteği ve QTR kalibrasyon hareketi eklendi. 