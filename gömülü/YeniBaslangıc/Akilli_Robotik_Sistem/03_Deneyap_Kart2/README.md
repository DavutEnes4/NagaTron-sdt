# 🔗 I²C Haberleşme Köprüsü

## 📋 Genel Bakış

Deneyap Kart 2 (ESP32) tabanlı I²C haberleşme köprüsü. Arduino Mega 2560 ile MQTT broker arasında veri aktarımı sağlar. Sensör verilerini MQTT'e yayınlar ve MQTT komutlarını Arduino'ya iletir.

## 🛠️ Donanım Gereksinimleri

### Ana Bileşenler
- **Deneyap Kart 2 (ESP32)** - I²C Slave ve MQTT Client
- **WiFi Modülü** - MQTT bağlantısı için
- **I²C Bağlantıları** - Arduino Mega ile haberleşme

### Pin Bağlantıları

#### I²C Bağlantıları
| Sinyal | Deneyap Kart 2 Pin | Arduino Mega Pin | Açıklama |
|--------|-------------------|------------------|----------|
| SDA | GPIO21 | Pin 20 | I²C Data |
| SCL | GPIO22 | Pin 21 | I²C Clock |
| VCC | 3.3V | 5V | Güç (Arduino tarafında 5V) |
| GND | GND | GND | Toprak |

## 📊 I²C Haberleşme Protokolü

### Arduino Mega → Deneyap Kart 2 (Alınan Veriler)

#### Veri Paketi Formatı
```
[Header: 2 byte] [Device ID: 1 byte] [Data Type: 1 byte] [Timestamp: 4 byte] [Data Length: 1 byte] [Data: N byte]
```

#### Veri Tipleri
- **0x01**: IMU Verisi (GY-89)
- **0x02**: QTR8A Verisi
- **0x03**: Motor Verisi
- **0x04**: RadioLink Verisi (YENİ)

#### IMU Verisi (0x01)
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

#### QTR8A Verisi (0x02)
```cpp
struct QTR8AData {
  uint16_t sensor_values[8];          // Sensör değerleri
  uint16_t position;                  // Çizgi pozisyonu
  uint8_t calibrated;                 // Kalibrasyon durumu
  uint8_t line_detected;              // Çizgi algılama
};
```

#### Motor Verisi (0x03)
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

#### RadioLink Verisi (0x04) - YENİ
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

### Deneyap Kart 2 → Arduino Mega (Gönderilen Komutlar)

#### Komut Paketi Formatı
```
[Header: 2 byte] [Command Type: 1 byte] [Data Length: 1 byte] [Data: N byte]
```

#### Komut Tipleri
- **0x01**: Pin Konfigürasyonu
- **0x02**: PID Konfigürasyonu
- **0x03**: Motor Komutu

#### Pin Konfigürasyonu (0x01)
```cpp
struct Config {
  // QTR pinleri
  uint8_t qtr_pins[8];
  uint8_t qtr_emitter_pin;
  
  // Motor pinleri
  uint8_t motor_left_pwm, motor_left_dir, motor_left_brake;
  uint8_t motor_right_pwm, motor_right_dir, motor_right_brake;
  
  // I²C pinleri
  uint8_t i2c_sda, i2c_scl;
  
  // RadioLink pinleri (YENİ)
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

#### PID Konfigürasyonu (0x02)
```cpp
struct PIDConfig {
  float kp, ki, kd;                  // PID parametreleri
  float setpoint;                     // Hedef değer
  float output_limit;                 // Çıkış sınırı
};
```

#### Motor Komutu (0x03)
```cpp
struct MotorCommand {
  uint8_t left_speed;                 // Sol motor hızı
  uint8_t right_speed;                // Sağ motor hızı
  uint8_t left_direction;             // Sol motor yönü
  uint8_t right_direction;            // Sağ motor yönü
};
```

## 📡 MQTT Haberleşme

### Yayınlanan Konular (Published Topics)

#### Sensör Verileri
| Konu | Açıklama | Veri Formatı |
|------|----------|--------------|
| `sensors/arduino_mega/gy89` | IMU verileri | JSON |
| `sensors/arduino_mega/qtr8a` | QTR sensör verileri | JSON |
| `sensors/arduino_mega/motors` | Motor durumu | JSON |
| `sensors/arduino_mega/radiolink` | RadioLink verileri (YENİ) | JSON |

#### Sistem Durumu
| Konu | Açıklama | Veri Formatı |
|------|----------|--------------|
| `system/deneyap_kart_2/status` | Sistem durumu | JSON |

### Abone Olunan Konular (Subscribed Topics)

#### Komut Konuları
| Konu | Açıklama | Veri Formatı |
|------|----------|--------------|
| `command/motors/arduino_mega` | Motor komutları | JSON |
| `config/pins/arduino_mega` | Pin konfigürasyonu | JSON |
| `config/pid/arduino_mega` | PID konfigürasyonu | JSON |

### JSON Veri Formatları

#### IMU Verisi
```json
{
  "device_id": "arduino_mega",
  "sensor_type": "gy89",
  "timestamp": 1234567890,
  "data": {
    "accel_x": 0.5,
    "accel_y": -0.2,
    "accel_z": 9.8,
    "gyro_x": 0.1,
    "gyro_y": 0.05,
    "gyro_z": 0.02,
    "mag_x": 25.3,
    "mag_y": -12.1,
    "mag_z": 45.7,
    "temperature": 23.5,
    "pressure": 101325.0,
    "altitude": 100.5,
    "heading": 180.0
  }
}
```

#### QTR8A Verisi
```json
{
  "device_id": "arduino_mega",
  "sensor_type": "qtr8a",
  "timestamp": 1234567890,
  "data": {
    "sensor_values": [100, 150, 200, 250, 300, 350, 400, 450],
    "position": 3500,
    "calibrated": 1,
    "line_detected": 1
  }
}
```

#### Motor Verisi
```json
{
  "device_id": "arduino_mega",
  "sensor_type": "motors",
  "timestamp": 1234567890,
  "data": {
    "left_speed": 150,
    "right_speed": 145,
    "left_direction": 0,
    "right_direction": 0,
    "left_current": 0.45,
    "right_current": 0.43,
    "left_temperature": 28.5,
    "right_temperature": 27.8
  }
}
```

#### RadioLink Verisi (YENİ)
```json
{
  "device_id": "arduino_mega",
  "sensor_type": "radiolink_r12ds",
  "timestamp": 1234567890,
  "data": {
    "throttle": 1500,
    "aileron": 1500,
    "elevator": 1500,
    "rudder": 1500,
    "gear": 1500,
    "aux1": 1700,
    "aux2": 1500,
    "aux3": 1500,
    "connected": 1,
    "control_mode": 1
  }
}
```

#### Motor Komutu
```json
{
  "device_id": "arduino_mega",
  "command_type": "motor_control",
  "left_speed": 200,
  "right_speed": 200,
  "left_direction": 0,
  "right_direction": 0
}
```

## 🚀 Özellikler

### ✅ Mevcut Özellikler
- **I²C Slave Modu**: Arduino Mega'dan veri alma
- **MQTT Client**: Broker ile haberleşme
- **Çoklu WiFi Desteği**: WiFiMulti ile otomatik bağlanma
- **Otomatik Yeniden Bağlanma**: Bağlantı kopması durumunda
- **JSON Veri Formatı**: Standart veri yapısı
- **Gerçek Zamanlı Veri Aktarımı**: 100ms aralıklarla
- **RadioLink Veri Aktarımı**: R12DS v1.1 desteği (YENİ)

### 🔧 Yeni Özellikler
- **RadioLink R12DS v1.1 Desteği**: Uzaktan kumanda verilerini MQTT'e aktarma
- **Kontrol Modu Takibi**: Arduino'nun hangi modda çalıştığını izleme
- **Bağlantı Durumu**: RadioLink sinyal durumunu takip etme

## ⚙️ Kurulum

### 1. WiFi Ayarları
```cpp
const char* ssid1 = "WiFi_SSID_1";
const char* password1 = "WiFi_Password_1";
const char* ssid2 = "WiFi_SSID_2";
const char* password2 = "WiFi_Password_2";
```

### 2. MQTT Ayarları
```cpp
const char* mqtt_server = "broker.mqtt.com";
const int mqtt_port = 1883;
const char* mqtt_username = "your_username";
const char* mqtt_password = "your_password";
const char* device_id = "deneyap_kart_2";
```

### 3. I²C Ayarları
```cpp
#define I2C_SLAVE_ADDRESS 0x10
#define ARDUINO_MEGA_ADDRESS 0x01
```

## 🧪 Test ve Doğrulama

### I²C Haberleşme Testi
1. Arduino Mega ile I²C bağlantısını kontrol edin
2. Serial monitörde veri alımını doğrulayın
3. Veri paketlerinin doğru formatını kontrol edin

### MQTT Bağlantı Testi
1. WiFi bağlantısını kontrol edin
2. MQTT broker'a bağlanmayı doğrulayın
3. Veri yayınlama ve komut alma testini yapın

### RadioLink Testi (YENİ)
1. Arduino Mega'da RadioLink verilerinin okunduğunu kontrol edin
2. MQTT'de `sensors/arduino_mega/radiolink` konusunda veri yayınlandığını doğrulayın
3. Bağlantı durumu ve kontrol modu bilgilerini kontrol edin

## 🐛 Hata Ayıklama

### Yaygın Sorunlar
- **I²C Haberleşme Hatası**: SDA/SCL bağlantılarını ve adresleri kontrol edin
- **MQTT Bağlantı Sorunu**: WiFi ve broker ayarlarını kontrol edin
- **Veri Kaybı**: Zamanlama ayarlarını optimize edin
- **RadioLink Veri Eksikliği**: Arduino Mega'da RadioLink okuma fonksiyonlarını kontrol edin

### Debug Mesajları
```cpp
Serial.println("🔗 I²C Haberleşme Köprüsü Başlatılıyor...");
Serial.println("📡 RadioLink Verisi Yayınlandı");
Serial.println("✅ Deneyap Kart 2 Sistemi Hazır!");
```

## 📈 Performans

### Zamanlama
- **I²C Veri Alma**: 50ms (20Hz)
- **MQTT Veri Yayınlama**: 100ms (10Hz)
- **WiFi Yeniden Bağlanma**: 5 saniye
- **JSON İşleme**: < 1ms

### Bellek Kullanımı
- **Program Belleği**: ~8KB
- **RAM**: ~1.5KB
- **JSON Buffer**: 512 byte

## 🔒 Güvenlik

### Veri Bütünlüğü
- I²C checksum kontrolü
- JSON format doğrulama
- Veri uzunluğu kontrolü

### Bağlantı Güvenliği
- WiFi şifreleme
- MQTT kullanıcı adı/şifre
- Otomatik yeniden bağlanma

## 📝 Lisans

Bu proje eğitim amaçlıdır. Donanım yapılandırmaları açık kaynaklıdır ve geliştirilebilir.

---

**Son Güncelleme**: RadioLink R12DS v1.1 veri aktarımı eklendi. 