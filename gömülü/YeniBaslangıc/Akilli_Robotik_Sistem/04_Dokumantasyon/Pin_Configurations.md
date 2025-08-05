# 🔧 Pin Konfigürasyonları

## 📋 Genel Bakış

Bu dokümantasyon, Akıllı Robotik İzleme ve Erişim Sistemi'nin tüm pin konfigürasyonlarını detaylandırır. Tüm pinler MQTT üzerinden kontrol edilebilir ve EEPROM'a kayıt edilebilir.

## 🔌 Arduino Mega Pin Mappings

### QTR-8A Çizgi Sensörü
| QTR-8A Pin | Arduino Mega Pin | Açıklama | MQTT Kontrolü |
|-------------|------------------|----------|---------------|
| Sensor 1 | A0 | Analog Input | ✅ |
| Sensor 2 | A1 | Analog Input | ✅ |
| Sensor 3 | A2 | Analog Input | ✅ |
| Sensor 4 | A3 | Analog Input | ✅ |
| Sensor 5 | A4 | Analog Input | ✅ |
| Sensor 6 | A5 | Analog Input | ✅ |
| Sensor 7 | A6 | Analog Input | ✅ |
| Sensor 8 | A7 | Analog Input | ✅ |
| Emitter | 2 | Digital Output | ✅ |
| VCC | 5V | Güç | ❌ |
| GND | GND | Toprak | ❌ |

### BTS7960B Motor Sürücüleri
| BTS7960B Pin | Arduino Mega Pin | Açıklama | MQTT Kontrolü |
|---------------|------------------|----------|---------------|
| Sol Motor PWM | 9 | PWM Output | ✅ |
| Sol Motor Dir | 8 | Digital Output | ✅ |
| Sol Motor Brake | 7 | Digital Output | ✅ |
| Sağ Motor PWM | 10 | PWM Output | ✅ |
| Sağ Motor Dir | 11 | Digital Output | ✅ |
| Sağ Motor Brake | 12 | Digital Output | ✅ |
| VCC | 12V | Güç | ❌ |
| GND | GND | Toprak | ❌ |

### I²C Bağlantıları
| Sensör | Arduino Mega Pin | Açıklama | MQTT Kontrolü |
|--------|------------------|----------|---------------|
| I²C SDA | 20 | I²C Data | ✅ |
| I²C SCL | 21 | I²C Clock | ✅ |
| MPU6050 | I²C (0x68) | İvme ve Jiroskop | ✅ |
| HMC5883L | I²C (0x1E) | Magnetometre | ✅ |
| BMP180 | I²C (0x77) | Basınç ve Sıcaklık | ✅ |

## 🔌 Deneyap Kart 1 (RFID) Pin Mappings

### RC522 RFID Sensörü
| RC522 Pin | Deneyap Kart 1 Pin | Açıklama | MQTT Kontrolü |
|-----------|-------------------|----------|---------------|
| SDA | GPIO5 | SPI Data | ✅ |
| SCK | GPIO18 | SPI Clock | ✅ |
| MOSI | GPIO23 | SPI MOSI | ✅ |
| MISO | GPIO19 | SPI MISO | ✅ |
| RST | GPIO22 | Reset | ✅ |
| IRQ | GPIO21 | Interrupt | ✅ |
| VCC | 3.3V | Güç | ❌ |
| GND | GND | Toprak | ❌ |

### Sistem Pinleri
| Bileşen | Deneyap Kart 1 Pin | Açıklama | MQTT Kontrolü |
|---------|-------------------|----------|---------------|
| Status LED | GPIO2 | Sistem Durumu | ✅ |
| Debug UART | GPIO1/3 | Serial Debug | ✅ |

## 🔌 Deneyap Kart 2 (I²C Bridge) Pin Mappings

### I²C Bağlantıları
| Arduino Mega Pin | Deneyap Kart 2 Pin | Açıklama | MQTT Kontrolü |
|------------------|-------------------|----------|---------------|
| SDA (20) | GPIO21 | I²C Data | ✅ |
| SCL (21) | GPIO22 | I²C Clock | ✅ |
| VCC | 3.3V | Güç | ❌ |
| GND | GND | Toprak | ❌ |

### Sistem Pinleri
| Bileşen | Deneyap Kart 2 Pin | Açıklama | MQTT Kontrolü |
|---------|-------------------|----------|---------------|
| Status LED | GPIO2 | Sistem Durumu | ✅ |
| Debug UART | GPIO1/3 | Serial Debug | ✅ |

## 📡 MQTT Pin Konfigürasyonu

### Pin Değiştirme Komutu
```json
{
  "topic": "config/pins/arduino_mega",
  "payload": {
    "command": "update_pin",
    "component": "qtr8a",
    "pin_type": "emitter",
    "new_pin": 3
  }
}
```

### Toplu Pin Konfigürasyonu
```json
{
  "topic": "config/pins/arduino_mega",
  "payload": {
    "device": "arduino_mega",
    "pins": {
      "qtr8a": {
        "sensor_pins": [A0, A1, A2, A3, A4, A5, A6, A7],
        "emitter_pin": 2
      },
      "motors": {
        "left_motor": {
          "pwm_pin": 9,
          "direction_pin": 8,
          "brake_pin": 7
        },
        "right_motor": {
          "pwm_pin": 10,
          "direction_pin": 11,
          "brake_pin": 12
        }
      },
      "i2c": {
        "sda_pin": 20,
        "scl_pin": 21
      }
    }
  }
}
```

## 🔧 EEPROM Konfigürasyon Yapısı

### Arduino Mega EEPROM Yapısı
```cpp
struct EEPROMConfig {
  uint32_t magic_number;    // 0xDEADBEEF
  uint8_t version;          // 1
  uint8_t device_id;        // 0x01 = Arduino Mega
  
  // Pin konfigürasyonları
  struct PinConfig {
    uint8_t qtr8a_pins[8];
    uint8_t qtr8a_emitter_pin;
    uint8_t motor_left_pwm;
    uint8_t motor_left_dir;
    uint8_t motor_left_brake;
    uint8_t motor_right_pwm;
    uint8_t motor_right_dir;
    uint8_t motor_right_brake;
    uint8_t i2c_sda;
    uint8_t i2c_scl;
  } pins;
  
  // PID parametreleri
  struct PIDConfig {
    float kp, ki, kd;
    float setpoint;
    float output_limit;
  } pid_line_following, pid_motor_control;
  
  // Sensör kalibrasyonu
  struct SensorCalibration {
    float accel_offset[3];
    float gyro_offset[3];
    uint16_t qtr8a_min[8];
    uint16_t qtr8a_max[8];
  } calibration;
  
  uint16_t checksum;
};
```

### Deneyap Kart 1 EEPROM Yapısı
```cpp
struct EEPROMConfig {
  uint32_t magic_number;    // 0xDEADBEEF
  uint8_t version;          // 1
  uint8_t device_id;        // 0x02 = Deneyap Kart 1
  
  // RFID pin konfigürasyonları
  struct RFIDPinConfig {
    uint8_t rfid_sda;
    uint8_t rfid_sck;
    uint8_t rfid_mosi;
    uint8_t rfid_miso;
    uint8_t rfid_rst;
    uint8_t rfid_irq;
  } rfid_pins;
  
  // WiFi ayarları
  struct WiFiConfig {
    char ssid1[32];
    char password1[64];
    char ssid2[32];
    char password2[64];
  } wifi_config;
  
  // MQTT ayarları
  struct MQTTConfig {
    char server[64];
    uint16_t port;
    char username[32];
    char password[64];
  } mqtt_config;
  
  uint16_t checksum;
};
```

### Deneyap Kart 2 EEPROM Yapısı
```cpp
struct EEPROMConfig {
  uint32_t magic_number;    // 0xDEADBEEF
  uint8_t version;          // 1
  uint8_t device_id;        // 0x03 = Deneyap Kart 2
  
  // I²C pin konfigürasyonları
  struct I2CPinConfig {
    uint8_t i2c_sda;
    uint8_t i2c_scl;
  } i2c_pins;
  
  // WiFi ayarları
  struct WiFiConfig {
    char ssid1[32];
    char password1[64];
    char ssid2[32];
    char password2[64];
  } wifi_config;
  
  // MQTT ayarları
  struct MQTTConfig {
    char server[64];
    uint16_t port;
    char username[32];
    char password[64];
  } mqtt_config;
  
  uint16_t checksum;
};
```

## 🔧 Pin Değişiklik Protokolü

### 1. MQTT Komutu Gönderme
```json
{
  "topic": "config/pins/arduino_mega",
  "payload": {
    "command": "update_pin",
    "component": "qtr8a",
    "pin_type": "emitter",
    "new_pin": 3
  }
}
```

### 2. Sistem Yanıtı
```json
{
  "topic": "config/pins/arduino_mega/response",
  "payload": {
    "device": "arduino_mega",
    "command": "update_pin",
    "component": "qtr8a",
    "pin_type": "emitter",
    "old_pin": 2,
    "new_pin": 3,
    "status": "success",
    "timestamp": 1640995200000
  }
}
```

### 3. Hata Yanıtı
```json
{
  "topic": "config/pins/arduino_mega/error",
  "payload": {
    "device": "arduino_mega",
    "command": "update_pin",
    "error": "invalid_pin",
    "message": "Pin 3 is already in use",
    "timestamp": 1640995200000
  }
}
```

## 📊 Pin Durumu İzleme

### Pin Durumu Sorgulama
```json
{
  "topic": "config/pins/arduino_mega/status",
  "payload": {
    "device": "arduino_mega",
    "command": "get_pin_status",
    "component": "all"
  }
}
```

### Pin Durumu Yanıtı
```json
{
  "topic": "config/pins/arduino_mega/status/response",
  "payload": {
    "device": "arduino_mega",
    "timestamp": 1640995200000,
    "pins": {
      "qtr8a": {
        "sensor_pins": [A0, A1, A2, A3, A4, A5, A6, A7],
        "emitter_pin": 2,
        "status": "active"
      },
      "motors": {
        "left_motor": {
          "pwm_pin": 9,
          "direction_pin": 8,
          "brake_pin": 7,
          "status": "active"
        },
        "right_motor": {
          "pwm_pin": 10,
          "direction_pin": 11,
          "brake_pin": 12,
          "status": "active"
        }
      },
      "i2c": {
        "sda_pin": 20,
        "scl_pin": 21,
        "status": "active"
      }
    }
  }
}
```

## 🔒 Güvenlik

### Pin Çakışması Kontrolü
- Aynı pin birden fazla bileşene atanamaz
- PWM pinleri sadece motor kontrolü için kullanılabilir
- I²C pinleri sadece I²C haberleşme için kullanılabilir
- Analog pinler sadece analog sensörler için kullanılabilir

### Geçerlilik Kontrolü
- Pin numaraları 0-53 arasında olmalı
- PWM pinleri: 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13
- Analog pinler: A0-A15
- I²C pinleri: 20 (SDA), 21 (SCL)

### Hata Toleransı
- Geçersiz pin değişiklikleri reddedilir
- Sistem çalışırken kritik pinler değiştirilemez
- Pin değişikliği sonrası sistem yeniden başlatılır

## 📈 Performans

### Pin Değişiklik Süreleri
- **Tek Pin Değişikliği:** < 100ms
- **Toplu Pin Değişikliği:** < 500ms
- **Sistem Yeniden Başlatma:** < 2s
- **EEPROM Yazma:** < 50ms

### Bellek Kullanımı
- **EEPROM Konfigürasyonu:** ~256 bytes
- **Pin Durumu:** ~128 bytes
- **Geçici Veri:** ~64 bytes

## 📄 Lisans

Bu konfigürasyon sistemi eğitim amaçlıdır. Endüstriyel uygulamalarda ek güvenlik önlemleri alınmalıdır. 