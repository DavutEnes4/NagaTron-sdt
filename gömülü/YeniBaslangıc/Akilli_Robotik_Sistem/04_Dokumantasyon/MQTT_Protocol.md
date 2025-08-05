# 📡 MQTT Haberleşme Protokolü

## 📋 Genel Bakış

Bu dokümantasyon, Akıllı Robotik İzleme ve Erişim Sistemi'nin MQTT haberleşme protokolünü detaylandırır. Sistem tamamen MQTT tabanlı haberleşme kullanır ve tüm pin konfigürasyonları MQTT ile kontrol edilebilir.

## 🔧 MQTT Broker Ayarları

### Broker Bilgileri
- **Broker Adresi:** `broker.mqtt.com`
- **Port:** `1883`
- **Protokol:** MQTT 3.1.1
- **QoS:** 1 (En az bir kez teslim)
- **Keep Alive:** 60 saniye

### Güvenlik
- **Kullanıcı Adı:** `your_username`
- **Şifre:** `your_password`
- **Client ID:** Device ID (örn: `deneyap_kart_1`)

## 📡 MQTT Konuları

### 🔧 Konfigürasyon Konuları

#### Pin Konfigürasyonu
- **Topic:** `config/pins/{device_id}`
- **QoS:** 1
- **Retain:** false
- **Format:** JSON

**Örnek Payload:**
```json
{
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
```

#### Sensör Konfigürasyonu
- **Topic:** `config/sensors/{device_id}`
- **QoS:** 1
- **Retain:** false
- **Format:** JSON

**Örnek Payload:**
```json
{
  "device": "arduino_mega",
  "sensors": {
    "gy89": {
      "i2c_address": "0x68",
      "update_rate": 100,
      "calibration": {
        "accelerometer": {
          "offset_x": 0.0,
          "offset_y": 0.0,
          "offset_z": 0.0
        },
        "gyroscope": {
          "offset_x": 0.0,
          "offset_y": 0.0,
          "offset_z": 0.0
        }
      }
    },
    "qtr8a": {
      "threshold": 500,
      "emitter_on": true,
      "calibration": {
        "min_values": [0, 0, 0, 0, 0, 0, 0, 0],
        "max_values": [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
      }
    }
  }
}
```

#### PID Konfigürasyonu
- **Topic:** `config/pid/{device_id}`
- **QoS:** 1
- **Retain:** false
- **Format:** JSON

**Örnek Payload:**
```json
{
  "device": "arduino_mega",
  "pid": {
    "line_following": {
      "kp": 0.5,
      "ki": 0.0,
      "kd": 0.1,
      "setpoint": 3500,
      "output_limit": 255
    },
    "motor_control": {
      "kp": 1.0,
      "ki": 0.1,
      "kd": 0.05,
      "setpoint": 0,
      "output_limit": 255
    }
  }
}
```

### 📊 Veri Yayınlama Konuları

#### Sensör Verileri
- **Topic:** `sensors/{device_id}/{sensor_type}`
- **QoS:** 1
- **Retain:** false
- **Format:** JSON

**QTR8A Çizgi Sensörü:**
```json
{
  "device": "arduino_mega",
  "sensor": "qtr8a",
  "timestamp": 1640995200000,
  "values": [0, 0, 0, 1000, 1000, 0, 0, 0],
  "position": 3500,
  "calibrated": true,
  "line_detected": true
}
```

**GY-89 IMU Verileri:**
```json
{
  "device": "arduino_mega",
  "sensor": "gy89",
  "timestamp": 1640995200000,
  "accelerometer": {
    "x": 0.12,
    "y": -0.05,
    "z": 9.81
  },
  "gyroscope": {
    "x": 0.01,
    "y": 0.02,
    "z": -0.01
  },
  "magnetometer": {
    "x": 25.5,
    "y": -12.3,
    "z": 45.2
  },
  "temperature": 23.5,
  "pressure": 1013.25,
  "altitude": 100.5,
  "heading": 180.5
}
```

**Motor Durumu:**
```json
{
  "device": "arduino_mega",
  "timestamp": 1640995200000,
  "motors": {
    "left": {
      "speed": 150,
      "direction": "forward",
      "current": 0.5,
      "temperature": 35.2
    },
    "right": {
      "speed": 145,
      "direction": "forward",
      "current": 0.48,
      "temperature": 34.8
    }
  },
  "pid_output": 5.2
}
```

#### RFID Verileri
- **Topic:** `rfid/{device_id}/event`
- **QoS:** 1
- **Retain:** false
- **Format:** JSON

**Örnek Payload:**
```json
{
  "device": "deneyap_kart_1",
  "event": "card_detected",
  "uid": "04:A3:B2:C1:D0:E5:F6",
  "card_type": "MIFARE Classic",
  "timestamp": 1640995200000,
  "rssi": -45
}
```

### 🎛️ Komut Konuları

#### Motor Kontrolü
- **Topic:** `command/motors/{device_id}`
- **QoS:** 1
- **Retain:** false
- **Format:** JSON

**Örnek Payload:**
```json
{
  "device": "arduino_mega",
  "command": "set_speed",
  "left_speed": 200,
  "right_speed": 200,
  "left_direction": 0,
  "right_direction": 0,
  "duration": 5000
}
```

#### PID Ayarları
- **Topic:** `command/pid/{device_id}`
- **QoS:** 1
- **Retain:** false
- **Format:** JSON

**Örnek Payload:**
```json
{
  "device": "arduino_mega",
  "command": "update_pid",
  "pid_type": "line_following",
  "kp": 0.6,
  "ki": 0.0,
  "kd": 0.15,
  "setpoint": 3500,
  "output_limit": 255
}
```

#### Sistem Komutları
- **Topic:** `command/system/{device_id}`
- **QoS:** 1
- **Retain:** false
- **Format:** JSON

**Kalibrasyon Başlat:**
```json
{
  "device": "arduino_mega",
  "command": "calibrate_sensors",
  "sensors": ["qtr8a", "gy89"],
  "duration": 10000
}
```

**Konfigürasyon Kaydet:**
```json
{
  "device": "arduino_mega",
  "command": "save_config",
  "config_type": "all"
}
```

**Sistem Durumu:**
```json
{
  "device": "arduino_mega",
  "command": "get_status",
  "include": ["sensors", "motors", "pid", "i2c"]
}
```

### 📊 Durum Konuları

#### Sistem Durumu
- **Topic:** `system/{device_id}/status`
- **QoS:** 1
- **Retain:** true
- **Format:** JSON

**Örnek Payload:**
```json
{
  "device": "deneyap_kart_1",
  "status": "connected",
  "timestamp": 1640995200000,
  "wifi_rssi": -45,
  "uptime": 3600,
  "free_memory": 20480
}
```

## 🔧 Cihaz ID'leri

| Cihaz | Device ID | Açıklama |
|-------|-----------|----------|
| Arduino Mega | `arduino_mega` | Ana robotik kontrol |
| Deneyap Kart 1 | `deneyap_kart_1` | RFID erişim sistemi |
| Deneyap Kart 2 | `deneyap_kart_2` | I²C haberleşme köprüsü |

## 📡 Veri Formatları

### Timestamp Formatı
- **Format:** Unix timestamp (milisaniye)
- **Örnek:** `1640995200000`
- **Açıklama:** 1 Ocak 1970'den itibaren geçen milisaniye

### JSON Formatı
- **Encoding:** UTF-8
- **Indentation:** 2 spaces
- **Null Values:** `null` yerine alan atlanır
- **Boolean:** `true`/`false`

### Veri Tipleri
- **Integer:** Tam sayılar
- **Float:** Ondalıklı sayılar (32-bit)
- **String:** UTF-8 encoded strings
- **Boolean:** true/false
- **Array:** JSON array
- **Object:** JSON object

## 🔒 Güvenlik

### Bağlantı Güvenliği
- **TLS/SSL:** Gerekirse TLS 1.2 kullanılabilir
- **Authentication:** Username/password authentication
- **Authorization:** Topic-based access control

### Veri Güvenliği
- **Checksum:** I²C verilerinde checksum kullanımı
- **Validation:** JSON schema validation
- **Sanitization:** Input sanitization

## 📈 Performans

### Veri Aktarım Hızları
- **Sensör Verileri:** 100ms interval
- **Komut Yanıtı:** < 50ms
- **Konfigürasyon:** < 100ms
- **Durum Güncellemesi:** 5s interval

### Mesaj Boyutları
- **Sensör Verisi:** ~500 bytes
- **Komut:** ~200 bytes
- **Konfigürasyon:** ~1KB
- **Durum:** ~300 bytes

## 🔧 Hata Yönetimi

### Bağlantı Hataları
- **Otomatik Yeniden Bağlanma:** 5 saniye aralıklarla
- **Exponential Backoff:** Başarısız bağlantılar için
- **Keep Alive:** 60 saniye

### Veri Hataları
- **JSON Parse Hatası:** Log ve ignore
- **Missing Fields:** Default değerler kullan
- **Invalid Data:** Error response gönder

### Hata Yanıtları
```json
{
  "device": "arduino_mega",
  "error": "invalid_command",
  "message": "Unknown command type",
  "timestamp": 1640995200000
}
```

## 📄 Lisans

Bu protokol eğitim amaçlıdır. Endüstriyel uygulamalarda ek güvenlik önlemleri alınmalıdır. 