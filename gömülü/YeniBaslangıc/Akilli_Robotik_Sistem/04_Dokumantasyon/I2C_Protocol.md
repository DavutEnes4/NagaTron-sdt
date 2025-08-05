# 🔌 I²C Haberleşme Protokolü

## 📋 Genel Bakış

Bu dokümantasyon, Arduino Mega ile Deneyap Kart 2 arasındaki I²C haberleşme protokolünü detaylandırır. I²C protokolü, sensör verilerinin aktarımı ve konfigürasyon komutlarının iletilmesi için kullanılır.

## 🔧 I²C Ayarları

### Temel Parametreler
- **Hız:** 400kHz (Fast Mode)
- **Adres Genişliği:** 7-bit
- **Pull-up Dirençleri:** 4.7kΩ (her hatta)
- **Voltaj Seviyesi:** 3.3V

### I²C Adresleri
| Cihaz | Adres | Açıklama |
|-------|-------|----------|
| GY-89 (MPU6050) | 0x68 | IMU - İvme ve Jiroskop |
| GY-89 (HMC5883L) | 0x1E | Magnetometre |
| GY-89 (BMP180) | 0x77 | Basınç ve Sıcaklık |
| Deneyap Kart 2 | 0x10 | ESP32 I²C Slave |

## 📡 Veri Formatları

### Arduino Mega → Deneyap Kart 2

#### Sensör Verileri Paketi
```c
struct SensorData {
  uint8_t header[2];        // 0xAA, 0x55
  uint8_t device_id;        // 0x01 = Arduino Mega
  uint8_t data_type;        // 0x01 = IMU, 0x02 = QTR8A, 0x03 = Motor
  uint32_t timestamp;       // 4 bytes
  uint8_t data_length;      // 1 byte
  uint8_t data[64];         // Variable length
  uint8_t checksum;         // 1 byte
};
```

#### IMU Veri Formatı
```c
struct IMUData {
  float accel_x, accel_y, accel_z;  // 12 bytes (3 * 4 bytes)
  float gyro_x, gyro_y, gyro_z;     // 12 bytes (3 * 4 bytes)
  float mag_x, mag_y, mag_z;        // 12 bytes (3 * 4 bytes)
  float temperature;                 // 4 bytes
  float pressure;                    // 4 bytes
  float altitude;                    // 4 bytes
  float heading;                     // 4 bytes
}; // Total: 52 bytes
```

#### QTR8A Veri Formatı
```c
struct QTR8AData {
  uint16_t sensor_values[8];        // 16 bytes (8 * 2 bytes)
  uint16_t position;                // 2 bytes
  uint8_t calibrated;               // 1 byte
  uint8_t line_detected;            // 1 byte
}; // Total: 20 bytes
```

#### Motor Durum Veri Formatı
```c
struct MotorData {
  uint8_t left_speed;               // 1 byte
  uint8_t right_speed;              // 1 byte
  uint8_t left_direction;           // 1 byte
  uint8_t right_direction;          // 1 byte
  float left_current;               // 4 bytes
  float right_current;              // 4 bytes
  float left_temperature;           // 4 bytes
  float right_temperature;          // 4 bytes
}; // Total: 20 bytes
```

### Deneyap Kart 2 → Arduino Mega

#### Konfigürasyon Komutları
```c
struct ConfigCommand {
  uint8_t header[2];        // 0xBB, 0x66
  uint8_t command_type;     // 0x01 = Pin Config, 0x02 = PID Config, 0x03 = Motor
  uint8_t data_length;      // 1 byte
  uint8_t data[32];         // Variable length
  uint8_t checksum;         // 1 byte
};
```

#### Pin Konfigürasyonu
```c
struct PinConfig {
  uint8_t qtr_pins[8];             // 8 bytes
  uint8_t qtr_emitter_pin;         // 1 byte
  uint8_t motor_left_pwm;          // 1 byte
  uint8_t motor_left_dir;          // 1 byte
  uint8_t motor_left_brake;        // 1 byte
  uint8_t motor_right_pwm;         // 1 byte
  uint8_t motor_right_dir;         // 1 byte
  uint8_t motor_right_brake;       // 1 byte
  uint8_t i2c_sda;                 // 1 byte
  uint8_t i2c_scl;                 // 1 byte
}; // Total: 16 bytes
```

#### PID Konfigürasyonu
```c
struct PIDConfig {
  float kp;                         // 4 bytes
  float ki;                         // 4 bytes
  float kd;                         // 4 bytes
  float setpoint;                   // 4 bytes
  float output_limit;               // 4 bytes
}; // Total: 20 bytes
```

## 🔧 Checksum Hesaplama

### Algoritma
```cpp
uint8_t calculateChecksum(uint8_t* data, uint8_t length) {
  uint8_t checksum = 0;
  for (int i = 0; i < length; i++) {
    checksum ^= data[i];
  }
  return checksum;
}
```

### Örnek
```cpp
// Header: 0xAA, 0x55
// Device ID: 0x01
// Data Type: 0x01 (IMU)
// Checksum = 0xAA ^ 0x55 ^ 0x01 ^ 0x01 = 0xFF
```

## 📡 Veri Aktarım Protokolü

### Arduino Mega → Deneyap Kart 2

#### Adım 1: Başlatma
```cpp
Wire.beginTransmission(0x10); // Deneyap Kart 2 adresi
```

#### Adım 2: Header Gönderme
```cpp
Wire.write(0xAA); // Header byte 1
Wire.write(0x55); // Header byte 2
```

#### Adım 3: Meta Veri Gönderme
```cpp
Wire.write(0x01); // Device ID (Arduino Mega)
Wire.write(data_type); // 0x01=IMU, 0x02=QTR8A, 0x03=Motor
```

#### Adım 4: Timestamp Gönderme
```cpp
uint32_t timestamp = millis();
Wire.write((uint8_t*)&timestamp, 4);
```

#### Adım 5: Veri Uzunluğu ve Veri Gönderme
```cpp
Wire.write(data_length);
Wire.write(data, data_length);
```

#### Adım 6: Checksum Gönderme
```cpp
uint8_t checksum = calculateChecksum(data, data_length);
Wire.write(checksum);
```

#### Adım 7: İletimi Sonlandırma
```cpp
Wire.endTransmission();
```

### Deneyap Kart 2 → Arduino Mega

#### Adım 1: Başlatma
```cpp
Wire.beginTransmission(0x01); // Arduino Mega adresi
```

#### Adım 2: Header Gönderme
```cpp
Wire.write(0xBB); // Header byte 1
Wire.write(0x66); // Header byte 2
```

#### Adım 3: Komut Tipi ve Veri Uzunluğu
```cpp
Wire.write(command_type); // 0x01=Pin, 0x02=PID, 0x03=Motor
Wire.write(data_length);
```

#### Adım 4: Veri Gönderme
```cpp
Wire.write(data, data_length);
```

#### Adım 5: İletimi Sonlandırma
```cpp
Wire.endTransmission();
```

## 🔧 Veri İşleme

### Arduino Mega (Master) Fonksiyonları

#### I²C Master Başlatma
```cpp
void i2c_master_init() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz
}
```

#### Sensör Verilerini Gönderme
```cpp
void send_sensor_data_to_esp32(uint8_t data_type, uint8_t* data, uint8_t length) {
  Wire.beginTransmission(0x10);
  Wire.write(0xAA); // Header
  Wire.write(0x55);
  Wire.write(0x01); // Device ID
  Wire.write(data_type);
  Wire.write((uint8_t*)&millis(), 4); // Timestamp
  Wire.write(length);
  Wire.write(data, length);
  
  // Checksum hesapla
  uint8_t checksum = 0xAA ^ 0x55 ^ 0x01 ^ data_type;
  for(int i = 0; i < length; i++) {
    checksum ^= data[i];
  }
  Wire.write(checksum);
  
  Wire.endTransmission();
}
```

#### Konfigürasyon Komutlarını Alma
```cpp
void receive_config_from_esp32() {
  if(Wire.available() >= 4) {
    uint8_t header1 = Wire.read();
    uint8_t header2 = Wire.read();
    
    if(header1 == 0xBB && header2 == 0x66) {
      uint8_t command_type = Wire.read();
      uint8_t data_length = Wire.read();
      
      uint8_t data[32];
      for(int i = 0; i < data_length; i++) {
        data[i] = Wire.read();
      }
      
      process_config_command(command_type, data, data_length);
    }
  }
}
```

### Deneyap Kart 2 (Slave) Fonksiyonları

#### I²C Slave Başlatma
```cpp
void i2c_slave_init() {
  Wire.begin(0x10); // Slave adresi
  Wire.onReceive(receive_data_from_arduino);
  Wire.onRequest(send_data_to_arduino);
}
```

#### Arduino'dan Gelen Verileri Alma
```cpp
void receive_data_from_arduino(int byteCount) {
  if(byteCount >= 4) {
    uint8_t header1 = Wire.read();
    uint8_t header2 = Wire.read();
    
    if(header1 == 0xAA && header2 == 0x55) {
      uint8_t device_id = Wire.read();
      uint8_t data_type = Wire.read();
      
      // Veriyi MQTT'e gönder
      send_sensor_data_to_mqtt(data_type);
    }
  }
}
```

#### Arduino'ya Konfigürasyon Gönderme
```cpp
void send_config_to_arduino(uint8_t command_type, uint8_t* data, uint8_t length) {
  Wire.beginTransmission(0x01); // Arduino Mega adresi
  Wire.write(0xBB); // Header
  Wire.write(0x66);
  Wire.write(command_type);
  Wire.write(length);
  Wire.write(data, length);
  Wire.endTransmission();
}
```

## 📈 Performans

### Veri Aktarım Hızları
- **I²C Hızı:** 400kHz
- **Sensör Verisi:** 50ms interval
- **Komut Yanıtı:** < 10ms
- **Konfigürasyon:** < 20ms

### Mesaj Boyutları
- **IMU Verisi:** 52 bytes
- **QTR8A Verisi:** 20 bytes
- **Motor Verisi:** 20 bytes
- **Pin Konfigürasyonu:** 16 bytes
- **PID Konfigürasyonu:** 20 bytes

## 🔧 Hata Yönetimi

### Bağlantı Hataları
- **I²C Busy:** Retry mechanism
- **Address NACK:** Device not found
- **Data NACK:** Transmission error
- **Timeout:** 100ms timeout

### Veri Hataları
- **Checksum Mismatch:** Data corruption
- **Invalid Header:** Protocol error
- **Length Mismatch:** Data length error
- **Invalid Data Type:** Unknown command

### Hata Yanıtları
```cpp
struct ErrorResponse {
  uint8_t header[2];        // 0xCC, 0xDD
  uint8_t error_code;       // 0x01=Checksum, 0x02=Length, 0x03=Type
  uint8_t original_command; // Original command that failed
  uint8_t checksum;
};
```

## 🔒 Güvenlik

### Veri Bütünlüğü
- **Checksum:** XOR-based checksum
- **Header Validation:** Fixed header values
- **Length Validation:** Expected data length
- **Type Validation:** Valid command types

### Hata Toleransı
- **Retry Mechanism:** 3 attempts on failure
- **Timeout Handling:** 100ms timeout
- **Error Logging:** Error code logging
- **Recovery:** Automatic recovery from errors

## 📄 Lisans

Bu protokol eğitim amaçlıdır. Endüstriyel uygulamalarda ek güvenlik önlemleri alınmalıdır. 