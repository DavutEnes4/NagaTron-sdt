# 🤖 Akıllı Robotik İzleme ve Erişim Sistemi

## 📋 Proje Genel Bakış

Bu proje, RFID erişim kontrolü, PID tabanlı çizgi takibi, 9-eksenli IMU sensörleri ve RadioLink R12DS v1.1 uzaktan kumanda desteği ile çalışan kapsamlı bir robotik sistemdir. Sistem, Arduino Mega 2560, Deneyap Kart 1 (ESP32) ve Deneyap Kart 2 (ESP32) arasında I²C ve MQTT protokolleri ile haberleşir.

## 🏗️ Sistem Mimarisi

```
┌─────────────────┐    I²C    ┌─────────────────┐    WiFi    ┌─────────────────┐
│   Arduino Mega  │◄─────────►│  Deneyap Kart 2 │◄─────────►│   MQTT Broker   │
│   (Kontrolcü)   │            │   (I²C Bridge)  │            │                 │
└─────────────────┘            └─────────────────┘            └─────────────────┘
         │                               │                              │
         │                               │                              │
         ▼                               ▼                              ▼
┌─────────────────┐            ┌─────────────────┐            ┌─────────────────┐
│   QTR-8A       │            │   WiFi          │            │   Deneyap Kart 1│
│   GY-89 IMU    │            │   MQTT Client   │            │   (RFID Modül)  │
│   BTS7960B     │            │   JSON Parser   │            │                 │
│   RadioLink     │            │                 │            │                 │
│   R12DS v1.1   │            │                 │            │                 │
└─────────────────┘            └─────────────────┘            └─────────────────┘
```

## 🛠️ Sistem Bileşenleri

### Modül 1: RFID Erişim ve Kontrol Sistemi
- **Donanım**: Deneyap Kart 1 (ESP32) + RC522 RFID Sensörü
- **Fonksiyonlar**: RFID kart okuma/yazma, MQTT veri yayınlama
- **Protokoller**: SPI (RC522), WiFi, MQTT
- **Özellikler**: Ortak konfigürasyon sistemi

### Modül 2: Robotik Kontrol Sistemi
- **Donanım**: Arduino Mega 2560 + QTR-8A + GY-89 + BTS7960B + RadioLink R12DS v1.1
- **Fonksiyonlar**: 
  - PID tabanlı çizgi takibi
  - 9-eksenli IMU veri işleme
  - RadioLink R12DS v1.1 uzaktan kumanda
  - QTR kalibrasyonu sırasında araç hareketi
- **Kontrol Modları**:
  - **Otonom Mod**: PID tabanlı çizgi takibi
  - **Manuel Mod**: RadioLink kumanda ile kontrol
  - **MQTT Mod**: Uzaktan komut kontrolü
  - **Kalibrasyon Mod**: QTR sensörü kalibrasyonu
- **Özellikler**: Ortak konfigürasyon sistemi

### Modül 3: I²C Haberleşme Köprüsü
- **Donanım**: Deneyap Kart 2 (ESP32)
- **Fonksiyonlar**: Arduino Mega ile MQTT arasında veri aktarımı
- **Protokoller**: I²C (Slave), WiFi, MQTT
- **Özellikler**: Ortak konfigürasyon sistemi

### 🔧 Ortak Konfigürasyon Sistemi
- **Merkezi Yönetim**: Tüm ayarlar tek dosyada (`Shared_Config.h`)
- **Kategoriler**: WiFi, MQTT, pin konfigürasyonları, PID parametreleri
- **Avantajlar**: Tek noktadan yönetim, tutarlılık, kolay bakım
- **Dokümantasyon**: `04_Dokumantasyon/Shared_Config_README.md`

## 🔧 Kontrol Modları

### 1. Otonom Mod (MODE_AUTONOMOUS)
- PID tabanlı çizgi takibi
- QTR-8A sensörü ile çizgi algılama
- Otomatik hız ve yön kontrolü
- GY-89 IMU ile hareket algılama

### 2. Manuel Mod (MODE_MANUAL)
- RadioLink R12DS v1.1 ile manuel kontrol
- AUX1 kanalı ile mod değiştirme
- Throttle (CH1) ve Rudder (CH4) ile hareket
- Gerçek zamanlı kumanda tepkisi

### 3. MQTT Mod (MODE_MQTT)
- MQTT komutları ile uzaktan kontrol
- I²C üzerinden Deneyap Kart 2'den komut alma
- JSON formatında komut işleme

### 4. Kalibrasyon Mod (MODE_CALIBRATION)
- QTR sensörü kalibrasyonu sırasında araç hareketi
- Sağa-sola hareket ile sensör kalibrasyonu
- Otomatik kalibrasyon süreci

## 📡 MQTT Haberleşme

### Yayınlanan Konular (Published Topics)

#### Sensör Verileri
| Konu | Açıklama | Veri Formatı |
|------|----------|--------------|
| `sensors/arduino_mega/gy89` | IMU verileri | JSON |
| `sensors/arduino_mega/qtr8a` | QTR sensör verileri | JSON |
| `sensors/arduino_mega/motors` | Motor durumu | JSON |
| `sensors/arduino_mega/radiolink` | RadioLink verileri | JSON |
| `rfid/reader/uid` | RFID kart verileri | JSON |
| `rfid/status` | RFID sistem durumu | JSON |

#### Sistem Durumu
| Konu | Açıklama | Veri Formatı |
|------|----------|--------------|
| `system/deneyap_kart_1/status` | RFID modül durumu | JSON |
| `system/deneyap_kart_2/status` | I²C Bridge durumu | JSON |

### Abone Olunan Konular (Subscribed Topics)

#### Komut Konuları
| Konu | Açıklama | Veri Formatı |
|------|----------|--------------|
| `command/motors/arduino_mega` | Motor komutları | JSON |
| `config/pins/arduino_mega` | Pin konfigürasyonu | JSON |
| `config/pid/arduino_mega` | PID konfigürasyonu | JSON |
| `rfid/command/write` | RFID yazma komutları | JSON |

## 🔌 I²C Haberleşme

### Arduino Mega → Deneyap Kart 2
- **Veri Tipleri**: IMU, QTR8A, Motor, RadioLink
- **Format**: Header + Device ID + Data Type + Timestamp + Data
- **Hız**: 400kHz
- **Frekans**: 20Hz

### Deneyap Kart 2 → Arduino Mega
- **Komut Tipleri**: Pin Config, PID Config, Motor Command
- **Format**: Header + Command Type + Data Length + Data
- **Güvenlik**: Checksum doğrulama

## 🚀 Özellikler

### ✅ Mevcut Özellikler
- **RFID Erişim Kontrolü**: Kart okuma/yazma ve MQTT yayınlama
- **PID Tabanlı Çizgi Takibi**: Hassas çizgi takibi algoritması
- **9-Eksenli IMU**: Tam hareket algılama ve pusula
- **RadioLink R12DS v1.1 Desteği**: Uzaktan kumanda kontrolü
- **QTR Kalibrasyon Hareketi**: Otomatik sağa-sola hareket ile kalibrasyon
- **Çoklu Kontrol Modu**: Otonom, Manuel, MQTT, Kalibrasyon
- **I²C Haberleşme**: Arduino Mega ile Deneyap Kart 2 arasında
- **MQTT Veri Aktarımı**: Gerçek zamanlı sensör verileri
- **EEPROM Konfigürasyonu**: Kalıcı ayar saklama
- **Çoklu WiFi Desteği**: Otomatik ağ değiştirme

### 🔧 Yeni Özellikler
- **RadioLink R12DS v1.1 Entegrasyonu**: 8 kanallı uzaktan kumanda
- **Kontrol Modu Geçişi**: AUX1 kanalı ile mod değiştirme
- **QTR Kalibrasyon Hareketi**: Araç sağa-sola hareket ederek kalibrasyon
- **Gerçek Zamanlı Kumanda Tepkisi**: 50Hz kumanda okuma
- **Bağlantı Durumu İzleme**: RadioLink sinyal durumu takibi

## 📁 Klasör Yapısı

```
Akilli_Robotik_Sistem/
├── README.md                           # Ana proje dokümantasyonu
├── 01_RFID_Modul/                     # RFID Erişim Sistemi
│   ├── RFID_Access.ino                # Arduino kodu
│   └── README.md                      # Modül dokümantasyonu
├── 02_Arduino_Mega/                   # Robotik Kontrol Sistemi
│   ├── Robot_Control.ino              # Arduino kodu
│   └── README.md                      # Modül dokümantasyonu
├── 03_Deneyap_Kart2/                  # I²C Haberleşme Köprüsü
│   ├── I2C_Bridge.ino                 # Arduino kodu
│   └── README.md                      # Modül dokümantasyonu
└── 04_Dokumantasyon/                  # Detaylı Protokol Dokümantasyonu
    ├── MQTT_Protocol.md               # MQTT protokol detayları
    ├── I2C_Protocol.md                # I²C protokol detayları
    └── Pin_Configurations.md          # Pin konfigürasyonları
```

## ⚙️ Sistem Gereksinimleri

### Donanım Gereksinimleri
- **Arduino Mega 2560**: Ana kontrolcü
- **Deneyap Kart 1 (ESP32)**: RFID modülü
- **Deneyap Kart 2 (ESP32)**: I²C Bridge
- **QTR-8A**: Analog çizgi sensörü
- **GY-89**: 9-eksenli IMU modülü
- **BTS7960B**: Motor sürücüleri (x2)
- **RC522**: RFID sensörü
- **RadioLink R12DS v1.1**: Uzaktan kumanda ve alıcı

### Yazılım Gereksinimleri
- **Arduino IDE**: Geliştirme ortamı
- **MQTT Broker**: Veri aktarımı için
- **WiFi Ağı**: İnternet bağlantısı
- **Gerekli Kütüphaneler**:
  - `QTRSensors`
  - `PID_v1`
  - `MPU6050`
  - `HMC5883L`
  - `BMP180`
  - `MFRC522`
  - `WiFiMulti`
  - `PubSubClient`
  - `ArduinoJson`

## 🚀 Kurulum ve Çalıştırma

### 1. Donanım Kurulumu
1. **Arduino Mega Bağlantıları**:
   - QTR-8A sensörünü analog pinlere bağlayın
   - GY-89 IMU modülünü I²C bağlayın
   - BTS7960B motor sürücülerini PWM pinlere bağlayın
   - RadioLink R12DS v1.1 alıcısını dijital pinlere bağlayın

2. **Deneyap Kart 1 Bağlantıları**:
   - RC522 RFID sensörünü SPI pinlerine bağlayın

3. **Deneyap Kart 2 Bağlantıları**:
   - Arduino Mega ile I²C bağlantısını yapın

### 2. Yazılım Kurulumu
1. **Kütüphaneleri Yükleyin**:
   ```cpp
   // Arduino IDE'de Library Manager'dan yükleyin
   QTRSensors
   PID_v1
   MPU6050
   HMC5883L
   BMP180
   MFRC522
   WiFiMulti
   PubSubClient
   ArduinoJson
   ```

2. **Konfigürasyon Ayarları**:
   - WiFi bilgilerini güncelleyin
   - MQTT broker ayarlarını yapın
   - Pin konfigürasyonlarını kontrol edin

### 3. Sistem Başlatma
1. **Arduino Mega**: Robot_Control.ino'yu yükleyin
2. **Deneyap Kart 1**: RFID_Access.ino'yu yükleyin
3. **Deneyap Kart 2**: I2C_Bridge.ino'yu yükleyin
4. **MQTT Broker**: Bağlantıyı test edin

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

### RFID Testi
1. RFID kartını sensöre yaklaştırın
2. MQTT'de `rfid/reader/uid` konusunda veri yayınlandığını kontrol edin
3. Kart yazma işlemini test edin

### I²C Haberleşme Testi
1. Deneyap Kart 2 ile bağlantıyı kontrol edin
2. MQTT broker'a bağlanın
3. Sensör verilerinin yayınlandığını doğrulayın

## 📈 Performans

### Zamanlama
- **Sensör Okuma**: 100ms (10Hz)
- **I²C Veri Aktarımı**: 50ms (20Hz)
- **MQTT Yayınlama**: 100ms (10Hz)
- **RadioLink Okuma**: 20ms (50Hz)
- **PID Hesaplama**: 10ms (100Hz)

### Bellek Kullanımı
- **Arduino Mega**: ~15KB program, ~2KB RAM
- **Deneyap Kart 1**: ~8KB program, ~1KB RAM
- **Deneyap Kart 2**: ~8KB program, ~1.5KB RAM

## 🔒 Güvenlik

### Motor Güvenliği
- Akım sınırlama
- Sıcaklık izleme
- Acil durdurma fonksiyonu

### RadioLink Güvenliği
- Sinyal kaybı algılama
- Otomatik güvenli mod
- Bağlantı durumu izleme

### Veri Güvenliği
- I²C checksum kontrolü
- JSON format doğrulama
- MQTT kullanıcı doğrulama

## 📝 Lisans

Bu proje eğitim amaçlıdır. Donanım yapılandırmaları açık kaynaklıdır ve geliştirilebilir.

## 🎯 Proje Hedefleri

### ✅ Tamamlanan Hedefler
- [x] RFID erişim kontrolü
- [x] PID tabanlı çizgi takibi
- [x] 9-eksenli IMU entegrasyonu
- [x] I²C haberleşme köprüsü
- [x] MQTT veri aktarımı
- [x] RadioLink R12DS v1.1 desteği
- [x] QTR kalibrasyon hareketi
- [x] Çoklu kontrol modu
- [x] EEPROM konfigürasyonu

### 🔄 Gelecek Hedefler
- [ ] Web arayüzü geliştirme
- [ ] Mobil uygulama entegrasyonu
- [ ] Yapay zeka tabanlı çizgi takibi
- [ ] GPS entegrasyonu
- [ ] Kamera modülü ekleme
- [ ] Ses komut sistemi

---

**Son Güncelleme**: RadioLink R12DS v1.1 desteği ve QTR kalibrasyon hareketi eklendi. 