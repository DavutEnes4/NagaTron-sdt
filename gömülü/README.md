# NAGATRON GÖMÜLÜ SİSTEMLER

Bu proje, RFID ve QTR/PID sistemlerini içeren gömülü sistemler koleksiyonudur. Her iki sistem de MQTT ağına bağlanarak veri gönderir ve alır.

## 📋 SİSTEM GENEL BAKIŞ

            ### 🔐 RFID Sistemi
            - **Donanım**: Deneyap Kart 1 + RC522 Sensör
            - **İşlev**: RFID kartları okuma/yazma
            - **Bağlantı**: MQTT ağına bağlanır
            - **Özellikler**:
              - Kart okuma ve yazma
              - MQTT üzerinden uzaktan kontrol
              - Çoklu WiFi ağı desteği
              - Otomatik yeniden bağlanma

            ### 🤖 QTR/PID Sistemi
            - **Donanım**: Arduino Mega + QTR8A + BTS7960B Motor Sürücüleri + Deneyap Kart 2
            - **İşlev**: Çizgi takibi ve PID kontrolü
            - **Bağlantı**: I²C protokolü + MQTT ağı
            - **Özellikler**:
              - QTR8A sensörü ile çizgi takibi
              - PID kontrolü ile motor sürme
              - I²C ile Arduino Mega kontrolü
              - MQTT üzerinden uzaktan kontrol

## 🏗️ SİSTEM MİMARİSİ

            ```
            ┌─────────────────┐    MQTT    ┌─────────────────┐
            │ Deneyap Kart 1  │ ─────────→ │   MQTT Broker   │
            │  (RFID Master)  │            │                 │
            └─────────────────┘            └─────────────────┘
                     │                              │
                     │                              │
                     ▼                              │
            ┌─────────────────┐                    │
            │   RC522 Sensör  │                    │
            │                 │                    │
            └─────────────────┘                    │
                                                  │
            ┌─────────────────┐    I²C    ┌─────────────────┐    MQTT    │
            │ Deneyap Kart 2  │ ────────→ │  Arduino Mega   │ ─────────→ │
            │ (QTR/PID Master)│           │   (QTR Slave)   │            │
            └─────────────────┘           └─────────────────┘            │
                     │                              │                     │
                     │                              │                     │
                     ▼                              ▼                     │
            ┌─────────────────┐            ┌─────────────────┐           │
            │ Potansiyometre  │            │   QTR8A Sensör  │           │
            │   + Butonlar    │            │   + Motors      │           │
            └─────────────────┘            └─────────────────┘           │
                                                      │                  │
                                                      ▼                  │
                                            ┌─────────────────┐         │
                                            │   Web Arayüzü   │ ←────────┘
                                            │   / API         │
                                            └─────────────────┘
            ```

## 📁 DOSYA YAPISI

```
gömülü/
├── config.h                          # Ortak konfigürasyon dosyası
├── rfid_system/
│   └── rfid_system.ino              # RFID sistemi (Deneyap Kart)
├── qtr_pid_system/
│   ├── arduino_mega_slave.ino       # QTR/PID sistemi (Arduino Mega)
│   └── deneyap_master.ino           # QTR/PID kontrolü (Deneyap Kart)
├── API_DOCUMENTATION.md              # API dokümantasyonu
└── README.md                        # Bu dosya
```

## ⚙️ KURULUM TALİMATLARI

### 1. Gerekli Kütüphaneler

#### RFID Sistemi için:
```cpp
#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ArduinoJson.h>
```

#### QTR/PID Sistemi için:
```cpp
#include <Wire.h>
#include <QTRSensors.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
```

### 2. Donanım Bağlantıları

#### RFID Sistemi (Deneyap Kart):
- **RC522 RST**: D9
- **RC522 SS**: D10
- **RC522 MOSI**: D11
- **RC522 MISO**: D12
- **RC522 SCK**: D13

#### QTR/PID Sistemi (Arduino Mega):
- **QTR8A Sensörler**: A0-A7
- **BTS7960B Sağ Motor**:
  - RPWM: Pin 10
  - LPWM: Pin 9
  - R_EN: Pin 7
  - L_EN: Pin 8
- **BTS7960B Sol Motor**:
  - RPWM: Pin 6
  - LPWM: Pin 5
  - R_EN: Pin 3
  - L_EN: Pin 4

#### QTR/PID Kontrolü (Deneyap Kart):
- **Potansiyometreler**:
  - Hız: A0
  - Kp: A1
  - Kd: A2
- **Butonlar**:
  - START/STOP: Pin 2
  - RESET: Pin 3

### 3. Konfigürasyon

`config.h` dosyasında aşağıdaki ayarları yapabilirsiniz:

```cpp
// WiFi ağları
WiFiNetwork wifiNetworks[] = {
    {"WiFi_Adı", "Şifre", true},
    {"Yedek_WiFi", "Şifre", false}
};

// MQTT sunucuları
MQTTServer mqttServers[] = {
    {"broker.adresi", 1883, "kullanıcı", "şifre", "cihaz_id"}
};
```

## 🚀 ÇALIŞMA MANTIĞI

### RFID Sistemi Çalışma Mantığı:

1. **Başlatma**: Sistem 3 saniye bekler
2. **WiFi Bağlantısı**: Aktif ağlara sırayla bağlanmaya çalışır
3. **MQTT Bağlantısı**: MQTT broker'a bağlanır
4. **Kart Okuma**: RC522 sensörü ile kartları okur
5. **Veri Gönderme**: Okunan kart bilgilerini MQTT'ye gönderir
6. **Yazma Modu**: MQTT'den gelen komutlarla karta veri yazar

### QTR/PID Sistemi Çalışma Mantığı:

1. **Başlatma**: Sistem 3 saniye bekler
2. **Kalibrasyon**: QTR sensörleri kalibre edilir
3. **I²C Bağlantısı**: Arduino Mega slave olarak çalışır
4. **PID Kontrolü**: QTR sensör verilerine göre PID hesaplar
5. **Motor Kontrolü**: BTS7960B sürücüleri ile motorları sürer
6. **MQTT İletişimi**: Deneyap kart master olarak MQTT'ye bağlanır

## 📡 MQTT TOPIC'LERİ

### RFID Sistemi:
- `rfid/data` - Kart okuma verileri
- `rfid/write` - Kart yazma komutları
- `rfid/status` - Yazma durumu

### QTR/PID Sistemi:
- `qtr/data` - QTR sensör verileri
- `motor/control` - Motor kontrol komutları
- `pid/config` - PID parametreleri
- `system/status` - Sistem durumu

## 🔧 API KULLANIMI

### RFID Kartına Yazma:
```bash
curl -X POST http://localhost:5000/api/write-rfid \
  -H "Content-Type: application/json" \
  -d '{"card_id": "1234567890", "data": "Test verisi"}'
```

### Motor Kontrolü:
```bash
curl -X POST http://localhost:5000/api/motor-control \
  -H "Content-Type: application/json" \
  -d '{"action": "start", "speed": 150}'
```

### PID Ayarları:
```bash
curl -X POST http://localhost:5000/api/update-pid \
  -H "Content-Type: application/json" \
  -d '{"kp": 2.5, "ki": 0.2, "kd": 0.8}'
```

## 🛠️ SORUN GİDERME

### RFID Sistemi Sorunları:
1. **Kart okunmuyor**: RC522 pinlerini kontrol edin
2. **MQTT bağlantı hatası**: WiFi ve MQTT ayarlarını kontrol edin
3. **Yazma başarısız**: Kartın yazılabilir olduğundan emin olun

### QTR/PID Sistemi Sorunları:
1. **I²C bağlantı hatası**: SDA/SCL pinlerini kontrol edin
2. **Motor çalışmıyor**: BTS7960B bağlantılarını kontrol edin
3. **PID ayarları**: Kp, Ki, Kd değerlerini optimize edin

## 📊 SİSTEM DURUMU

### RFID Sistemi Durumu:
- WiFi bağlantısı
- MQTT bağlantısı
- Toplam okuma/yazma sayısı
- Son okunan kart ID'si

### QTR/PID Sistemi Durumu:
- I²C bağlantısı
- QTR sensör pozisyonu
- Motor hızları
- PID parametreleri
- Sistem aktif/pasif durumu

## 🔄 OTOMATİK YENİDEN BAĞLANMA

Her iki sistem de:
- WiFi bağlantısı kesildiğinde otomatik yeniden bağlanır
- MQTT bağlantısı kesildiğinde otomatik yeniden bağlanır
- I²C bağlantısı kesildiğinde timeout ile durdurulur

## 📝 NOTLAR

- Sistem varsayılan olarak 3 saniye bekler ve çalışmaya başlar
- Tüm sistemler ortak `config.h` dosyasını kullanır
- MQTT topic'leri standartlaştırılmıştır
- Hata durumlarında sistem otomatik olarak kendini korur
- Debug modu aktif olarak çalışır

## 🤝 KATKIDA BULUNMA

1. Fork yapın
2. Feature branch oluşturun (`git checkout -b feature/yeni-ozellik`)
3. Commit yapın (`git commit -am 'Yeni özellik eklendi'`)
4. Push yapın (`git push origin feature/yeni-ozellik`)
5. Pull Request oluşturun

## 📄 LİSANS

Bu proje MIT lisansı altında lisanslanmıştır. 