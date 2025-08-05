# 🔧 Ortak Konfigürasyon Sistemi

Bu sistem sayesinde tüm modüllerdeki ayarları tek bir yerden yönetebilirsiniz. Bir yerde yaptığınız değişiklik tüm modüllerde geçerli olur.

## 📋 İçerik

- [Sistem Avantajları](#sistem-avantajları)
- [Kullanım Talimatları](#kullanım-talimatları)
- [Konfigürasyon Kategorileri](#konfigürasyon-kategorileri)
- [Örnek Kullanımlar](#örnek-kullanımlar)
- [Değişiklik Yapma](#değişiklik-yapma)
- [Sık Sorulan Sorular](#sık-sorulan-sorular)

## 🎯 Sistem Avantajları

### ✅ Tek Noktadan Yönetim
- Tüm WiFi ayarları tek yerde
- MQTT broker bilgileri merkezi
- Pin konfigürasyonları ortak
- PID parametreleri standart

### ✅ Tutarlılık
- Tüm modüllerde aynı ayarlar
- Kod tekrarı yok
- Hata riski minimize

### ✅ Kolay Bakım
- Değişiklik tek dosyada
- Otomatik güncelleme
- Versiyon kontrolü kolay

## 📝 Kullanım Talimatları

### 1. Dosyayı Dahil Etme

Her `.ino` dosyasının başına ekleyin:

```cpp
#include "../04_Dokumantasyon/Shared_Config.h"
```

### 2. Eski Tanımlamaları Kaldırma

❌ **Eski Yöntem:**
```cpp
const char* ssid1 = "WiFi_SSID_1";
const char* mqtt_server = "broker.mqtt.com";
#define RFID_SDA_PIN 5
```

✅ **Yeni Yöntem:**
```cpp
// Shared_Config.h otomatik olarak tanımlar
wifiMulti.addAP(WIFI_SSID_1, WIFI_PASSWORD_1);
client.setServer(MQTT_SERVER, MQTT_PORT);
mfrc522.PCD_Init(RFID_SDA_PIN, RFID_RST_PIN);
```

### 3. Değişiklik Yapma

Sadece `Shared_Config.h` dosyasını düzenleyin:

```cpp
// WiFi ayarlarını değiştir
#define WIFI_SSID_1 "Yeni_Ağ_Adı"
#define WIFI_PASSWORD_1 "Yeni_Şifre"

// MQTT broker değiştir
#define MQTT_SERVER "yeni.broker.com"

// Pin değiştir
#define RFID_SDA_PIN 4
```

## 📂 Konfigürasyon Kategorileri

### 🌐 WiFi Ayarları
```cpp
#define WIFI_SSID_1 "WiFi_SSID_1"
#define WIFI_PASSWORD_1 "WiFi_Password_1"
#define WIFI_SSID_2 "WiFi_SSID_2"
#define WIFI_PASSWORD_2 "WiFi_Password_2"
```

### 📡 MQTT Ayarları
```cpp
#define MQTT_SERVER "broker.mqtt.com"
#define MQTT_PORT 1883
#define MQTT_USERNAME "your_username"
#define MQTT_PASSWORD "your_password"
```

### 🏷️ Cihaz Kimlikleri
```cpp
#define DEVICE_ID_RFID "deneyap_kart_1"
#define DEVICE_ID_BRIDGE "deneyap_kart_2"
#define DEVICE_ID_ARDUINO "arduino_mega"
```

### 📍 Pin Konfigürasyonları
```cpp
// RFID Pins
#define RFID_SDA_PIN 5
#define RFID_RST_PIN 22

// Motor Pins
#define MOTOR_LEFT_RPWM_PIN 3
#define MOTOR_RIGHT_RPWM_PIN 7

// RadioLink Pins
#define RADIOLINK_CH1_PIN 22
```

### ⚙️ PID Parametreleri
```cpp
#define PID_KP_DEFAULT 2.0
#define PID_KI_DEFAULT 0.1
#define PID_KD_DEFAULT 0.5
#define PID_SETPOINT_DEFAULT 3500
```

### 📋 MQTT Konuları
```cpp
#define MQTT_TOPIC_RFID_UID "rfid/reader/uid"
#define MQTT_TOPIC_IMU "sensors/arduino_mega/gy89"
#define MQTT_TOPIC_COMMAND "command/motors/arduino_mega"
```

## 💡 Örnek Kullanımlar

### WiFi Bağlantısı
```cpp
void setupWiFi() {
  wifiMulti.addAP(WIFI_SSID_1, WIFI_PASSWORD_1);
  wifiMulti.addAP(WIFI_SSID_2, WIFI_PASSWORD_2);
}
```

### MQTT Bağlantısı
```cpp
void setupMQTT() {
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.connect(DEVICE_ID_RFID, MQTT_USERNAME, MQTT_PASSWORD);
}
```

### Pin Kullanımı
```cpp
// RFID
MFRC522 mfrc522(RFID_SDA_PIN, RFID_RST_PIN);

// Motor
analogWrite(MOTOR_LEFT_RPWM_PIN, speed);

// RadioLink
uint16_t throttle = pulseIn(RADIOLINK_CH1_PIN, HIGH, 25000);
```

### PID Kontrolü
```cpp
PID linePID(&input, &output, &setpoint, 
            PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT, DIRECT);
```

## 🔄 Değişiklik Yapma

### 1. WiFi Ağı Değiştirme
```cpp
// Shared_Config.h dosyasında
#define WIFI_SSID_1 "Yeni_Ağ_Adı"
#define WIFI_PASSWORD_1 "Yeni_Şifre"
```

### 2. MQTT Broker Değiştirme
```cpp
#define MQTT_SERVER "yeni.broker.com"
#define MQTT_PORT 8883  // SSL port
```

### 3. Pin Değiştirme
```cpp
#define RFID_SDA_PIN 4   // 5'ten 4'e değiştir
#define MOTOR_LEFT_RPWM_PIN 5  // 3'ten 5'e değiştir
```

### 4. PID Parametreleri
```cpp
#define PID_KP_DEFAULT 3.0  // 2.0'dan 3.0'a değiştir
#define PID_KI_DEFAULT 0.2  // 0.1'den 0.2'ye değiştir
```

## ❓ Sık Sorulan Sorular

### Q: Yeni bir pin eklemek istiyorum, nasıl yaparım?
A: `Shared_Config.h` dosyasında yeni bir `#define` ekleyin:
```cpp
#define YENI_SENSOR_PIN 13
```

### Q: MQTT konularını değiştirmek istiyorum?
A: `MQTT_TOPIC_*` tanımlarını güncelleyin:
```cpp
#define MQTT_TOPIC_RFID_UID "yeni/konu/adi"
```

### Q: PID parametrelerini dinamik olarak değiştirebilir miyim?
A: Evet, MQTT üzerinden dinamik olarak değiştirebilirsiniz. Varsayılan değerler `Shared_Config.h`'den alınır.

### Q: Farklı modüller için farklı ayarlar gerekirse?
A: Modül-spesifik tanımlar ekleyebilirsiniz:
```cpp
#define RFID_SPECIFIC_PIN 15
#define ARDUINO_SPECIFIC_PIN 16
```

## 🚀 Gelecek Özellikler

- [ ] Web arayüzü ile konfigürasyon
- [ ] OTA (Over-The-Air) güncelleme
- [ ] Konfigürasyon yedekleme/geri yükleme
- [ ] Otomatik konfigürasyon doğrulama
- [ ] Çoklu ortam desteği (dev, test, prod)

## 📞 Destek

Herhangi bir sorun yaşarsanız:
1. `Shared_Config.h` dosyasını kontrol edin
2. Include path'lerini doğrulayın
3. Derleme hatalarını kontrol edin
4. Gerekirse eski tanımlamaları kaldırın

---

**Not:** Bu sistem sayesinde artık tek bir yerden tüm ayarları yönetebilir, değişiklikleri kolayca uygulayabilirsiniz! 🎉 