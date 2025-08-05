# 🛡️ RFID Erişim ve Kontrol Sistemi

## 📋 Modül Genel Bakış

Bu modül, Deneyap Kart 1 (ESP32) üzerinde çalışan RFID erişim kontrol sistemidir. RC522 RFID sensörü kullanarak kart okuma/yazma işlemleri yapar ve MQTT üzerinden veri yayınlar.

## 🔧 Donanım Gereksinimleri

- **Deneyap Kart 1 (ESP32)**
- **RC522 RFID Sensörü**
- **Jumper Kablolar**

## 🔌 Pin Bağlantıları

| RC522 Pin | Deneyap Kart 1 Pin | Açıklama |
|-----------|-------------------|----------|
| SDA | GPIO5 | SPI Data |
| SCK | GPIO18 | SPI Clock |
| MOSI | GPIO23 | SPI MOSI |
| MISO | GPIO19 | SPI MISO |
| RST | GPIO22 | Reset |
| IRQ | GPIO21 | Interrupt |
| VCC | 3.3V | Güç |
| GND | GND | Toprak |

## 📡 MQTT Haberleşme

### Yayınlanan Konular

#### Kart Tespit Edildi
- **Topic:** `rfid/reader/uid`
- **Format:** JSON
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

#### Sistem Durumu
- **Topic:** `rfid/status`
- **Format:** JSON
```json
{
  "device": "deneyap_kart_1",
  "status": "connected",
  "timestamp": 1640995200000,
  "wifi_rssi": -45
}
```

### Dinlenen Konular

#### RFID Komutları
- **Topic:** `rfid/command/write`
- **Format:** JSON
```json
{
  "command": "write_card",
  "uid": "04:A3:B2:C1:D0:E5:F6",
  "data": "test_data"
}
```

#### Pin Konfigürasyonu
- **Topic:** `config/pins/deneyap_kart_1`
- **Format:** JSON
```json
{
  "pins": {
    "rfid": {
      "sda_pin": 5,
      "sck_pin": 18,
      "mosi_pin": 23,
      "miso_pin": 19,
      "rst_pin": 22,
      "irq_pin": 21
    }
  }
}
```

## 🔧 Özellikler

### ✅ Çoklu WiFi Desteği
- WiFiMulti kütüphanesi ile birden fazla WiFi ağı
- Otomatik ağ değiştirme
- Bağlantı kopması durumunda otomatik yeniden bağlanma

### ✅ MQTT Haberleşme
- JSON formatında veri yayınlama
- Gerçek zamanlı kart tespiti
- Komut tabanlı kontrol sistemi

### ✅ Pin Konfigürasyonu
- MQTT üzerinden pin değişikliği
- Dinamik pin yapılandırması
- EEPROM'a kayıt edilebilir ayarlar

### ✅ Güvenlik
- Kart tipi tespiti
- UID doğrulama
- Güvenli kart yazma işlemleri

## 🚀 Kurulum

1. **Donanım Bağlantıları:**
   - RC522 sensörünü Deneyap Kart 1'e bağlayın
   - Pin bağlantılarını kontrol edin

2. **Yazılım Kurulumu:**
   - Gerekli kütüphaneleri yükleyin:
     - `WiFiMulti`
     - `PubSubClient`
     - `MFRC522`
     - `ArduinoJson`

3. **Konfigürasyon:**
   - WiFi ayarlarını güncelleyin
   - MQTT broker ayarlarını yapın
   - Device ID'yi ayarlayın

## 📊 Test ve Doğrulama

### Kart Okuma Testi
1. RFID kartını sensöre yaklaştırın
2. Serial monitörde UID'yi kontrol edin
3. MQTT mesajının gönderildiğini doğrulayın

### WiFi Bağlantı Testi
1. Serial monitörde WiFi durumunu kontrol edin
2. IP adresini not edin
3. MQTT bağlantısını test edin

### Pin Konfigürasyon Testi
1. MQTT üzerinden pin değişikliği gönderin
2. Sistemin yeniden başladığını kontrol edin
3. Yeni pinlerin çalıştığını doğrulayın

## 🔧 Hata Ayıklama

### Yaygın Sorunlar

#### RFID Sensörü Çalışmıyor
- Pin bağlantılarını kontrol edin
- SPI bağlantısını doğrulayın
- Sensör versiyonunu kontrol edin

#### WiFi Bağlantı Sorunu
- SSID ve şifreleri kontrol edin
- Sinyal gücünü kontrol edin
- Router ayarlarını kontrol edin

#### MQTT Bağlantı Sorunu
- Broker adresini kontrol edin
- Kullanıcı adı ve şifreyi doğrulayın
- Port numarasını kontrol edin

## 📈 Performans

- **Kart Tespit Süresi:** < 1 saniye
- **MQTT Gecikme:** < 100ms
- **WiFi Yeniden Bağlanma:** < 5 saniye
- **Pin Değişiklik Süresi:** < 2 saniye

## 🔒 Güvenlik Notları

- RFID kartlarının güvenliğini sağlayın
- MQTT broker güvenliğini kontrol edin
- WiFi şifrelerini güçlü tutun
- Kart yazma işlemlerini sınırlayın

## 📄 Lisans

Bu modül eğitim amaçlıdır. Güvenlik uygulamalarında ek önlemler alınmalıdır. 