
# 🔧 Akıllı Robotik İzleme ve Erişim Sistemi

---

## 🛡️ Modül 1: RFID Erişim ve Kontrol Sistemi

- **Donanım:**
  - Deneyap Kart 1 (ESP32 tabanlı)
  - RC522 RFID Sensörü

- **Fonksiyonlar:**
  - RFID kart okuma/yazma
  - MQTT üzerinden erişim verilerinin yayınlanması

- **Bağlantı ve Protokoller:**
  - RC522 ile SPI
  - WiFi üzerinden MQTT bağlantısı
  - ✅ Çoklu WiFi ağı desteği (WiFiMulti)
  - ✅ Otomatik yeniden bağlanma

- **MQTT Konuları:**
  - `rfid/reader/uid`
  - `rfid/command/write`
  - `rfid/status`

---

## 🤖 Modül 2: PID + Çizgi Takibi + GY-89 ile 9 Eksen IMU

- **Donanım:**
  - Arduino Mega 2560
  - QTR-8A analog çizgi sensörü
  - BTS7960B motor sürücüler (x2)
  - GY-89 Sensör Modülü:
    - MPU6050 (gyro + accel)
    - HMC5883L (magnetometre)
    - BMP180 (basınç + sıcaklık)
  - Deneyap Kart 2 (ESP32 tabanlı)

- **Fonksiyonlar:**
  - Çizgi takibi ve PID kontrol
  - 9 eksenli IMU verisi (ivme, jiroskop, yön, pusula)
  - Basınç ve yükseklik ölçümü
  - Deneyap Kart üzerinden MQTT ağına veri aktarımı
  - ✅ Çoklu WiFi ağı desteği (WiFiMulti)
  - ✅ Otomatik yeniden bağlanma
  - Pin konfigürasyonları EEPROM'a kayıt edilebilir ve MQTT üzerinden değiştirelebilir olmalaıdır


- **Haberleşme Yapısı:**

```
[GY-89 IMU] ↔ I²C ↔ [Arduino Mega] ↔ I²C ↔ [Deneyap Kart 2] ↔ WiFi ↔ MQTT
                          ↑                      ↓
                PID + QTR8A ve Motorlar     MQTT JSON Paketleri
                                                 ↓
                          RC522      ↔    [Deneyap Kart 1]
```

---

## 🔗 Sistem Bileşenleri Özeti

| Özellik / Kart     | Arduino Mega | Deneyap Kart 1 | Deneyap Kart 2 |
|--------------------|--------------|----------------|----------------|
| QTR8A Sensörü      | ✅            | ❌              | ❌              |
| GY-89 (IMU + Baro) | ✅            | ❌              | ❌              |
| Motor Kontrolü     | ✅            | ❌              | ❌              |
| RC522 RFID         | ❌            | ✅              | ❌              |
| MQTT / WiFi        | ❌            | ✅              | ✅              |
| I²C Haberleşme     | ✅            | ❌              | ✅              |
| Çoklu WiFi Desteği | ❌            | ✅              | ✅              |

---

## 📄 Lisans

Bu proje eğitim amaçlıdır. Donanım yapılandırmaları açık kaynaklıdır ve geliştirilebilir.
