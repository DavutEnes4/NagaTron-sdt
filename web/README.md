# 🚀 RFID-I2C Web Sunucusu

Modern, modüler yapıda geliştirilmiş RFID ve I2C motor kontrol sistemi. Flask tabanlı REST API ve WebSocket desteği ile gerçek zamanlı veri iletişimi sağlar.

## 📋 İçindekiler

- [Özellikler](#-özellikler)
- [Teknolojiler](#-teknolojiler)
- [Kurulum](#-kurulum)
- [API Dokümantasyonu](#-api-dokümantasyonu)
- [WebSocket Events](#-websocket-events)
- [MQTT Konuları](#-mqtt-konuları)
- [Proje Yapısı](#-proje-yapısı)
- [Geliştirme](#-geliştirme)
- [Test](#-test)

## ✨ Özellikler

### 🔐 RFID Sistemi
- Kart okuma ve yazma işlemleri
- Gerçek zamanlı veri takibi
- Geçmiş kayıtları
- İstatistik raporlama

### ⚙️ I2C Motor Kontrolü
- PID kontrol sistemi
- Motor hız ve yön kontrolü
- QTR sensör desteği
- Pin konfigürasyonu

### 🌐 Web Arayüzü
- Modern responsive tasarım
- Gerçek zamanlı veri güncelleme
- WebSocket bağlantısı
- REST API desteği

### 📡 MQTT Entegrasyonu
- Otomatik yeniden bağlanma
- Hata yönetimi
- Gerçek zamanlı mesajlaşma

## 🛠️ Teknolojiler

- **Backend**: Flask, Flask-SocketIO
- **MQTT**: paho-mqtt
- **Frontend**: HTML5, CSS3, JavaScript
- **Veri Formatı**: JSON
- **İletişim**: WebSocket, HTTP REST API

## 🚀 Kurulum

### Gereksinimler
- Python 3.8+
- MQTT Broker (nagatron-sdt.local:1883)

### 1. Projeyi İndirin
```bash
git clone <repository-url>
cd web
```

### 2. Bağımlılıkları Yükleyin
```bash
pip install -r requirements.txt
```

### 3. Konfigürasyon
`config/settings.py` dosyasında MQTT broker ayarlarını kontrol edin:
```python
MQTT_CONFIG = {
    'BROKER': "nagatron-sdt.local",
    'PORT': 1883,
    'KEEPALIVE': 60
}
```

### 4. Sunucuyu Başlatın
```bash
python app.py
```

Sunucu `http://localhost:5000` adresinde çalışmaya başlayacak.

## 📚 API Dokümantasyonu

### 🔗 Temel Endpoint'ler

#### Sağlık Kontrolü
```http
GET /api/health
```
**Yanıt:**
```json
{
  "status": "healthy",
  "checks": {
    "mqtt_connection": true,
    "i2c_system": true,
    "rfid_system": true,
    "web_server": true
  },
  "timestamp": "2024-01-01 12:00:00"
}
```

#### Sistem Bilgileri
```http
GET /api/system-info
```
**Yanıt:**
```json
{
  "system": {
    "name": "RFID-I2C Web Sunucusu",
    "version": "2.0.0",
    "status": "running"
  },
  "mqtt": {
    "connected": true,
    "broker": "nagatron-sdt.local"
  },
  "rfid": {
    "total_reads": 15,
    "total_writes": 3,
    "last_activity": "2024-01-01 12:00:00"
  },
  "i2c": {
    "connected": true,
    "motor_active": true,
    "qtr_active": true,
    "healthy": true
  }
}
```

#### MQTT Durumu
```http
GET /api/mqtt-status
```
**Yanıt:**
```json
{
  "connected": true,
  "broker": "nagatron-sdt.local",
  "port": 1883,
  "timestamp": "2024-01-01 12:00:00"
}
```

### 🔐 RFID Endpoint'leri

#### RFID Verilerini Al
```http
GET /api/rfid-data
```
**Yanıt:**
```json
{
  "last_card_id": "1234567890",
  "last_read_time": "2024-01-01 12:00:00",
  "total_reads": 15,
  "total_writes": 3,
  "card_history": [...],
  "write_history": [...]
}
```

#### RFID Kartına Yaz
```http
POST /api/write-rfid
Content-Type: application/json

{
  "card_id": "1234567890",
  "data": "Merhaba Dünya"
}
```
**Yanıt:**
```json
{
  "success": true,
  "message": "Yazma komutu gönderildi",
  "data": "Merhaba Dünya"
}
```

#### RFID İstatistikleri
```http
GET /api/rfid-stats
```
**Yanıt:**
```json
{
  "total_reads": 15,
  "total_writes": 3,
  "last_activity": "2024-01-01 12:00:00",
  "success_rate": 100.0
}
```

### ⚙️ I2C Endpoint'leri

#### I2C Verilerini Al
```http
GET /api/i2c-data
```
**Yanıt:**
```json
{
  "pid_settings": {
    "kp": 2.0,
    "ki": 0.1,
    "kd": 0.5,
    "setpoint": 0
  },
  "motor_data": {
    "left_speed": 150,
    "right_speed": 150,
    "direction": "forward",
    "last_update": "2024-01-01 12:00:00"
  },
  "qtr_data": {
    "sensors": [0, 1, 1, 0, 0, 0, 0, 0],
    "position": 2,
    "line_detected": true,
    "last_update": "2024-01-01 12:00:00"
  }
}
```

#### PID Ayarlarını Güncelle
```http
POST /api/update-pid
Content-Type: application/json

{
  "kp": 2.5,
  "ki": 0.2,
  "kd": 0.8,
  "setpoint": 0
}
```
**Yanıt:**
```json
{
  "success": true,
  "message": "PID ayarları güncellendi",
  "data": {
    "kp": 2.5,
    "ki": 0.2,
    "kd": 0.8,
    "setpoint": 0
  }
}
```

#### Motor Kontrolü
```http
POST /api/motor-control
Content-Type: application/json

{
  "action": "forward",
  "speed": 200
}
```
**Mümkün Action'lar:**
- `forward` - İleri
- `backward` - Geri
- `left` - Sola dön
- `right` - Sağa dön
- `stop` - Dur
- `emergency_stop` - Acil durum

#### Pin Ayarlarını Güncelle
```http
POST /api/update-pins
Content-Type: application/json

{
  "motor_left_pin1": 5,
  "motor_left_pin2": 6,
  "motor_right_pin1": 9,
  "motor_right_pin2": 10,
  "enable_left": 3,
  "enable_right": 11
}
```

#### Hız Ayarlarını Güncelle
```http
POST /api/update-speed
Content-Type: application/json

{
  "base_speed": 150,
  "max_speed": 255,
  "min_speed": 50
}
```

## 🔌 WebSocket Events

### Bağlantı Events
```javascript
// Bağlantı
socket.on('connect', () => {
  console.log('Bağlandı');
});

// Bağlantı kesme
socket.on('disconnect', () => {
  console.log('Bağlantı kesildi');
});
```

### RFID Events
```javascript
// RFID veri güncellemesi
socket.on('rfid_update', (data) => {
  console.log('RFID güncellemesi:', data);
});

// Yazma durumu
socket.on('write_status', (data) => {
  console.log('Yazma durumu:', data);
});

// RFID yazma komutu gönder
socket.emit('write_rfid', {
  card_id: '1234567890',
  data: 'Merhaba Dünya'
});
```

### I2C Events
```javascript
// I2C veri güncellemesi
socket.on('i2c_update', (data) => {
  console.log('I2C güncellemesi:', data);
});

// Motor veri güncellemesi
socket.on('motor_update', (data) => {
  console.log('Motor güncellemesi:', data);
});

// QTR sensör güncellemesi
socket.on('qtr_update', (data) => {
  console.log('QTR güncellemesi:', data);
});

// PID ayarları güncelle
socket.emit('update_pid_settings', {
  kp: 2.5,
  ki: 0.2,
  kd: 0.8,
  setpoint: 0
});

// Motor komutu gönder
socket.emit('motor_command', {
  action: 'forward',
  speed: 200
});
```

## 📡 MQTT Konuları

### RFID Konuları
- `rfid/data` - Kart okuma verileri
- `rfid/write` - Kart yazma komutları
- `rfid/status` - Yazma durumu

### I2C Konuları
- `i2c/command` - I2C komutları
- `i2c/data` - Genel I2C verileri
- `i2c/status` - I2C durum bilgileri
- `motor/data` - Motor verileri
- `qtr/data` - QTR sensör verileri

## 📁 Proje Yapısı

```
web/
├── app.py                 # Ana uygulama dosyası
├── requirements.txt       # Python bağımlılıkları
├── README.md             # Bu dosya
├── config/
│   └── settings.py       # Konfigürasyon ayarları
├── api/
│   ├── general_routes.py # Genel API endpoint'leri
│   ├── rfid_routes.py    # RFID API endpoint'leri
│   └── i2c_routes.py     # I2C API endpoint'leri
├── data/
│   ├── rfid_data.py      # RFID veri yönetimi
│   └── i2c_data.py       # I2C veri yönetimi
├── mqtt/
│   ├── client.py         # MQTT istemci
│   ├── handlers.py       # MQTT mesaj işleyicileri
│   └── topics.py         # MQTT konu tanımları
├── websocket/
│   └── events.py         # WebSocket event handler'ları
├── utils/
│   └── helpers.py        # Yardımcı fonksiyonlar
├── static/
│   ├── css/              # CSS dosyaları
│   ├── js/               # JavaScript dosyaları
│   └── images/           # Resim dosyaları
└── templates/
    └── index.html        # Ana sayfa template'i
```

## 🛠️ Geliştirme

### Yeni Endpoint Ekleme
1. `api/` klasöründe yeni route dosyası oluşturun
2. Blueprint kullanarak endpoint'leri tanımlayın
3. `app.py`'de blueprint'i kaydedin

### Yeni WebSocket Event Ekleme
1. `websocket/events.py`'de yeni event handler ekleyin
2. `register_events()` metodunda event'i kaydedin

### Yeni MQTT Konusu Ekleme
1. `config/settings.py`'de yeni konuyu tanımlayın
2. `mqtt/handlers.py`'de mesaj işleyici ekleyin
3. `mqtt/client.py`'de abonelik ekleyin

## 🧪 Test

### API Testleri
```bash
# Sağlık kontrolü
curl http://localhost:5000/api/health

# Sistem bilgileri
curl http://localhost:5000/api/system-info

# RFID verileri
curl http://localhost:5000/api/rfid-data

# I2C verileri
curl http://localhost:5000/api/i2c-data
```

### WebSocket Test
```javascript
// Browser console'da test edin
const socket = io('http://localhost:5000');

socket.on('connect', () => {
  console.log('Bağlandı');
});

socket.emit('get_rfid_data');
socket.on('rfid_update', (data) => {
  console.log('RFID verisi:', data);
});
```

### MQTT Test
```bash
# MQTT broker'a test mesajı gönder
mosquitto_pub -h nagatron-sdt.local -t "rfid/data" -m '{"card_id": "test123"}'
```

## 📝 Lisans

Bu proje MIT lisansı altında lisanslanmıştır.

## 🤝 Katkıda Bulunma

1. Fork edin
2. Feature branch oluşturun (`git checkout -b feature/amazing-feature`)
3. Commit edin (`git commit -m 'Add amazing feature'`)
4. Push edin (`git push origin feature/amazing-feature`)
5. Pull Request oluşturun

## 📞 İletişim

Proje hakkında sorularınız için:
- Email: [davutenes4@yaani.com]
- GitHub: [davutenes4]

---

**Not:** Bu dokümantasyon sürekli güncellenmektedir. En güncel bilgiler için GitHub repository'sini kontrol edin. 