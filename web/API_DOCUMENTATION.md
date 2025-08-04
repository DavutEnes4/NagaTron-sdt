# 📚 API Dokümantasyonu

Bu dokümantasyon RFID-I2C Web Sunucusu'nun tüm API endpoint'lerini detaylı olarak açıklar.

## 🔗 Temel Bilgiler

- **Base URL**: `http://localhost:5000`
- **Content-Type**: `application/json`
- **Authentication**: Şu an için gerekli değil

## 📊 Response Format

Tüm API yanıtları aşağıdaki standart formatta döner:

```json
{
  "success": true,
  "message": "İşlem başarılı",
  "data": {...},
  "timestamp": "2024-01-01 12:00:00"
}
```

### Hata Yanıtları

```json
{
  "success": false,
  "message": "Hata açıklaması",
  "error": "Detaylı hata bilgisi",
  "timestamp": "2024-01-01 12:00:00"
}
```

## 🔗 Temel Endpoint'ler

### 1. Sağlık Kontrolü

**Endpoint:** `GET /api/health`

Sistemin genel sağlık durumunu kontrol eder.

**Response:**
```json
{
  "status": "healthy",
  "checks": {
    "mqtt_connection": true,
    "i2c_system": true,
    "rfid_system": true,
    "web_server": true
  },
  "details": {
    "mqtt_broker": "nagatron-sdt.local:1883",
    "i2c_timeout": "30s",
    "rfid_status": "active"
  },
  "timestamp": "2024-01-01 12:00:00"
}
```

**Status Codes:**
- `200` - Sistem sağlıklı
- `503` - Sistem sağlıksız

### 2. Sistem Bilgileri

**Endpoint:** `GET /api/system-info`

Sistem hakkında detaylı bilgileri döndürür.

**Response:**
```json
{
  "system": {
    "name": "RFID-I2C Web Sunucusu",
    "version": "2.0.0",
    "status": "running",
    "uptime": "active"
  },
  "mqtt": {
    "connected": true,
    "broker": "nagatron-sdt.local",
    "last_activity": "2024-01-01 12:00:00"
  },
  "rfid": {
    "total_reads": 15,
    "total_writes": 3,
    "last_activity": "2024-01-01 12:00:00",
    "status": "active"
  },
  "i2c": {
    "connected": true,
    "motor_active": true,
    "qtr_active": true,
    "healthy": true,
    "last_ping": "2024-01-01 12:00:00"
  }
}
```

### 3. MQTT Durumu

**Endpoint:** `GET /api/mqtt-status`

MQTT bağlantı durumunu kontrol eder.

**Response:**
```json
{
  "connected": true,
  "broker": "nagatron-sdt.local",
  "port": 1883,
  "timestamp": "2024-01-01 12:00:00"
}
```

### 4. Ping Testi

**Endpoint:** `GET /api/ping`

Basit ping testi.

**Response:**
```json
{
  "success": true,
  "message": "pong",
  "data": {
    "timestamp": "2024-01-01 12:00:00",
    "server": "RFID-I2C Web Server"
  }
}
```

### 5. Tüm İstatistikler

**Endpoint:** `GET /api/stats`

Tüm sistem istatistiklerini döndürür.

**Response:**
```json
{
  "rfid_stats": {
    "total_reads": 15,
    "total_writes": 3,
    "last_activity": "2024-01-01 12:00:00",
    "success_rate": 100.0
  },
  "i2c_stats": {
    "system_status": {
      "i2c_connected": true,
      "motor_active": true,
      "qtr_active": true,
      "last_ping": "2024-01-01 12:00:00"
    },
    "motor_status": {
      "left_speed": 150,
      "right_speed": 150,
      "direction": "forward",
      "last_update": "2024-01-01 12:00:00"
    },
    "qtr_status": {
      "sensors": [0, 1, 1, 0, 0, 0, 0, 0],
      "position": 2,
      "line_detected": true,
      "last_update": "2024-01-01 12:00:00"
    },
    "healthy": true
  },
  "mqtt_stats": {
    "connected": true,
    "broker": "nagatron-sdt.local"
  },
  "timestamp": "2024-01-01 12:00:00"
}
```

### 6. Tüm Verileri Sıfırla

**Endpoint:** `POST /api/reset-all`

Tüm sistem verilerini sıfırlar.

**Response:**
```json
{
  "success": true,
  "message": "Tüm sistem verileri sıfırlandı",
  "data": {
    "rfid_data": {...},
    "i2c_data": {...}
  }
}
```

## 🔐 RFID Endpoint'leri

### 1. RFID Verilerini Al

**Endpoint:** `GET /api/rfid-data`

Tüm RFID verilerini döndürür.

**Response:**
```json
{
  "last_card_id": "1234567890",
  "last_read_time": "2024-01-01 12:00:00",
  "last_written_data": "Merhaba Dünya",
  "last_write_time": "2024-01-01 12:00:00",
  "total_reads": 15,
  "total_writes": 3,
  "card_history": [
    {
      "card_id": "1234567890",
      "data": "Kart verisi",
      "time": "2024-01-01 12:00:00",
      "type": "read"
    }
  ],
  "write_history": [
    {
      "card_id": "1234567890",
      "data": "Yazılan veri",
      "time": "2024-01-01 12:00:00",
      "status": "success"
    }
  ]
}
```

### 2. RFID İstatistikleri

**Endpoint:** `GET /api/rfid-stats`

RFID istatistiklerini döndürür.

**Response:**
```json
{
  "total_reads": 15,
  "total_writes": 3,
  "last_activity": "2024-01-01 12:00:00",
  "success_rate": 100.0
}
```

### 3. RFID Kartına Yaz

**Endpoint:** `POST /api/write-rfid`

RFID kartına veri yazar.

**Request Body:**
```json
{
  "card_id": "1234567890",
  "data": "Merhaba Dünya"
}
```

**Response:**
```json
{
  "success": true,
  "message": "Yazma komutu gönderildi",
  "data": "Merhaba Dünya"
}
```

**Validation:**
- `card_id`: String (opsiyonel)
- `data`: String (zorunlu, boş olamaz)

### 4. Kart Geçmişi

**Endpoint:** `GET /api/card-history`

Kart okuma geçmişini döndürür.

**Response:**
```json
{
  "success": true,
  "history": [
    {
      "card_id": "1234567890",
      "data": "Kart verisi",
      "time": "2024-01-01 12:00:00",
      "type": "read"
    }
  ],
  "total_reads": 15
}
```

### 5. Yazma Geçmişi

**Endpoint:** `GET /api/write-history`

Kart yazma geçmişini döndürür.

**Response:**
```json
{
  "success": true,
  "history": [
    {
      "card_id": "1234567890",
      "data": "Yazılan veri",
      "time": "2024-01-01 12:00:00",
      "status": "success"
    }
  ],
  "total_writes": 3
}
```

### 6. Geçmişi Temizle

**Endpoint:** `POST /api/clear-history`

RFID geçmişini temizler.

**Request Body:**
```json
{
  "type": "read"  // "read", "write" veya null (hepsi)
}
```

**Response:**
```json
{
  "success": true,
  "message": "Tüm geçmiş temizlendi",
  "data": {...}
}
```

### 7. RFID Verilerini Sıfırla

**Endpoint:** `POST /api/reset-data`

Tüm RFID verilerini sıfırlar.

**Response:**
```json
{
  "success": true,
  "message": "RFID verileri sıfırlandı",
  "data": {...}
}
```

## ⚙️ I2C Endpoint'leri

### 1. I2C Verilerini Al

**Endpoint:** `GET /api/i2c-data`

Tüm I2C verilerini döndürür.

**Response:**
```json
{
  "pid_settings": {
    "kp": 2.0,
    "ki": 0.1,
    "kd": 0.5,
    "setpoint": 0
  },
  "pin_settings": {
    "motor_left_pin1": 5,
    "motor_left_pin2": 6,
    "motor_right_pin1": 9,
    "motor_right_pin2": 10,
    "enable_left": 3,
    "enable_right": 11
  },
  "speed_control": {
    "base_speed": 150,
    "max_speed": 255,
    "min_speed": 50
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
  },
  "system_status": {
    "i2c_connected": true,
    "motor_active": true,
    "qtr_active": true,
    "last_ping": "2024-01-01 12:00:00"
  }
}
```

### 2. Sistem Durumu

**Endpoint:** `GET /api/system-status`

I2C sistem durumunu döndürür.

**Response:**
```json
{
  "i2c_connected": true,
  "motor_active": true,
  "qtr_active": true,
  "last_ping": "2024-01-01 12:00:00",
  "healthy": true
}
```

### 3. PID Ayarlarını Güncelle

**Endpoint:** `POST /api/update-pid`

PID kontrol ayarlarını günceller.

**Request Body:**
```json
{
  "kp": 2.5,
  "ki": 0.2,
  "kd": 0.8,
  "setpoint": 0
}
```

**Validation:**
- `kp`: Float (-1000 to 1000)
- `ki`: Float (-1000 to 1000)
- `kd`: Float (-1000 to 1000)
- `setpoint`: Float (-1000 to 1000)

**Response:**
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

### 4. Pin Ayarlarını Güncelle

**Endpoint:** `POST /api/update-pins`

Motor pin ayarlarını günceller.

**Request Body:**
```json
{
  "motor_left_pin1": 5,
  "motor_left_pin2": 6,
  "motor_right_pin1": 9,
  "motor_right_pin2": 10,
  "enable_left": 3,
  "enable_right": 11
}
```

**Validation:**
- Tüm pin değerleri: Integer (0 to 53)

**Response:**
```json
{
  "success": true,
  "message": "Pin ayarları güncellendi",
  "data": {...}
}
```

### 5. Hız Ayarlarını Güncelle

**Endpoint:** `POST /api/update-speed`

Motor hız ayarlarını günceller.

**Request Body:**
```json
{
  "base_speed": 150,
  "max_speed": 255,
  "min_speed": 50
}
```

**Validation:**
- `base_speed`: Integer (0 to 255)
- `max_speed`: Integer (0 to 255)
- `min_speed`: Integer (0 to 255)

**Response:**
```json
{
  "success": true,
  "message": "Hız ayarları güncellendi",
  "data": {...}
}
```

### 6. Motor Kontrolü

**Endpoint:** `POST /api/motor-control`

Motor kontrol komutları gönderir.

**Request Body:**
```json
{
  "action": "forward",
  "speed": 200
}
```

**Mümkün Action'lar:**
- `forward` - İleri hareket
- `backward` - Geri hareket
- `left` - Sola dönüş
- `right` - Sağa dönüş
- `stop` - Durma
- `emergency_stop` - Acil durum

**Validation:**
- `action`: String (yukarıdaki değerlerden biri)
- `speed`: Integer (0 to 255, opsiyonel)

**Response:**
```json
{
  "success": true,
  "message": "Motor komutu gönderildi: forward",
  "action": "forward",
  "speed": 200
}
```

### 7. Motor Durumu

**Endpoint:** `GET /api/motor-status`

Motor durumunu döndürür.

**Response:**
```json
{
  "left_speed": 150,
  "right_speed": 150,
  "direction": "forward",
  "last_update": "2024-01-01 12:00:00"
}
```

### 8. QTR Durumu

**Endpoint:** `GET /api/qtr-status`

QTR sensör durumunu döndürür.

**Response:**
```json
{
  "sensors": [0, 1, 1, 0, 0, 0, 0, 0],
  "position": 2,
  "line_detected": true,
  "last_update": "2024-01-01 12:00:00"
}
```

### 9. Acil Durum Durdurma

**Endpoint:** `POST /api/emergency-stop`

Tüm motorları acil durumda durdurur.

**Response:**
```json
{
  "success": true,
  "message": "Acil durum komutu gönderildi - motorlar durduruldu"
}
```

### 10. I2C Verilerini Sıfırla

**Endpoint:** `POST /api/reset-i2c-data`

Tüm I2C verilerini sıfırlar.

**Response:**
```json
{
  "success": true,
  "message": "I2C verileri sıfırlandı",
  "data": {...}
}
```

## 🔧 Hata Kodları

| HTTP Status | Açıklama |
|-------------|----------|
| 200 | Başarılı |
| 400 | Geçersiz istek (validation hatası) |
| 405 | Method not allowed |
| 500 | Sunucu hatası |
| 503 | Servis kullanılamıyor |

## 📝 Örnek Kullanım

### cURL Örnekleri

```bash
# Sağlık kontrolü
curl http://localhost:5000/api/health

# RFID verileri
curl http://localhost:5000/api/rfid-data

# RFID kartına yaz
curl -X POST http://localhost:5000/api/write-rfid \
  -H "Content-Type: application/json" \
  -d '{"card_id": "1234567890", "data": "Test verisi"}'

# PID ayarları güncelle
curl -X POST http://localhost:5000/api/update-pid \
  -H "Content-Type: application/json" \
  -d '{"kp": 2.5, "ki": 0.2, "kd": 0.8, "setpoint": 0}'

# Motor kontrolü
curl -X POST http://localhost:5000/api/motor-control \
  -H "Content-Type: application/json" \
  -d '{"action": "forward", "speed": 200}'
```

### JavaScript Örnekleri

```javascript
// RFID verilerini al
fetch('/api/rfid-data')
  .then(response => response.json())
  .then(data => console.log(data));

// RFID kartına yaz
fetch('/api/write-rfid', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    card_id: '1234567890',
    data: 'Test verisi'
  })
})
.then(response => response.json())
.then(data => console.log(data));

// Motor kontrolü
fetch('/api/motor-control', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    action: 'forward',
    speed: 200
  })
})
.then(response => response.json())
.then(data => console.log(data));
```

### Python Örnekleri

```python
import requests

# RFID verilerini al
response = requests.get('http://localhost:5000/api/rfid-data')
data = response.json()
print(data)

# RFID kartına yaz
response = requests.post('http://localhost:5000/api/write-rfid', 
  json={
    'card_id': '1234567890',
    'data': 'Test verisi'
  })
data = response.json()
print(data)

# Motor kontrolü
response = requests.post('http://localhost:5000/api/motor-control',
  json={
    'action': 'forward',
    'speed': 200
  })
data = response.json()
print(data)
```

## 🔄 WebSocket Events

WebSocket bağlantısı için Socket.IO kullanılır:

```javascript
const socket = io('http://localhost:5000');

// Bağlantı
socket.on('connect', () => {
  console.log('Bağlandı');
});

// RFID güncellemeleri
socket.on('rfid_update', (data) => {
  console.log('RFID güncellemesi:', data);
});

// I2C güncellemeleri
socket.on('i2c_update', (data) => {
  console.log('I2C güncellemesi:', data);
});

// Motor güncellemeleri
socket.on('motor_update', (data) => {
  console.log('Motor güncellemesi:', data);
});

// QTR güncellemeleri
socket.on('qtr_update', (data) => {
  console.log('QTR güncellemesi:', data);
});
```

---

**Not:** Bu API dokümantasyonu sürekli güncellenmektedir. En güncel bilgiler için GitHub repository'sini kontrol edin. 