# 👨‍💻 Geliştirici Rehberi

Bu rehber, RFID-I2C Web Sunucusu'na katkıda bulunmak isteyen geliştiriciler için hazırlanmıştır.

## 🚀 Başlangıç

### Gereksinimler
- Python 3.8+
- Git
- MQTT Broker (nagatron-sdt.local:1883)

### Geliştirme Ortamı Kurulumu

1. **Repository'yi klonlayın**
```bash
git clone <repository-url>
cd web
```

2. **Virtual environment oluşturun**
```bash
python -m venv venv
source venv/bin/activate  # Linux/Mac
# veya
venv\Scripts\activate     # Windows
```

3. **Bağımlılıkları yükleyin**
```bash
pip install -r requirements.txt
```

4. **Geliştirme modunda çalıştırın**
```bash
python app.py
```

## 📁 Proje Yapısı

```
web/
├── app.py                 # Ana uygulama dosyası
├── requirements.txt       # Python bağımlılıkları
├── README.md             # Proje dokümantasyonu
├── API_DOCUMENTATION.md  # API dokümantasyonu
├── DEVELOPER_GUIDE.md    # Bu dosya
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

## 🛠️ Geliştirme Kuralları

### 1. Kod Stili
- **PEP 8** standartlarına uyun
- **Türkçe yorumlar** kullanın
- **Anlamlı değişken isimleri** seçin
- **Docstring'ler** ekleyin

### 2. Dosya Organizasyonu
- Her modül **tek bir sorumluluğa** sahip olmalı
- **Blueprint** kullanarak route'ları organize edin
- **Global değişkenler** yerine **manager sınıfları** kullanın

### 3. Hata Yönetimi
- **Try-catch** blokları kullanın
- **Logging** yapın
- **Kullanıcı dostu hata mesajları** döndürün

### 4. Test Yazma
- **Unit testler** yazın
- **Integration testler** ekleyin
- **API testleri** yapın

## 🔧 Yeni Özellik Ekleme

### 1. Yeni API Endpoint Ekleme

#### Adım 1: Route Dosyası Oluşturun
```python
# api/new_feature_routes.py
from flask import Blueprint, jsonify, request
from utils.helpers import create_response, log_error, log_info

new_feature_bp = Blueprint('new_feature', __name__, url_prefix='/api')

@new_feature_bp.route('/new-endpoint', methods=['GET'])
def get_new_data():
    """Yeni endpoint açıklaması"""
    try:
        # İşlemler burada
        return jsonify(create_response(True, "Başarılı", data={}))
    except Exception as e:
        log_error(f"Hata: {e}", "NEW_FEATURE")
        return jsonify(create_response(False, "Hata oluştu", error=str(e))), 500
```

#### Adım 2: Blueprint'i Kaydedin
```python
# app.py
from api.new_feature_routes import new_feature_bp

# Blueprint'leri kaydet
app.register_blueprint(new_feature_bp)
```

### 2. Yeni WebSocket Event Ekleme

#### Adım 1: Event Handler Ekleyin
```python
# websocket/events.py
def _handle_new_event(self, data):
    """Yeni event handler'ı"""
    try:
        # İşlemler burada
        emit('new_event_success', {'message': 'Başarılı'})
    except Exception as e:
        emit('new_event_error', {'message': f'Hata: {str(e)}'})
```

#### Adım 2: Event'i Kaydedin
```python
# websocket/events.py - register_events() metodunda
@self.socketio.on('new_event')
def handle_new_event(data):
    self._handle_new_event(data)
```

### 3. Yeni MQTT Konusu Ekleme

#### Adım 1: Konuyu Tanımlayın
```python
# config/settings.py
NEW_TOPICS = {
    'DATA_TOPIC': "new/data",
    'COMMAND_TOPIC': "new/command"
}
```

#### Adım 2: Handler Ekleyin
```python
# mqtt/handlers.py
def _handle_new_data(self, message):
    """Yeni veri işleyici"""
    try:
        # İşlemler burada
        if self.socketio:
            self.socketio.emit('new_update', data)
    except Exception as e:
        print(f"Yeni veri işleme hatası: {e}")
```

#### Adım 3: Abonelik Ekleyin
```python
# mqtt/client.py - _subscribe_to_topics() metodunda
for topic in NEW_TOPICS.values():
    result = self.client.subscribe(topic)
    if result[0] == mqtt.MQTT_ERR_SUCCESS:
        print(f"✅ Yeni konuya abone olundu: {topic}")
```

### 4. Yeni Veri Yöneticisi Ekleme

#### Adım 1: Manager Sınıfı Oluşturun
```python
# data/new_data.py
from datetime import datetime
from config.settings import GENERAL_CONFIG

class NewDataManager:
    def __init__(self):
        self.data = {
            'status': 'inactive',
            'last_update': '',
            'history': []
        }
    
    def get_data(self):
        """Tüm verileri döndür"""
        return self.data
    
    def update_data(self, new_data):
        """Verileri güncelle"""
        try:
            current_time = datetime.now().strftime(GENERAL_CONFIG['DATE_FORMAT'])
            self.data.update(new_data)
            self.data['last_update'] = current_time
            return True
        except Exception as e:
            print(f"Veri güncelleme hatası: {e}")
            return False

# Global instance
new_manager = NewDataManager()
```

## 🧪 Test Yazma

### 1. Unit Test Örneği
```python
# tests/test_rfid_data.py
import unittest
from data.rfid_data import RFIDDataManager

class TestRFIDDataManager(unittest.TestCase):
    def setUp(self):
        self.rfid_manager = RFIDDataManager()
    
    def test_update_card_read(self):
        """Kart okuma testi"""
        result = self.rfid_manager.update_card_read('test_card_123')
        self.assertTrue(result)
        self.assertEqual(self.rfid_manager.data['last_card_id'], 'test_card_123')
        self.assertEqual(self.rfid_manager.data['total_reads'], 1)
    
    def test_get_statistics(self):
        """İstatistik testi"""
        stats = self.rfid_manager.get_statistics()
        self.assertIn('total_reads', stats)
        self.assertIn('total_writes', stats)
```

### 2. API Test Örneği
```python
# tests/test_api.py
import requests
import json

class TestAPI(unittest.TestCase):
    def setUp(self):
        self.base_url = 'http://localhost:5000'
    
    def test_health_endpoint(self):
        """Sağlık kontrolü testi"""
        response = requests.get(f'{self.base_url}/api/health')
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertIn('status', data)
        self.assertIn('checks', data)
    
    def test_rfid_data_endpoint(self):
        """RFID veri endpoint testi"""
        response = requests.get(f'{self.base_url}/api/rfid-data')
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertIn('total_reads', data)
        self.assertIn('total_writes', data)
```

## 🔍 Debugging

### 1. Logging
```python
from utils.helpers import log_error, log_info

# Bilgi logu
log_info("İşlem başarılı", "MODULE_NAME")

# Hata logu
log_error("Hata oluştu", "MODULE_NAME")
```

### 2. Debug Modu
```python
# app.py
if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
```

### 3. MQTT Debug
```python
# mqtt/client.py
def _on_message(self, client, userdata, msg):
    print(f"📨 Gelen veri - Topic: {msg.topic}, Mesaj: {msg.payload.decode('utf-8')}")
```

## 📝 Dokümantasyon

### 1. Kod Dokümantasyonu
```python
def complex_function(param1, param2):
    """
    Karmaşık fonksiyon açıklaması.
    
    Args:
        param1 (str): İlk parametre açıklaması
        param2 (int): İkinci parametre açıklaması
    
    Returns:
        dict: Sonuç açıklaması
    
    Raises:
        ValueError: Hata durumu açıklaması
    
    Example:
        >>> result = complex_function("test", 123)
        >>> print(result)
        {'status': 'success'}
    """
    # Fonksiyon implementasyonu
    pass
```

### 2. API Dokümantasyonu
```markdown
## Yeni Endpoint

### GET /api/new-endpoint

Yeni endpoint açıklaması.

**Response:**
```json
{
  "success": true,
  "data": {
    "field1": "value1",
    "field2": "value2"
  }
}
```
```

## 🚀 Deployment

### 1. Production Ayarları
```python
# config/settings.py
PRODUCTION_CONFIG = {
    'DEBUG': False,
    'HOST': '0.0.0.0',
    'PORT': 5000,
    'SECRET_KEY': 'production_secret_key'
}
```

### 2. Gunicorn ile Deployment
```bash
pip install gunicorn
gunicorn -w 4 -b 0.0.0.0:5000 app:app
```

### 3. Docker ile Deployment
```dockerfile
FROM python:3.9-slim

WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt

COPY . .
EXPOSE 5000

CMD ["python", "app.py"]
```

## 🤝 Katkıda Bulunma

### 1. Issue Açma
- **Bug report**: Hata açıklaması, adımlar, beklenen davranış
- **Feature request**: Özellik açıklaması, kullanım senaryosu

### 2. Pull Request
1. **Fork** edin
2. **Feature branch** oluşturun
3. **Değişiklikleri** yapın
4. **Testleri** yazın
5. **Dokümantasyonu** güncelleyin
6. **Pull request** açın

### 3. Commit Mesajları
```
feat: yeni özellik eklendi
fix: hata düzeltildi
docs: dokümantasyon güncellendi
test: test eklendi
refactor: kod yeniden düzenlendi
```

## 📚 Faydalı Kaynaklar

- [Flask Dokümantasyonu](https://flask.palletsprojects.com/)
- [Flask-SocketIO Dokümantasyonu](https://flask-socketio.readthedocs.io/)
- [Paho MQTT Dokümantasyonu](https://pypi.org/project/paho-mqtt/)
- [PEP 8 Style Guide](https://www.python.org/dev/peps/pep-0008/)

## 📞 İletişim

Geliştirme ile ilgili sorularınız için:
- **GitHub Issues**: [Repository Issues](https://github.com/DavutEnes4/NagaTron-sdt/)
- **Email**: [davutenes4@yaani.com]

---

**Not:** Bu rehber sürekli güncellenmektedir. En güncel bilgiler için GitHub repository'sini kontrol edin. 