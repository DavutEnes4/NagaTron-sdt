"""
RFID-I2C Web Sunucusu - Ana Uygulama Dosyası
Modüler yapı ile yeniden organize edilmiş Flask uygulaması.
"""

from flask import Flask
from flask_socketio import SocketIO

# Konfigürasyon
from config.settings import FLASK_CONFIG

# API Blueprint'leri
from api.general_routes import general_bp
from api.rfid_routes import rfid_bp
from api.i2c_routes import i2c_bp

# MQTT ve WebSocket
from mqtt.client import initialize_mqtt
from websocket.events import initialize_websocket_events

def create_app():
    """Flask uygulamasını oluştur ve yapılandır"""
    
    # Flask uygulamasını oluştur
    app = Flask(__name__)
    app.config['SECRET_KEY'] = FLASK_CONFIG['SECRET_KEY']
    
    # SocketIO'yu başlat
    socketio = SocketIO(app, cors_allowed_origins="*")
    
    # Blueprint'leri kaydet
    app.register_blueprint(general_bp)
    app.register_blueprint(rfid_bp)
    app.register_blueprint(i2c_bp)
    
    # WebSocket event handler'larını başlat
    initialize_websocket_events(socketio)
    
    # MQTT'yi başlat
    mqtt_success = initialize_mqtt(socketio)
    if mqtt_success:
        print("✅ MQTT başarıyla başlatıldı")
    else:
        print("❌ MQTT başlatılamadı")
    
    return app, socketio

def main():
    """Ana uygulama fonksiyonu"""
    print("🚀 RFID-I2C Web Sunucusu başlatılıyor...")
    print("📁 Modüler yapı yükleniyor...")
    
    # Uygulamayı oluştur
    app, socketio = create_app()
    
    # Başlangıç bilgileri
    print(f"🌐 Server adresi: http://{FLASK_CONFIG['HOST']}:{FLASK_CONFIG['PORT']}")
    print("📡 MQTT bağlantısı kontrol ediliyor...")
    print("🔧 API endpoint'leri hazır:")
    print("   - /api/rfid-data")
    print("   - /api/i2c-data") 
    print("   - /api/system-info")
    print("   - /api/health")
    print("🔌 WebSocket bağlantıları kabul ediliyor...")
    print("✨ Sistem hazır!")
    
    # Sunucuyu başlat
    socketio.run(
        app, 
        host=FLASK_CONFIG['HOST'], 
        port=FLASK_CONFIG['PORT'], 
        debug=FLASK_CONFIG['DEBUG']
    )

if __name__ == '__main__':
    main()