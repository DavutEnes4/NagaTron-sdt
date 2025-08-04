# mqtt/client.py
"""
MQTT İstemci Modülü
MQTT bağlantısı ve temel operasyonlar bu modülde yönetilir.
"""

import paho.mqtt.client as mqtt
import threading
import json
import time
from config.settings import MQTT_CONFIG, RFID_TOPICS, I2C_TOPICS
from mqtt.handlers import MQTTMessageHandler
from utils.helpers import get_current_timestamp

class MQTTClient:
    def __init__(self, socketio=None):
        self.client = mqtt.Client()
        self.connected = False
        self.socketio = socketio
        self.handler = MQTTMessageHandler(socketio)
        self.connection_attempts = 0
        self.max_retries = 5
        
        # Callback'leri ayarla
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        self.client.on_connect_fail = self._on_connect_fail
        
    def _on_connect(self, client, userdata, flags, rc):
        """MQTT bağlantı callback'i"""
        if rc == 0:
            print(f"✅ MQTT Broker'a başarıyla bağlandı: {MQTT_CONFIG['BROKER']}:{MQTT_CONFIG['PORT']}")
            self.connected = True
            self.connection_attempts = 0
            
            # Tüm konuları dinle
            self._subscribe_to_topics()
        else:
            print(f"❌ MQTT bağlantı hatası. Kod: {rc}")
            self.connected = False
        
    def _on_disconnect(self, client, userdata, rc):
        """MQTT bağlantı kesme callback'i"""
        print(f"⚠️ MQTT Broker bağlantısı kesildi. Kod: {rc}")
        self.connected = False
        
        # Otomatik yeniden bağlanma
        if rc != 0 and self.connection_attempts < self.max_retries:
            print(f"🔄 {self.max_retries - self.connection_attempts} deneme kaldı. Yeniden bağlanılıyor...")
            time.sleep(2)
            self.connect()
        
    def _on_connect_fail(self, client, userdata):
        """MQTT bağlantı başarısızlık callback'i"""
        print(f"❌ MQTT bağlantı başarısız: {MQTT_CONFIG['BROKER']}")
        self.connected = False
        
    def _on_message(self, client, userdata, msg):
        """MQTT mesaj alma callback'i"""
        try:
            topic = msg.topic
            message = msg.payload.decode('utf-8')
            print(f"📨 Gelen veri - Topic: {topic}, Mesaj: {message}")
            
            # Mesajı uygun handler'a yönlendir
            self.handler.handle_message(topic, message)
            
        except Exception as e:
            print(f"❌ MQTT mesaj işleme hatası: {e}")
    
    def _subscribe_to_topics(self):
        """Tüm MQTT konularına abone ol"""
        try:
            # RFID konuları
            for topic_name, topic in RFID_TOPICS.items():
                result = self.client.subscribe(topic)
                if result[0] == mqtt.MQTT_ERR_SUCCESS:
                    print(f"✅ RFID konusuna abone olundu: {topic}")
                else:
                    print(f"❌ RFID konu aboneliği başarısız: {topic}")
            
            # I2C konuları
            for topic_name, topic in I2C_TOPICS.items():
                result = self.client.subscribe(topic)
                if result[0] == mqtt.MQTT_ERR_SUCCESS:
                    print(f"✅ I2C konusuna abone olundu: {topic}")
                else:
                    print(f"❌ I2C konu aboneliği başarısız: {topic}")
                    
        except Exception as e:
            print(f"❌ Konu aboneliği hatası: {e}")
    
    def connect(self):
        """MQTT broker'a bağlan"""
        try:
            self.connection_attempts += 1
            print(f"🔗 MQTT Broker'a bağlanılıyor: {MQTT_CONFIG['BROKER']}:{MQTT_CONFIG['PORT']}")
            
            self.client.connect(
                MQTT_CONFIG['BROKER'], 
                MQTT_CONFIG['PORT'], 
                MQTT_CONFIG['KEEPALIVE']
            )
            return True
        except Exception as e:
            print(f"❌ MQTT bağlantı hatası: {e}")
            return False
    
    def start_loop(self):
        """MQTT loop'unu başlat"""
        try:
            print("🔄 MQTT loop başlatılıyor...")
            self.client.loop_forever()
        except Exception as e:
            print(f"❌ MQTT loop hatası: {e}")
    
    def start_in_thread(self):
        """MQTT'yi ayrı thread'de başlat"""
        if self.connect():
            mqtt_thread = threading.Thread(target=self.start_loop, daemon=True)
            mqtt_thread.start()
            print("✅ MQTT thread başlatıldı")
            return True
        else:
            print("❌ MQTT thread başlatılamadı")
            return False
    
    def publish(self, topic, message):
        """MQTT mesajı yayınla"""
        try:
            if not self.connected:
                print(f"❌ MQTT bağlantısı yok, mesaj gönderilemedi: {topic}")
                return False
            
            if isinstance(message, dict):
                message = json.dumps(message)
            
            result = self.client.publish(topic, message)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                print(f"✅ Mesaj gönderildi - Topic: {topic}")
                return True
            else:
                print(f"❌ Mesaj gönderme hatası - Topic: {topic}, Hata: {result.rc}")
                return False
                
        except Exception as e:
            print(f"❌ MQTT publish hatası: {e}")
            return False
    
    def publish_rfid_write(self, card_id, data):
        """RFID yazma komutu gönder"""
        command = {
            'action': 'write',
            'card_id': card_id,
            'data': data,
            'timestamp': get_current_timestamp()
        }
        
        return self.publish(RFID_TOPICS['WRITE_TOPIC'], command)
    
    def publish_i2c_command(self, command_type, data=None):
        """I2C komutu gönder"""
        command = {
            'type': command_type,
            'timestamp': get_current_timestamp()
        }
        
        if data:
            command['data'] = data
            
        return self.publish(I2C_TOPICS['COMMAND_TOPIC'], command)
    
    def publish_motor_control(self, action, speed=None):
        """Motor kontrol komutu gönder"""
        command = {
            'type': 'motor_control',
            'action': action,
            'timestamp': get_current_timestamp()
        }
        
        if speed is not None:
            command['speed'] = int(speed)
            
        return self.publish(I2C_TOPICS['COMMAND_TOPIC'], command)
    
    def create_command(self, command_type):
        """MQTT komutu oluştur"""
        from datetime import datetime
        from config.settings import GENERAL_CONFIG
        
        command = {
            'type': command_type,
            'timestamp': datetime.now().strftime(GENERAL_CONFIG['DATE_FORMAT'])
        }
        
        return command
    
    def is_connected(self):
        """Bağlantı durumunu döndür"""
        return self.connected
    
    def disconnect(self):
        """MQTT bağlantısını kapat"""
        try:
            self.client.disconnect()
            self.connected = False
            print("✅ MQTT bağlantısı kapatıldı")
            return True
        except Exception as e:
            print(f"❌ MQTT bağlantı kapatma hatası: {e}")
            return False

# Global MQTT client instance'ı
mqtt_client_instance = None

def get_mqtt_client(socketio=None):
    """Global MQTT client'ı al veya oluştur"""
    global mqtt_client_instance
    if mqtt_client_instance is None:
        mqtt_client_instance = MQTTClient(socketio)
    elif socketio and not mqtt_client_instance.socketio:
        mqtt_client_instance.socketio = socketio
        mqtt_client_instance.handler.socketio = socketio
    return mqtt_client_instance

def initialize_mqtt(socketio):
    """MQTT'yi başlat"""
    client = get_mqtt_client(socketio)
    return client.start_in_thread()