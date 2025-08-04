# websocket/events.py
"""
WebSocket Event Handler'ları
Tüm WebSocket event'leri bu modülde yönetilir.
"""

from flask_socketio import emit
from datetime import datetime
from data.rfid_data import rfid_manager
from data.i2c_data import i2c_manager
from mqtt.client import get_mqtt_client
from config.settings import GENERAL_CONFIG

class WebSocketEventHandler:
    def __init__(self, socketio):
        self.socketio = socketio
        self.register_events()
    
    def register_events(self):
        """Tüm WebSocket event'lerini kaydet"""
        
        # Bağlantı event'leri
        @self.socketio.on('connect')
        def handle_connect():
            print('WebSocket client bağlandı')
            # Bağlanan client'a mevcut verileri gönder
            emit('rfid_update', rfid_manager.get_data())
            emit('i2c_update', i2c_manager.get_data())
            emit('connection_status', {'status': 'connected'})
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            print('WebSocket client bağlantısı kesildi')
        
        # RFID WebSocket event'leri
        @self.socketio.on('write_rfid')
        def handle_write_rfid(data):
            self._handle_rfid_write(data)
        
        @self.socketio.on('get_rfid_data')
        def handle_get_rfid_data():
            emit('rfid_update', rfid_manager.get_data())
        
        @self.socketio.on('clear_rfid_history')
        def handle_clear_rfid_history(data):
            self._handle_clear_rfid_history(data)
        
        # I2C WebSocket event'leri
        @self.socketio.on('update_pid_settings')
        def handle_update_pid_settings(data):
            self._handle_pid_update(data)
        
        @self.socketio.on('update_pin_settings')
        def handle_update_pin_settings(data):
            self._handle_pin_update(data)
        
        @self.socketio.on('update_speed_settings')
        def handle_update_speed_settings(data):
            self._handle_speed_update(data)
        
        @self.socketio.on('motor_command')
        def handle_motor_command(data):
            self._handle_motor_command(data)
        
        @self.socketio.on('get_i2c_data')
        def handle_get_i2c_data():
            emit('i2c_update', i2c_manager.get_data())
        
        # Genel sistem event'leri
        @self.socketio.on('get_system_status')
        def handle_get_system_status():
            self._handle_system_status_request()
        
        @self.socketio.on('ping')
        def handle_ping():
            emit('pong', {'timestamp': i2c_manager.create_command('ping')['timestamp']})
    
    def _handle_rfid_write(self, data):
        """RFID yazma event'ini işle"""
        try:
            card_id = data.get('card_id', '')
            write_data = data.get('data', '')
            
            if not write_data:
                emit('write_error', {'message': 'Yazılacak veri boş olamaz'})
                return
            
            mqtt_client = get_mqtt_client()
            
            if not mqtt_client.is_connected():
                emit('write_error', {'message': 'MQTT bağlantısı yok'})
                return
            
            success = mqtt_client.publish_rfid_write(card_id, write_data)
            
            if success:
                emit('write_sent', {'message': 'Yazma komutu gönderildi'})
            else:
                emit('write_error', {'message': 'Yazma komutu gönderilemedi'})
            
        except Exception as e:
            emit('write_error', {'message': f'Hata: {str(e)}'})
    
    def _handle_clear_rfid_history(self, data):
        """RFID geçmişi temizleme event'ini işle"""
        try:
            history_type = data.get('type', None)
            rfid_manager.clear_history(history_type)
            
            message = f"{history_type.title() if history_type else 'Tüm'} geçmiş temizlendi"
            
            emit('history_cleared', {
                'message': message,
                'type': history_type
            })
            emit('rfid_update', rfid_manager.get_data())
            
        except Exception as e:
            emit('history_clear_error', {'message': f'Hata: {str(e)}'})
    
    def _handle_pid_update(self, data):
        """PID ayarları güncelleme event'ini işle"""
        try:
            success = i2c_manager.update_pid_settings(data)
            
            if not success:
                emit('pid_update_error', {'message': 'PID ayarları güncellenemedi'})
                return
            
            mqtt_client = get_mqtt_client()
            
            if not mqtt_client.is_connected():
                emit('pid_update_error', {'message': 'MQTT bağlantısı yok'})
                return
            
            command_success = mqtt_client.publish_i2c_command('pid_update', i2c_manager.data['pid_settings'])
            
            if command_success:
                emit('pid_update_success', {'message': 'PID ayarları güncellendi'})
                emit('i2c_update', i2c_manager.get_data())
            else:
                emit('pid_update_error', {'message': 'PID komutu gönderilemedi'})
            
        except Exception as e:
            emit('pid_update_error', {'message': f'Hata: {str(e)}'})
    
    def _handle_pin_update(self, data):
        """Pin ayarları güncelleme event'ini işle"""
        try:
            success = i2c_manager.update_pin_settings(data)
            
            if not success:
                emit('pin_update_error', {'message': 'Pin ayarları güncellenemedi'})
                return
            
            mqtt_client = get_mqtt_client()
            
            if not mqtt_client.is_connected():
                emit('pin_update_error', {'message': 'MQTT bağlantısı yok'})
                return
            
            command_success = mqtt_client.publish_i2c_command('pin_update', i2c_manager.data['pin_settings'])
            
            if command_success:
                emit('pin_update_success', {'message': 'Pin ayarları güncellendi'})
                emit('i2c_update', i2c_manager.get_data())
            else:
                emit('pin_update_error', {'message': 'Pin komutu gönderilemedi'})
            
        except Exception as e:
            emit('pin_update_error', {'message': f'Hata: {str(e)}'})
    
    def _handle_speed_update(self, data):
        """Hız ayarları güncelleme event'ini işle"""
        try:
            success = i2c_manager.update_speed_settings(data)
            
            if not success:
                emit('speed_update_error', {'message': 'Hız ayarları güncellenemedi'})
                return
            
            mqtt_client = get_mqtt_client()
            
            if not mqtt_client.is_connected():
                emit('speed_update_error', {'message': 'MQTT bağlantısı yok'})
                return
            
            command_success = mqtt_client.publish_i2c_command('speed_update', i2c_manager.data['speed_control'])
            
            if command_success:
                emit('speed_update_success', {'message': 'Hız ayarları güncellendi'})
                emit('i2c_update', i2c_manager.get_data())
            else:
                emit('speed_update_error', {'message': 'Hız komutu gönderilemedi'})
            
        except Exception as e:
            emit('speed_update_error', {'message': f'Hata: {str(e)}'})
    
    def _handle_motor_command(self, data):
        """Motor komut event'ini işle"""
        try:
            action = data.get('action', 'stop')
            speed = data.get('speed', i2c_manager.data['speed_control']['base_speed'])
            
            mqtt_client = get_mqtt_client()
            
            if not mqtt_client.is_connected():
                emit('motor_command_error', {'message': 'MQTT bağlantısı yok'})
                return
            
            success = mqtt_client.publish_motor_control(action, speed)
            
            if success:
                emit('motor_command_sent', {
                    'message': f"Motor komutu gönderildi: {action}",
                    'action': action,
                    'speed': speed
                })
            else:
                emit('motor_command_error', {'message': 'Motor komutu gönderilemedi'})
            
        except Exception as e:
            emit('motor_command_error', {'message': f'Hata: {str(e)}'})
    
    def _handle_system_status_request(self):
        """Sistem durumu talep event'ini işle"""
        try:
            mqtt_client = get_mqtt_client()
            
            status = {
                'mqtt_connected': mqtt_client.is_connected(),
                'i2c_status': i2c_manager.get_system_status(),
                'rfid_stats': rfid_manager.get_statistics(),
                'system_healthy': i2c_manager.is_system_healthy()
            }
            
            emit('system_status', status)
            
        except Exception as e:
            emit('system_status_error', {'message': f'Hata: {str(e)}'})

# WebSocket event handler'ı başlatmak için fonksiyon
def initialize_websocket_events(socketio):
    """WebSocket event handler'ını başlat"""
    return WebSocketEventHandler(socketio)