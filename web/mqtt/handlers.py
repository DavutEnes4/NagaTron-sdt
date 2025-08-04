# mqtt/handlers.py
"""
MQTT Mesaj İşleyici Modülü
Gelen MQTT mesajlarını işleyip uygun veri yöneticilerine yönlendirir.
"""

from datetime import datetime
from config.settings import RFID_TOPICS, I2C_TOPICS, GENERAL_CONFIG
from data.rfid_data import rfid_manager
from data.i2c_data import i2c_manager
from utils.helpers import get_current_timestamp

class MQTTMessageHandler:
    def __init__(self, socketio=None):
        self.socketio = socketio
    
    def handle_message(self, topic, message):
        """Gelen MQTT mesajını konusuna göre uygun işleyiciye yönlendir"""
        
        # RFID mesajları
        if topic == RFID_TOPICS['READ_TOPIC']:
            self._handle_rfid_read(message)
        elif topic == RFID_TOPICS['STATUS_TOPIC']:
            self._handle_rfid_write_status(message)
        
        # I2C mesajları
        elif topic == I2C_TOPICS['DATA_TOPIC']:
            self._handle_i2c_data(message)
        elif topic == I2C_TOPICS['STATUS_TOPIC']:
            self._handle_i2c_status(message)
        elif topic == I2C_TOPICS['MOTOR_DATA_TOPIC']:
            self._handle_motor_data(message)
        elif topic == I2C_TOPICS['QTR_DATA_TOPIC']:
            self._handle_qtr_data(message)
        else:
            print(f"Bilinmeyen MQTT konusu: {topic}")
    
    def _handle_rfid_read(self, message):
        """RFID okuma verilerini işle"""
        success = rfid_manager.update_card_read(message)
        
        if success and self.socketio:
            # WebSocket ile istemcilere gönder
            self.socketio.emit('rfid_update', rfid_manager.get_data())
    
    def _handle_rfid_write_status(self, message):
        """RFID yazma durumunu işle"""
        result = rfid_manager.update_write_status(message)
        
        if result and self.socketio:
            # Yazma durumunu gönder
            self.socketio.emit('write_status', result)
            # Güncel RFID verilerini gönder
            self.socketio.emit('rfid_update', rfid_manager.get_data())
    
    def _handle_i2c_data(self, message):
        """Genel I2C verilerini işle"""
        success = i2c_manager.update_general_i2c_data(message)
        
        if success and self.socketio:
            self.socketio.emit('i2c_update', i2c_manager.get_data())
    
    def _handle_i2c_status(self, message):
        """I2C durum verilerini işle"""
        success = i2c_manager.update_i2c_status(message)
        
        if success and self.socketio:
            self.socketio.emit('i2c_status_update', i2c_manager.get_system_status())
    
    def _handle_motor_data(self, message):
        """Motor verilerini işle"""
        success = i2c_manager.update_motor_data(message)
        
        if success and self.socketio:
            self.socketio.emit('motor_update', i2c_manager.get_motor_status())
    
    def _handle_qtr_data(self, message):
        """QTR sensör verilerini işle"""
        success = i2c_manager.update_qtr_data(message)
        
        if success and self.socketio:
            self.socketio.emit('qtr_update', i2c_manager.get_qtr_status())