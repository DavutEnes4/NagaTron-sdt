"""
I2C/Motor Veri Yönetimi Modülü
I2C, Motor ve QTR sensör verileri bu modülde yönetilir.
"""

from datetime import datetime
import json
from config.settings import DEFAULT_I2C_DATA, GENERAL_CONFIG

class I2CDataManager:
    def __init__(self):
        self.data = DEFAULT_I2C_DATA.copy()
    
    def get_data(self):
        """Tüm I2C verilerini döndür"""
        return self.data
    
    def update_pid_settings(self, new_settings):
        """PID ayarlarını güncelle"""
        try:
            self.data['pid_settings'].update({
                'kp': float(new_settings.get('kp', self.data['pid_settings']['kp'])),
                'ki': float(new_settings.get('ki', self.data['pid_settings']['ki'])),
                'kd': float(new_settings.get('kd', self.data['pid_settings']['kd'])),
                'setpoint': float(new_settings.get('setpoint', self.data['pid_settings']['setpoint']))
            })
            return True
        except Exception as e:
            print(f"PID ayarları güncelleme hatası: {e}")
            return False
    
    def update_pin_settings(self, new_settings):
        """Pin ayarlarını güncelle"""
        try:
            self.data['pin_settings'].update({
                'motor_left_pin1': int(new_settings.get('motor_left_pin1', self.data['pin_settings']['motor_left_pin1'])),
                'motor_left_pin2': int(new_settings.get('motor_left_pin2', self.data['pin_settings']['motor_left_pin2'])),
                'motor_right_pin1': int(new_settings.get('motor_right_pin1', self.data['pin_settings']['motor_right_pin1'])),
                'motor_right_pin2': int(new_settings.get('motor_right_pin2', self.data['pin_settings']['motor_right_pin2'])),
                'enable_left': int(new_settings.get('enable_left', self.data['pin_settings']['enable_left'])),
                'enable_right': int(new_settings.get('enable_right', self.data['pin_settings']['enable_right']))
            })
            return True
        except Exception as e:
            print(f"Pin ayarları güncelleme hatası: {e}")
            return False
    
    def update_speed_settings(self, new_settings):
        """Hız ayarlarını güncelle"""
        try:
            self.data['speed_control'].update({
                'base_speed': int(new_settings.get('base_speed', self.data['speed_control']['base_speed'])),
                'max_speed': int(new_settings.get('max_speed', self.data['speed_control']['max_speed'])),
                'min_speed': int(new_settings.get('min_speed', self.data['speed_control']['min_speed']))
            })
            return True
        except Exception as e:
            print(f"Hız ayarları güncelleme hatası: {e}")
            return False
    
    def update_i2c_status(self, message):
        """I2C durum bilgilerini güncelle"""
        try:
            data = json.loads(message)
            current_time = datetime.now().strftime(GENERAL_CONFIG['DATE_FORMAT'])
            
            self.data['system_status'].update({
                'i2c_connected': data.get('connected', False),
                'motor_active': data.get('motor_active', False),
                'qtr_active': data.get('qtr_active', False),
                'last_ping': current_time
            })
            
            return True
            
        except Exception as e:
            print(f"I2C durum güncelleme hatası: {e}")
            return False
    
    def update_motor_data(self, message):
        """Motor verilerini güncelle"""
        try:
            data = json.loads(message)
            current_time = datetime.now().strftime(GENERAL_CONFIG['DATE_FORMAT'])
            
            self.data['motor_data'].update({
                'left_speed': data.get('left_speed', 0),
                'right_speed': data.get('right_speed', 0),
                'direction': data.get('direction', 'stop'),
                'last_update': current_time
            })
            
            # Sistem durumunu güncelle
            self.data['system_status']['motor_active'] = True
            self.data['system_status']['last_ping'] = current_time
            
            return True
            
        except Exception as e:
            print(f"Motor verisi güncelleme hatası: {e}")
            return False
    
    def update_qtr_data(self, message):
        """QTR sensör verilerini güncelle"""
        try:
            data = json.loads(message)
            current_time = datetime.now().strftime(GENERAL_CONFIG['DATE_FORMAT'])
            
            self.data['qtr_data'].update({
                'sensors': data.get('sensors', [0]*8),
                'position': data.get('position', 0),
                'line_detected': data.get('line_detected', False),
                'last_update': current_time
            })
            
            # Sistem durumunu güncelle
            self.data['system_status']['qtr_active'] = True
            self.data['system_status']['last_ping'] = current_time
            
            return True
            
        except Exception as e:
            print(f"QTR verisi güncelleme hatası: {e}")
            return False
    
    def update_general_i2c_data(self, message):
        """Genel I2C verilerini güncelle"""
        try:
            current_time = datetime.now().strftime(GENERAL_CONFIG['DATE_FORMAT'])
            
            self.data['system_status']['i2c_connected'] = True
            self.data['system_status']['last_ping'] = current_time
            
            return True
            
        except Exception as e:
            print(f"Genel I2C verisi güncelleme hatası: {e}")
            return False
    
    def get_system_status(self):
        """Sistem durumunu döndür"""
        return self.data['system_status']
    
    def get_motor_status(self):
        """Motor durumunu döndür"""
        return self.data['motor_data']
    
    def get_qtr_status(self):
        """QTR sensör durumunu döndür"""
        return self.data['qtr_data']
    
    def create_command(self, command_type, data=None):
        """MQTT komutu oluştur"""
        command = {
            'type': command_type,
            'timestamp': datetime.now().strftime(GENERAL_CONFIG['DATE_FORMAT'])
        }
        
        if data:
            command['data'] = data
            
        return command
    
    def is_system_healthy(self):
        """Sistem sağlığını kontrol et"""
        status = self.data['system_status']
        
        # Son ping zamanını kontrol et
        if not status['last_ping']:
            return False
        
        try:
            last_ping = datetime.strptime(status['last_ping'], GENERAL_CONFIG['DATE_FORMAT'])
            current_time = datetime.now()
            time_diff = (current_time - last_ping).total_seconds()
            
            # 30 saniyeden fazla ping gelmemişse sağlıksız kabul et
            return time_diff < 30
            
        except:
            return False
    
    def reset_data(self):
        """Tüm verileri sıfırla"""
        self.data = DEFAULT_I2C_DATA.copy()

# Global I2C veri yöneticisi
i2c_manager = I2CDataManager()