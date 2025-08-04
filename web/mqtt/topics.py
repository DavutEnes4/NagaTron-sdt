"""
MQTT Konu Tanımları
Tüm MQTT konuları bu modülde merkezi olarak yönetilir.
"""

from config.settings import RFID_TOPICS, I2C_TOPICS

class MQTTTopics:
    """MQTT konu yönetimi sınıfı"""
    
    # RFID konuları
    RFID_READ = RFID_TOPICS['READ_TOPIC']
    RFID_WRITE = RFID_TOPICS['WRITE_TOPIC']  
    RFID_STATUS = RFID_TOPICS['STATUS_TOPIC']
    
    # I2C konuları
    I2C_COMMAND = I2C_TOPICS['COMMAND_TOPIC']
    I2C_DATA = I2C_TOPICS['DATA_TOPIC']
    I2C_STATUS = I2C_TOPICS['STATUS_TOPIC']
    I2C_MOTOR_DATA = I2C_TOPICS['MOTOR_DATA_TOPIC']
    I2C_QTR_DATA = I2C_TOPICS['QTR_DATA_TOPIC']
    
    @classmethod
    def get_all_topics(cls):
        """Tüm konuları liste olarak döndür"""
        return [
            cls.RFID_READ,
            cls.RFID_WRITE,
            cls.RFID_STATUS,
            cls.I2C_COMMAND,
            cls.I2C_DATA,
            cls.I2C_STATUS,
            cls.I2C_MOTOR_DATA,
            cls.I2C_QTR_DATA
        ]
    
    @classmethod
    def get_rfid_topics(cls):
        """Sadece RFID konularını döndür"""
        return [
            cls.RFID_READ,
            cls.RFID_WRITE,
            cls.RFID_STATUS
        ]
    
    @classmethod
    def get_i2c_topics(cls):
        """Sadece I2C konularını döndür"""
        return [
            cls.I2C_COMMAND,
            cls.I2C_DATA,
            cls.I2C_STATUS,
            cls.I2C_MOTOR_DATA,
            cls.I2C_QTR_DATA
        ]
    
    @classmethod
    def is_rfid_topic(cls, topic):
        """Verilen konu RFID konusu mu kontrol et"""
        return topic in cls.get_rfid_topics()
    
    @classmethod
    def is_i2c_topic(cls, topic):
        """Verilen konu I2C konusu mu kontrol et"""
        return topic in cls.get_i2c_topics()
    
    @classmethod
    def get_topic_info(cls, topic):
        """Konu hakkında bilgi döndür"""
        topic_info = {
            cls.RFID_READ: {'type': 'RFID', 'direction': 'IN', 'description': 'RFID kart okuma verileri'},
            cls.RFID_WRITE: {'type': 'RFID', 'direction': 'OUT', 'description': 'RFID kart yazma komutları'},
            cls.RFID_STATUS: {'type': 'RFID', 'direction': 'IN', 'description': 'RFID yazma durumu'},
            cls.I2C_COMMAND: {'type': 'I2C', 'direction': 'OUT', 'description': 'I2C komutları'},
            cls.I2C_DATA: {'type': 'I2C', 'direction': 'IN', 'description': 'Genel I2C verileri'},
            cls.I2C_STATUS: {'type': 'I2C', 'direction': 'IN', 'description': 'I2C durum bilgileri'},
            cls.I2C_MOTOR_DATA: {'type': 'I2C', 'direction': 'IN', 'description': 'Motor verileri'},
            cls.I2C_QTR_DATA: {'type': 'I2C', 'direction': 'IN', 'description': 'QTR sensör verileri'}
        }
        
        return topic_info.get(topic, {'type': 'UNKNOWN', 'direction': 'UNKNOWN', 'description': 'Bilinmeyen konu'})