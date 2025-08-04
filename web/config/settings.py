"""
RFID-I2C Web Sunucusu Konfigürasyon Dosyası
Tüm sistem ayarları bu dosyada merkezi olarak yönetilir.
"""

# Flask Konfigürasyonu
FLASK_CONFIG = {
    'SECRET_KEY': 'rfid_i2c_secret_key_2024',
    'HOST': '0.0.0.0',
    'PORT': 5000,
    'DEBUG': True
}

# MQTT Broker Ayarları
MQTT_CONFIG = {
    'BROKER': "nagatron-sdt.local",
    'PORT': 1883,
    'KEEPALIVE': 60
}

# RFID MQTT Konuları
RFID_TOPICS = {
    'READ_TOPIC': "rfid/data",
    'WRITE_TOPIC': "rfid/write", 
    'STATUS_TOPIC': "rfid/status"
}

# I2C MQTT Konuları
I2C_TOPICS = {
    'COMMAND_TOPIC': "i2c/command",
    'DATA_TOPIC': "i2c/data",
    'STATUS_TOPIC': "i2c/status",
    'MOTOR_DATA_TOPIC': "motor/data",
    'QTR_DATA_TOPIC': "qtr/data"
}

# Varsayılan RFID Veri Yapısı
DEFAULT_RFID_DATA = {
    'last_card_id': 'Henüz kart okunmadı',
    'last_read_time': '',
    'last_written_data': '',
    'last_write_time': '',
    'total_reads': 0,
    'total_writes': 0,
    'card_history': [],
    'write_history': []
}

# Varsayılan I2C Veri Yapısı
DEFAULT_I2C_DATA = {
    'pid_settings': {
        'kp': 2.0,
        'ki': 0.1,
        'kd': 0.5,
        'setpoint': 0
    },
    'pin_settings': {
        'motor_left_pin1': 5,
        'motor_left_pin2': 6,
        'motor_right_pin1': 9,
        'motor_right_pin2': 10,
        'enable_left': 3,
        'enable_right': 11
    },
    'speed_control': {
        'base_speed': 150,
        'max_speed': 255,
        'min_speed': 50
    },
    'motor_data': {
        'left_speed': 0,
        'right_speed': 0,
        'direction': 'stop',
        'last_update': ''
    },
    'qtr_data': {
        'sensors': [0, 0, 0, 0, 0, 0, 0, 0],  # 8 sensör
        'position': 0,
        'line_detected': False,
        'last_update': ''
    },
    'system_status': {
        'i2c_connected': False,
        'motor_active': False,
        'qtr_active': False,
        'last_ping': ''
    }
}

# Genel Ayarlar
GENERAL_CONFIG = {
    'MAX_HISTORY_SIZE': 10,
    'CORS_ALLOWED_ORIGINS': "*",
    'DATE_FORMAT': "%Y-%m-%d %H:%M:%S"
}