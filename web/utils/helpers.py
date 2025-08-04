# utils/helpers.py
"""
Yardımcı Fonksiyonlar
Genel amaçlı yardımcı fonksiyonlar bu modülde tanımlanır.
"""

import json
from datetime import datetime
from config.settings import GENERAL_CONFIG

def get_current_timestamp():
    """Mevcut zamanı formatlanmış string olarak döndür"""
    return datetime.now().strftime(GENERAL_CONFIG['DATE_FORMAT'])

def safe_json_loads(data, default=None):
    """Güvenli JSON parse işlemi"""
    try:
        return json.loads(data)
    except (json.JSONDecodeError, TypeError):
        return default or {}

def safe_json_dumps(data, default=None):
    """Güvenli JSON serialize işlemi"""
    try:
        return json.dumps(data, ensure_ascii=False, indent=2)
    except (TypeError, ValueError):
        return default or "{}"

def validate_card_id(card_id):
    """Kart ID'sini doğrula"""
    if not card_id or not isinstance(card_id, str):
        return False
    
    # Basit doğrulama kuralları
    if len(card_id) < 1 or len(card_id) > 50:
        return False
    
    return True

def validate_speed_value(speed):
    """Hız değerini doğrula"""
    try:
        speed_int = int(speed)
        return 0 <= speed_int <= 255
    except (ValueError, TypeError):
        return False

def validate_pin_number(pin):
    """Pin numarasını doğrula"""
    try:
        pin_int = int(pin)
        return 0 <= pin_int <= 53  # Arduino pin aralığı
    except (ValueError, TypeError):
        return False

def validate_pid_value(value):
    """PID değerini doğrula"""
    try:
        float_val = float(value)
        return -1000 <= float_val <= 1000  # Makul PID aralığı
    except (ValueError, TypeError):
        return False

def format_time_ago(timestamp_str):
    """Zaman damgasını 'x dakika önce' formatında döndür"""
    try:
        timestamp = datetime.strptime(timestamp_str, GENERAL_CONFIG['DATE_FORMAT'])
        now = datetime.now()
        diff = now - timestamp
        
        if diff.days > 0:
            return f"{diff.days} gün önce"
        elif diff.seconds > 3600:
            hours = diff.seconds // 3600
            return f"{hours} saat önce"
        elif diff.seconds > 60:
            minutes = diff.seconds // 60
            return f"{minutes} dakika önce"
        else:
            return "Az önce"
            
    except (ValueError, TypeError):
        return "Bilinmeyen zaman"

def truncate_string(text, max_length=50):
    """Uzun metinleri kısalt"""
    if not text or len(text) <= max_length:
        return text
    
    return text[:max_length-3] + "..."

def sanitize_input(input_str, max_length=1000):
    """Kullanıcı girdisini temizle"""
    if not input_str:
        return ""
    
    # Temel temizlik
    cleaned = str(input_str).strip()
    
    # Uzunluk kontrolü
    if len(cleaned) > max_length:
        cleaned = cleaned[:max_length]
    
    return cleaned

def create_response(success=True, message="", data=None, error=None):
    """Standart API yanıt formatı oluştur"""
    response = {
        'success': success,
        'timestamp': get_current_timestamp()
    }
    
    if message:
        response['message'] = message
    
    if data is not None:
        response['data'] = data
    
    if error:
        response['error'] = error
        response['success'] = False
    
    return response

def log_error(error_msg, context="GENERAL"):
    """Hata logla"""
    timestamp = get_current_timestamp()
    print(f"[{timestamp}] [{context}] ERROR: {error_msg}")

def log_info(info_msg, context="GENERAL"):
    """Bilgi logla"""
    timestamp = get_current_timestamp()
    print(f"[{timestamp}] [{context}] INFO: {info_msg}")

def calculate_percentage(part, total):
    """Yüzde hesapla"""
    if total == 0:
        return 0.0
    return round((part / total) * 100, 2)

def is_system_command(command):
    """Sistem komutu mu kontrol et"""
    system_commands = [
        'emergency_stop',
        'system_reset',
        'factory_reset',
        'shutdown',
        'restart'
    ]
    return command in system_commands

def format_sensor_data(sensors):
    """Sensör verilerini formatla"""
    if not isinstance(sensors, list):
        return [0] * 8
    
    # 8 sensör olduğundan emin ol
    while len(sensors) < 8:
        sensors.append(0)
    
    return sensors[:8]  # Fazlasını kes

def get_motor_direction_from_speeds(left_speed, right_speed):
    """Motor hızlarından yön belirle"""
    if left_speed == 0 and right_speed == 0:
        return "stop"
    elif left_speed > 0 and right_speed > 0:
        return "forward"
    elif left_speed < 0 and right_speed < 0:
        return "backward"
    elif left_speed > right_speed:
        return "right"
    elif right_speed > left_speed:
        return "left"
    else:
        return "unknown"