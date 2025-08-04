"""
Genel API Endpoint'leri
Genel sistem endpoint'leri bu modülde tanımlanır.
"""

from flask import Blueprint, jsonify, render_template, request
from mqtt.client import get_mqtt_client
from data.rfid_data import rfid_manager
from data.i2c_data import i2c_manager
from utils.helpers import create_response, log_error, log_info

# Blueprint oluştur
general_bp = Blueprint('general', __name__)

@general_bp.route('/')
def index():
    """Ana sayfa"""
    try:
        return render_template('index.html')
    except Exception as e:
        log_error(f"Ana sayfa yüklenirken hata: {e}", "GENERAL_ROUTES")
        return jsonify(create_response(False, "Ana sayfa yüklenemedi", error=str(e))), 500

@general_bp.route('/api/mqtt-status')
def mqtt_status():
    """MQTT bağlantı durumunu döndür"""
    try:
        mqtt_client = get_mqtt_client()
        status = {
            'connected': mqtt_client.is_connected(),
            'broker': 'nagatron-sdt.local',
            'port': 1883,
            'timestamp': mqtt_client.create_command('status')['timestamp'] if hasattr(mqtt_client, 'create_command') else None
        }
        
        log_info(f"MQTT durumu sorgulandı: {status['connected']}", "GENERAL_ROUTES")
        return jsonify(status)
        
    except Exception as e:
        log_error(f"MQTT durumu alınırken hata: {e}", "GENERAL_ROUTES")
        return jsonify(create_response(False, "MQTT durumu alınamadı", error=str(e))), 500

@general_bp.route('/api/system-info')
def system_info():
    """Sistem bilgilerini döndür"""
    try:
        mqtt_client = get_mqtt_client()
        
        system_info = {
            'system': {
                'name': 'RFID-I2C Web Sunucusu',
                'version': '2.0.0',
                'status': 'running',
                'uptime': 'active'
            },
            'mqtt': {
                'connected': mqtt_client.is_connected(),
                'broker': 'nagatron-sdt.local',
                'last_activity': mqtt_client.create_command('ping')['timestamp'] if hasattr(mqtt_client, 'create_command') else None
            },
            'rfid': {
                'total_reads': rfid_manager.data['total_reads'],
                'total_writes': rfid_manager.data['total_writes'],
                'last_activity': rfid_manager.data.get('last_read_time', 'Henüz aktivite yok'),
                'status': 'active'
            },
            'i2c': {
                'connected': i2c_manager.data['system_status']['i2c_connected'],
                'motor_active': i2c_manager.data['system_status']['motor_active'],
                'qtr_active': i2c_manager.data['system_status']['qtr_active'],
                'healthy': i2c_manager.is_system_healthy(),
                'last_ping': i2c_manager.data['system_status']['last_ping']
            }
        }
        
        log_info("Sistem bilgileri sorgulandı", "GENERAL_ROUTES")
        return jsonify(system_info)
        
    except Exception as e:
        log_error(f"Sistem bilgisi alınırken hata: {e}", "GENERAL_ROUTES")
        return jsonify(create_response(False, "Sistem bilgisi alınamadı", error=str(e))), 500

@general_bp.route('/api/health')
def health_check():
    """Sistem sağlık kontrolü"""
    try:
        mqtt_client = get_mqtt_client()
        
        health_checks = {
            'mqtt_connection': mqtt_client.is_connected(),
            'i2c_system': i2c_manager.is_system_healthy(),
            'rfid_system': True,  # RFID için basit kontrol
            'web_server': True
        }
        
        # Sağlık durumunu belirle
        all_healthy = all(health_checks.values())
        status = 'healthy' if all_healthy else 'unhealthy'
        
        health_status = {
            'status': status,
            'timestamp': mqtt_client.create_command('health_check')['timestamp'] if hasattr(mqtt_client, 'create_command') else None,
            'checks': health_checks,
            'details': {
                'mqtt_broker': 'nagatron-sdt.local:1883',
                'i2c_timeout': '30s',
                'rfid_status': 'active'
            }
        }
        
        status_code = 200 if all_healthy else 503
        log_info(f"Sağlık kontrolü: {status}", "GENERAL_ROUTES")
        
        return jsonify(health_status), status_code
        
    except Exception as e:
        log_error(f"Sağlık kontrolü hatası: {e}", "GENERAL_ROUTES")
        return jsonify(create_response(False, "Sağlık kontrolü yapılamadı", error=str(e))), 500

@general_bp.route('/api/stats')
def get_all_stats():
    """Tüm sistem istatistiklerini döndür"""
    try:
        mqtt_client = get_mqtt_client() # mqtt_client'i burada tanımla
        stats = {
            'rfid_stats': rfid_manager.get_statistics(),
            'i2c_stats': {
                'system_status': i2c_manager.get_system_status(),
                'motor_status': i2c_manager.get_motor_status(),
                'qtr_status': i2c_manager.get_qtr_status(),
                'healthy': i2c_manager.is_system_healthy()
            },
            'mqtt_stats': {
                'connected': get_mqtt_client().is_connected(),
                'broker': 'nagatron-sdt.local'
            },
            'timestamp': mqtt_client.create_command('stats')['timestamp'] if hasattr(mqtt_client, 'create_command') else None
        }
        
        log_info("Sistem istatistikleri sorgulandı", "GENERAL_ROUTES")
        return jsonify(stats)
        
    except Exception as e:
        log_error(f"İstatistikler alınırken hata: {e}", "GENERAL_ROUTES")
        return jsonify(create_response(False, "İstatistikler alınamadı", error=str(e))), 500

@general_bp.route('/api/reset-all', methods=['POST'])
def reset_all_data():
    """Tüm sistem verilerini sıfırla"""
    try:
        # Güvenlik kontrolü - sadece POST isteklerini kabul et
        if request.method != 'POST':
            return jsonify(create_response(False, "Sadece POST metodu kabul edilir")), 405
        
        # Verileri sıfırla
        rfid_manager.reset_data()
        i2c_manager.reset_data()
        
        log_info("Tüm sistem verileri sıfırlandı", "GENERAL_ROUTES")
        
        return jsonify(create_response(
            True, 
            "Tüm sistem verileri sıfırlandı",
            data={
                'rfid_data': rfid_manager.get_data(),
                'i2c_data': i2c_manager.get_data()
            }
        ))
        
    except Exception as e:
        log_error(f"Veri sıfırlama hatası: {e}", "GENERAL_ROUTES")
        return jsonify(create_response(False, "Veriler sıfırlanamadı", error=str(e))), 500

@general_bp.route('/api/ping', methods=['GET', 'POST'])
def ping():
    """Basit ping endpoint'i"""
    try:
        mqtt_client = get_mqtt_client() # mqtt_client'i burada tanımla
        return jsonify(create_response(True, "pong", data={
            'timestamp': mqtt_client.create_command('ping')['timestamp'] if hasattr(mqtt_client, 'create_command') else None,
            'server': 'RFID-I2C Web Server'
        }))
    except Exception as e:
        return jsonify(create_response(False, "Ping başarısız", error=str(e))), 500