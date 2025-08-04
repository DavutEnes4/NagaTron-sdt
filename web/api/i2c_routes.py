"""
I2C/Motor API Endpoint'leri
I2C, Motor ve QTR sensör ile ilgili tüm HTTP API endpoint'leri bu modülde tanımlanır.
"""

from flask import Blueprint, jsonify, request
from data.i2c_data import i2c_manager
from mqtt.client import get_mqtt_client

# Blueprint oluştur
i2c_bp = Blueprint('i2c', __name__, url_prefix='/api')

@i2c_bp.route('/i2c-data', methods=['GET'])
def get_i2c_data():
    """Tüm I2C verilerini döndür"""
    try:
        return jsonify(i2c_manager.get_data())
    except Exception as e:
        return jsonify({'error': f'Veri alınırken hata oluştu: {str(e)}'}), 500

@i2c_bp.route('/system-status', methods=['GET'])
def get_system_status():
    """Sistem durumunu döndür"""
    try:
        status = i2c_manager.get_system_status()
        status['healthy'] = i2c_manager.is_system_healthy()
        return jsonify(status)
    except Exception as e:
        return jsonify({'error': f'Durum alınırken hata oluştu: {str(e)}'}), 500

@i2c_bp.route('/update-pid', methods=['POST'])
def update_pid():
    """PID ayarlarını güncelle"""
    try:
        data = request.get_json()
        
        if not data:
            return jsonify({'success': False, 'message': 'JSON verisi bulunamadı'}), 400
        
        # Veri yöneticisinde güncelle
        success = i2c_manager.update_pid_settings(data)
        
        if not success:
            return jsonify({'success': False, 'message': 'PID ayarları güncellenemedi'}), 500
        
        # MQTT ile Deneyap'a gönder
        mqtt_client = get_mqtt_client()
        
        if not mqtt_client.is_connected():
            return jsonify({'success': False, 'message': 'MQTT bağlantısı yok'}), 503
        
        command_success = mqtt_client.publish_i2c_command('pid_update', i2c_manager.data['pid_settings'])
        
        if command_success:
            return jsonify({
                'success': True, 
                'message': 'PID ayarları güncellendi',
                'data': i2c_manager.data['pid_settings']
            })
        else:
            return jsonify({'success': False, 'message': 'PID komutu gönderilemedi'}), 500
        
    except Exception as e:
        return jsonify({'success': False, 'message': f'Hata: {str(e)}'}), 500

@i2c_bp.route('/update-pins', methods=['POST'])
def update_pins():
    """Pin ayarlarını güncelle"""
    try:
        data = request.get_json()
        
        if not data:
            return jsonify({'success': False, 'message': 'JSON verisi bulunamadı'}), 400
        
        # Veri yöneticisinde güncelle
        success = i2c_manager.update_pin_settings(data)
        
        if not success:
            return jsonify({'success': False, 'message': 'Pin ayarları güncellenemedi'}), 500
        
        # MQTT ile Deneyap'a gönder
        mqtt_client = get_mqtt_client()
        
        if not mqtt_client.is_connected():
            return jsonify({'success': False, 'message': 'MQTT bağlantısı yok'}), 503
        
        command_success = mqtt_client.publish_i2c_command('pin_update', i2c_manager.data['pin_settings'])
        
        if command_success:
            return jsonify({
                'success': True, 
                'message': 'Pin ayarları güncellendi',
                'data': i2c_manager.data['pin_settings']
            })
        else:
            return jsonify({'success': False, 'message': 'Pin komutu gönderilemedi'}), 500
        
    except Exception as e:
        return jsonify({'success': False, 'message': f'Hata: {str(e)}'}), 500

@i2c_bp.route('/update-speed', methods=['POST'])
def update_speed():
    """Hız ayarlarını güncelle"""
    try:
        data = request.get_json()
        
        if not data:
            return jsonify({'success': False, 'message': 'JSON verisi bulunamadı'}), 400
        
        # Veri yöneticisinde güncelle
        success = i2c_manager.update_speed_settings(data)
        
        if not success:
            return jsonify({'success': False, 'message': 'Hız ayarları güncellenemedi'}), 500
        
        # MQTT ile Deneyap'a gönder
        mqtt_client = get_mqtt_client()
        
        if not mqtt_client.is_connected():
            return jsonify({'success': False, 'message': 'MQTT bağlantısı yok'}), 503
        
        command_success = mqtt_client.publish_i2c_command('speed_update', i2c_manager.data['speed_control'])
        
        if command_success:
            return jsonify({
                'success': True, 
                'message': 'Hız ayarları güncellendi',
                'data': i2c_manager.data['speed_control']
            })
        else:
            return jsonify({'success': False, 'message': 'Hız komutu gönderilemedi'}), 500
        
    except Exception as e:
        return jsonify({'success': False, 'message': f'Hata: {str(e)}'}), 500

@i2c_bp.route('/motor-control', methods=['POST'])
def motor_control():
    """Motor kontrolü"""
    try:
        data = request.get_json()
        
        if not data:
            return jsonify({'success': False, 'message': 'JSON verisi bulunamadı'}), 400
        
        action = data.get('action', 'stop')  # start, stop, forward, backward, left, right
        speed = data.get('speed', i2c_manager.data['speed_control']['base_speed'])
        
        # MQTT ile motor kontrol komutu gönder
        mqtt_client = get_mqtt_client()
        
        if not mqtt_client.is_connected():
            return jsonify({'success': False, 'message': 'MQTT bağlantısı yok'}), 503
        
        success = mqtt_client.publish_motor_control(action, speed)
        
        if success:
            return jsonify({
                'success': True, 
                'message': f'Motor komutu gönderildi: {action}',
                'action': action,
                'speed': speed
            })
        else:
            return jsonify({'success': False, 'message': 'Motor komutu gönderilemedi'}), 500
        
    except Exception as e:
        return jsonify({'success': False, 'message': f'Hata: {str(e)}'}), 500

@i2c_bp.route('/motor-status', methods=['GET'])
def get_motor_status():
    """Motor durumunu döndür"""
    try:
        return jsonify(i2c_manager.get_motor_status())
    except Exception as e:
        return jsonify({'error': f'Motor durumu alınırken hata oluştu: {str(e)}'}), 500

@i2c_bp.route('/qtr-status', methods=['GET'])
def get_qtr_status():
    """QTR sensör durumunu döndür"""
    try:
        return jsonify(i2c_manager.get_qtr_status())
    except Exception as e:
        return jsonify({'error': f'QTR durumu alınırken hata oluştu: {str(e)}'}), 500

@i2c_bp.route('/reset-i2c-data', methods=['POST'])
def reset_i2c_data():
    """I2C verilerini sıfırla"""
    try:
        i2c_manager.reset_data()
        
        return jsonify({
            'success': True,
            'message': 'I2C verileri sıfırlandı',
            'data': i2c_manager.get_data()
        })
        
    except Exception as e:
        return jsonify({'success': False, 'message': f'Hata: {str(e)}'}), 500

@i2c_bp.route('/emergency-stop', methods=['POST'])
def emergency_stop():
    """Acil durum - tüm motorları durdur"""
    try:
        mqtt_client = get_mqtt_client()
        
        if not mqtt_client.is_connected():
            return jsonify({'success': False, 'message': 'MQTT bağlantısı yok'}), 503
        
        success = mqtt_client.publish_motor_control('emergency_stop', 0)
        
        if success:
            return jsonify({
                'success': True, 
                'message': 'Acil durum komutu gönderildi - motorlar durduruldu'
            })
        else:
            return jsonify({'success': False, 'message': 'Acil durum komutu gönderilemedi'}), 500
        
    except Exception as e:
        return jsonify({'success': False, 'message': f'Hata: {str(e)}'}), 500