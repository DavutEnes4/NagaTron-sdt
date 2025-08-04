"""
RFID API Endpoint'leri
RFID ile ilgili tüm HTTP API endpoint'leri bu modülde tanımlanır.
"""

from flask import Blueprint, jsonify, request
from data.rfid_data import rfid_manager
from mqtt.client import get_mqtt_client

# Blueprint oluştur
rfid_bp = Blueprint('rfid', __name__, url_prefix='/api')

@rfid_bp.route('/rfid-data', methods=['GET'])
def get_rfid_data():
    """Tüm RFID verilerini döndür"""
    try:
        return jsonify(rfid_manager.get_data())
    except Exception as e:
        return jsonify({'error': f'Veri alınırken hata oluştu: {str(e)}'}), 500

@rfid_bp.route('/rfid-stats', methods=['GET'])
def get_rfid_stats():
    """RFID istatistiklerini döndür"""
    try:
        return jsonify(rfid_manager.get_statistics())
    except Exception as e:
        return jsonify({'error': f'İstatistikler alınırken hata oluştu: {str(e)}'}), 500

@rfid_bp.route('/write-rfid', methods=['POST'])
def write_rfid():
    """RFID kartına veri yaz"""
    try:
        data = request.get_json()
        
        if not data:
            return jsonify({'success': False, 'message': 'JSON verisi bulunamadı'}), 400
        
        card_id = data.get('card_id', '')
        write_data = data.get('data', '')
        
        if not write_data:
            return jsonify({'success': False, 'message': 'Yazılacak veri boş olamaz'}), 400
        
        # MQTT client'ı al ve mesaj gönder
        mqtt_client = get_mqtt_client()
        
        if not mqtt_client.is_connected():
            return jsonify({'success': False, 'message': 'MQTT bağlantısı yok'}), 503
        
        success = mqtt_client.publish_rfid_write(card_id, write_data)
        
        if success:
            return jsonify({
                'success': True, 
                'message': 'Yazma komutu gönderildi',
                'data': write_data
            })
        else:
            return jsonify({'success': False, 'message': 'Yazma komutu gönderilemedi'}), 500
        
    except Exception as e:
        return jsonify({'success': False, 'message': f'Hata: {str(e)}'}), 500

@rfid_bp.route('/clear-history', methods=['POST'])
def clear_rfid_history():
    """RFID geçmişini temizle"""
    try:
        data = request.get_json() or {}
        history_type = data.get('type', None)  # 'read', 'write' veya None (hepsi)
        
        rfid_manager.clear_history(history_type)
        
        message = f"{history_type.title() if history_type else 'Tüm'} geçmiş temizlendi"
        
        return jsonify({
            'success': True,
            'message': message,
            'data': rfid_manager.get_data()
        })
        
    except Exception as e:
        return jsonify({'success': False, 'message': f'Hata: {str(e)}'}), 500

@rfid_bp.route('/reset-data', methods=['POST'])
def reset_rfid_data():
    """Tüm RFID verilerini sıfırla"""
    try:
        rfid_manager.reset_data()
        
        return jsonify({
            'success': True,
            'message': 'RFID verileri sıfırlandı',
            'data': rfid_manager.get_data()
        })
        
    except Exception as e:
        return jsonify({'success': False, 'message': f'Hata: {str(e)}'}), 500

@rfid_bp.route('/card-history', methods=['GET'])
def get_card_history():
    """Kart okuma geçmişini döndür"""
    try:
        data = rfid_manager.get_data()
        return jsonify({
            'success': True,
            'history': data['card_history'],
            'total_reads': data['total_reads']
        })
    except Exception as e:
        return jsonify({'error': f'Geçmiş alınırken hata oluştu: {str(e)}'}), 500

@rfid_bp.route('/write-history', methods=['GET'])
def get_write_history():
    """Kart yazma geçmişini döndür"""
    try:
        data = rfid_manager.get_data()
        return jsonify({
            'success': True,
            'history': data['write_history'],
            'total_writes': data['total_writes']
        })
    except Exception as e:
        return jsonify({'error': f'Geçmiş alınırken hata oluştu: {str(e)}'}), 500