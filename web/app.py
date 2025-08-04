from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import paho.mqtt.client as mqtt
import json
from datetime import datetime
import threading

app = Flask(__name__)
app.config['SECRET_KEY'] = 'rfid_i2c_secret_key_2024'
socketio = SocketIO(app, cors_allowed_origins="*")

# RFID verileri için global değişkenler
rfid_data = {
    'last_card_id': 'Henüz kart okunmadı',
    'last_read_time': '',
    'last_written_data': '',
    'last_write_time': '',
    'total_reads': 0,
    'total_writes': 0,
    'card_history': [],
    'write_history': []
}

# I2C/Motor kontrol verileri için global değişkenler
i2c_data = {
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

# MQTT ayarları
MQTT_BROKER = "nagatron-sdt.local"
MQTT_PORT = 1883

# RFID MQTT konuları
MQTT_READ_TOPIC = "rfid/data"
MQTT_WRITE_TOPIC = "rfid/write"
MQTT_STATUS_TOPIC = "rfid/status"

# I2C MQTT konuları
MQTT_I2C_COMMAND_TOPIC = "i2c/command"      # Deneyap'a komut gönderme
MQTT_I2C_DATA_TOPIC = "i2c/data"            # Deneyap'tan veri alma
MQTT_I2C_STATUS_TOPIC = "i2c/status"        # I2C durum bilgisi
MQTT_MOTOR_DATA_TOPIC = "motor/data"        # Motor verileri
MQTT_QTR_DATA_TOPIC = "qtr/data"            # QTR sensör verileri

# MQTT Client oluştur
mqtt_client = mqtt.Client()
# MQTT bağlantı durumu izleme
mqtt_connected = False

def on_connect(client, userdata, flags, rc):
    global mqtt_connected
    print(f"MQTT Broker'a bağlandı. Kod: {rc}")
    mqtt_connected = True
    # RFID konularını dinle
    client.subscribe(MQTT_READ_TOPIC)
    client.subscribe(MQTT_STATUS_TOPIC)
    
    # I2C konularını dinle
    client.subscribe(MQTT_I2C_DATA_TOPIC)
    client.subscribe(MQTT_I2C_STATUS_TOPIC)
    client.subscribe(MQTT_MOTOR_DATA_TOPIC)
    client.subscribe(MQTT_QTR_DATA_TOPIC)

def on_message(client, userdata, msg):
    try:
        topic = msg.topic
        message = msg.payload.decode('utf-8')
        print(f"Gelen veri - Topic: {topic}, Mesaj: {message}")
        
        # RFID mesajları
        if topic == MQTT_READ_TOPIC:
            handle_rfid_read(message)
        elif topic == MQTT_STATUS_TOPIC:
            handle_write_status(message)
        
        # I2C mesajları
        elif topic == MQTT_I2C_DATA_TOPIC:
            handle_i2c_data(message)
        elif topic == MQTT_I2C_STATUS_TOPIC:
            handle_i2c_status(message)
        elif topic == MQTT_MOTOR_DATA_TOPIC:
            handle_motor_data(message)
        elif topic == MQTT_QTR_DATA_TOPIC:
            handle_qtr_data(message)
            
    except Exception as e:
        print(f"Veri işleme hatası: {e}")

# RFID işleyicileri (mevcut kodlar)
def handle_rfid_read(message):
    try:
        try:
            data = json.loads(message)
            card_id = data.get('card_id', message)
            card_data = data.get('data', '')
        except:
            card_id = message
            card_data = ''
        
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        rfid_data['last_card_id'] = card_id
        rfid_data['last_read_time'] = current_time
        rfid_data['total_reads'] += 1
        
        rfid_data['card_history'].insert(0, {
            'card_id': card_id,
            'data': card_data,
            'time': current_time,
            'type': 'read'
        })
        if len(rfid_data['card_history']) > 10:
            rfid_data['card_history'].pop()
        
        socketio.emit('rfid_update', rfid_data)
        
    except Exception as e:
        print(f"Okuma verisi işleme hatası: {e}")

def handle_write_status(message):
    try:
        data = json.loads(message)
        status = data.get('status', 'unknown')
        card_id = data.get('card_id', '')
        written_data = data.get('data', '')
        
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        if status == 'success':
            rfid_data['last_written_data'] = written_data
            rfid_data['last_write_time'] = current_time
            rfid_data['total_writes'] += 1
            
            rfid_data['write_history'].insert(0, {
                'card_id': card_id,
                'data': written_data,
                'time': current_time,
                'status': 'success'
            })
        else:
            rfid_data['write_history'].insert(0, {
                'card_id': card_id,
                'data': written_data,
                'time': current_time,
                'status': 'failed'
            })
        
        if len(rfid_data['write_history']) > 10:
            rfid_data['write_history'].pop()
        
        socketio.emit('write_status', {
            'status': status,
            'card_id': card_id,
            'data': written_data,
            'time': current_time
        })
        
        socketio.emit('rfid_update', rfid_data)
        
    except Exception as e:
        print(f"Yazma durumu işleme hatası: {e}")

# I2C/Motor işleyicileri
def handle_i2c_data(message):
    try:
        data = json.loads(message)
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        # Genel I2C durumu güncelle
        i2c_data['system_status']['i2c_connected'] = True
        i2c_data['system_status']['last_ping'] = current_time
        
        socketio.emit('i2c_update', i2c_data)
        
    except Exception as e:
        print(f"I2C verisi işleme hatası: {e}")

def handle_i2c_status(message):
    try:
        data = json.loads(message)
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        # I2C durum bilgilerini güncelle
        i2c_data['system_status'].update({
            'i2c_connected': data.get('connected', False),
            'motor_active': data.get('motor_active', False),
            'qtr_active': data.get('qtr_active', False),
            'last_ping': current_time
        })
        
        socketio.emit('i2c_status_update', i2c_data['system_status'])
        
    except Exception as e:
        print(f"I2C durum işleme hatası: {e}")

def handle_motor_data(message):
    try:
        data = json.loads(message)
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        # Motor verilerini güncelle
        i2c_data['motor_data'].update({
            'left_speed': data.get('left_speed', 0),
            'right_speed': data.get('right_speed', 0),
            'direction': data.get('direction', 'stop'),
            'last_update': current_time
        })
        
        i2c_data['system_status']['motor_active'] = True
        i2c_data['system_status']['last_ping'] = current_time
        
        socketio.emit('motor_update', i2c_data['motor_data'])
        
    except Exception as e:
        print(f"Motor verisi işleme hatası: {e}")

def handle_qtr_data(message):
    try:
        data = json.loads(message)
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        # QTR verilerini güncelle
        i2c_data['qtr_data'].update({
            'sensors': data.get('sensors', [0]*8),
            'position': data.get('position', 0),
            'line_detected': data.get('line_detected', False),
            'last_update': current_time
        })
        
        i2c_data['system_status']['qtr_active'] = True
        i2c_data['system_status']['last_ping'] = current_time
        
        socketio.emit('qtr_update', i2c_data['qtr_data'])
        
    except Exception as e:
        print(f"QTR verisi işleme hatası: {e}")

# MQTT client ayarları
mqtt_client.on_connect = on_connect

def on_disconnect(client, userdata, rc):
    global mqtt_connected
    print(f"MQTT Broker bağlantısı kesildi. Kod: {rc}")
    mqtt_connected = False

mqtt_client.on_disconnect = on_disconnect

mqtt_client.on_message = on_message

def start_mqtt():
    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_forever()
    except Exception as e:
        print(f"MQTT bağlantı hatası: {e}")

# Rotalar
@app.route('/')
def index():
    return render_template('index.html')

# RFID API'leri (mevcut)
@app.route('/api/rfid-data')
def get_rfid_data():
    return jsonify(rfid_data)

@app.route('/api/write-rfid', methods=['POST'])
def write_rfid():
    try:
        data = request.get_json()
        card_id = data.get('card_id', '')
        write_data = data.get('data', '')
        
        if not write_data:
            return jsonify({'success': False, 'message': 'Yazılacak veri boş olamaz'})
        
        write_command = {
            'action': 'write',
            'card_id': card_id,
            'data': write_data,
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        
        mqtt_client.publish(MQTT_WRITE_TOPIC, json.dumps(write_command))
        
        return jsonify({
            'success': True, 
            'message': 'Yazma komutu gönderildi',
            'data': write_data
        })
        
    except Exception as e:
        return jsonify({'success': False, 'message': f'Hata: {str(e)}'})

# I2C/Motor API'leri
@app.route('/api/i2c-data')
def get_i2c_data():
    return jsonify(i2c_data)

@app.route('/api/update-pid', methods=['POST'])
def update_pid():
    try:
        data = request.get_json()
        
        # PID değerlerini güncelle
        i2c_data['pid_settings'].update({
            'kp': float(data.get('kp', i2c_data['pid_settings']['kp'])),
            'ki': float(data.get('ki', i2c_data['pid_settings']['ki'])),
            'kd': float(data.get('kd', i2c_data['pid_settings']['kd'])),
            'setpoint': float(data.get('setpoint', i2c_data['pid_settings']['setpoint']))
        })
        
        # MQTT ile Deneyap'a gönder
        command = {
            'type': 'pid_update',
            'data': i2c_data['pid_settings'],
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        
        mqtt_client.publish(MQTT_I2C_COMMAND_TOPIC, json.dumps(command))
        
        return jsonify({'success': True, 'message': 'PID ayarları güncellendi'})
        
    except Exception as e:
        return jsonify({'success': False, 'message': f'Hata: {str(e)}'})

@app.route('/api/update-pins', methods=['POST'])
def update_pins():
    try:
        data = request.get_json()
        
        # Pin ayarlarını güncelle
        i2c_data['pin_settings'].update({
            'motor_left_pin1': int(data.get('motor_left_pin1', i2c_data['pin_settings']['motor_left_pin1'])),
            'motor_left_pin2': int(data.get('motor_left_pin2', i2c_data['pin_settings']['motor_left_pin2'])),
            'motor_right_pin1': int(data.get('motor_right_pin1', i2c_data['pin_settings']['motor_right_pin1'])),
            'motor_right_pin2': int(data.get('motor_right_pin2', i2c_data['pin_settings']['motor_right_pin2'])),
            'enable_left': int(data.get('enable_left', i2c_data['pin_settings']['enable_left'])),
            'enable_right': int(data.get('enable_right', i2c_data['pin_settings']['enable_right']))
        })
        
        # MQTT ile Deneyap'a gönder
        command = {
            'type': 'pin_update',
            'data': i2c_data['pin_settings'],
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        
        mqtt_client.publish(MQTT_I2C_COMMAND_TOPIC, json.dumps(command))
        
        return jsonify({'success': True, 'message': 'Pin ayarları güncellendi'})
        
    except Exception as e:
        return jsonify({'success': False, 'message': f'Hata: {str(e)}'})

@app.route('/api/update-speed', methods=['POST'])
def update_speed():
    try:
        data = request.get_json()
        
        # Hız ayarlarını güncelle
        i2c_data['speed_control'].update({
            'base_speed': int(data.get('base_speed', i2c_data['speed_control']['base_speed'])),
            'max_speed': int(data.get('max_speed', i2c_data['speed_control']['max_speed'])),
            'min_speed': int(data.get('min_speed', i2c_data['speed_control']['min_speed']))
        })
        
        # MQTT ile Deneyap'a gönder
        command = {
            'type': 'speed_update',
            'data': i2c_data['speed_control'],
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        
        mqtt_client.publish(MQTT_I2C_COMMAND_TOPIC, json.dumps(command))
        
        return jsonify({'success': True, 'message': 'Hız ayarları güncellendi'})
        
    except Exception as e:
        return jsonify({'success': False, 'message': f'Hata: {str(e)}'})

@app.route('/api/motor-control', methods=['POST'])
def motor_control():
    try:
        data = request.get_json()
        action = data.get('action', 'stop')  # start, stop, forward, backward, left, right
        speed = data.get('speed', i2c_data['speed_control']['base_speed'])
        
        # Motor kontrol komutu
        command = {
            'type': 'motor_control',
            'action': action,
            'speed': int(speed),
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        
        mqtt_client.publish(MQTT_I2C_COMMAND_TOPIC, json.dumps(command))
        
        return jsonify({'success': True, 'message': f'Motor komutu gönderildi: {action}'})
        
    except Exception as e:
        return jsonify({'success': False, 'message': f'Hata: {str(e)}'})

# WebSocket event'leri
@socketio.on('connect')
def handle_connect():
    print('Client bağlandı')
    emit('rfid_update', rfid_data)
    emit('i2c_update', i2c_data)

@socketio.on('disconnect')
def handle_disconnect():
    print('Client bağlantısı kesildi')

# RFID WebSocket (mevcut)
@socketio.on('write_rfid')
def handle_write_rfid(data):
    try:
        card_id = data.get('card_id', '')
        write_data = data.get('data', '')
        
        if not write_data:
            emit('write_error', {'message': 'Yazılacak veri boş olamaz'})
            return
        
        write_command = {
            'action': 'write',
            'card_id': card_id,
            'data': write_data,
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        
        mqtt_client.publish(MQTT_WRITE_TOPIC, json.dumps(write_command))
        emit('write_sent', {'message': 'Yazma komutu gönderildi'})
        
    except Exception as e:
        emit('write_error', {'message': f'Hata: {str(e)}'})

# I2C WebSocket event'leri
@socketio.on('update_pid_settings')
def handle_update_pid_settings(data):
    try:
        i2c_data['pid_settings'].update(data)
        
        command = {
            'type': 'pid_update',
            'data': i2c_data['pid_settings'],
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        
        mqtt_client.publish(MQTT_I2C_COMMAND_TOPIC, json.dumps(command))
        emit('pid_update_success', {'message': 'PID ayarları güncellendi'})
        
    except Exception as e:
        emit('pid_update_error', {'message': f'Hata: {str(e)}'})

@socketio.on('motor_command')
def handle_motor_command(data):
    try:
        command = {
            'type': 'motor_control',
            'action': data.get('action', 'stop'),
            'speed': data.get('speed', i2c_data['speed_control']['base_speed']),
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        
        mqtt_client.publish(MQTT_I2C_COMMAND_TOPIC, json.dumps(command))
        emit('motor_command_sent', {'message': f"Motor komutu gönderildi: {command['action']}"})
        
    except Exception as e:
        emit('motor_command_error', {'message': f'Hata: {str(e)}'})



@app.route('/api/mqtt-status')
def mqtt_status():
    return jsonify({'connected': mqtt_connected})

if __name__ == '__main__':
    # MQTT'yi ayrı thread'de başlat
    mqtt_thread = threading.Thread(target=start_mqtt)
    mqtt_thread.daemon = True
    mqtt_thread.start()
    
    # Flask uygulamasını başlat
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)