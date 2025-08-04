# data/rfid_data.py
"""
RFID Veri Yönetimi Modülü
RFID ile ilgili tüm veri işlemleri bu modülde yönetilir.
"""

from datetime import datetime
import json
from config.settings import DEFAULT_RFID_DATA, GENERAL_CONFIG

class RFIDDataManager:
    def __init__(self):
        self.data = DEFAULT_RFID_DATA.copy()
    
    def get_data(self):
        """Tüm RFID verilerini döndür"""
        return self.data
    
    def update_card_read(self, message):
        """Kart okuma verilerini güncelle"""
        try:
            # JSON formatında gelen veriyi parse et
            try:
                data = json.loads(message)
                card_id = data.get('card_id', message)
                card_data = data.get('data', '')
            except:
                card_id = message
                card_data = ''
            
            current_time = datetime.now().strftime(GENERAL_CONFIG['DATE_FORMAT'])
            
            # Ana verileri güncelle
            self.data['last_card_id'] = card_id
            self.data['last_read_time'] = current_time
            self.data['total_reads'] += 1
            
            # Geçmişe ekle
            self._add_to_history('card_history', {
                'card_id': card_id,
                'data': card_data,
                'time': current_time,
                'type': 'read'
            })
            
            return True
            
        except Exception as e:
            print(f"RFID okuma verisi işleme hatası: {e}")
            return False
    
    def update_write_status(self, message):
        """Yazma durumu verilerini güncelle"""
        try:
            data = json.loads(message)
            status = data.get('status', 'unknown')
            card_id = data.get('card_id', '')
            written_data = data.get('data', '')
            
            current_time = datetime.now().strftime(GENERAL_CONFIG['DATE_FORMAT'])
            
            if status == 'success':
                self.data['last_written_data'] = written_data
                self.data['last_write_time'] = current_time
                self.data['total_writes'] += 1
            
            # Yazma geçmişine ekle
            self._add_to_history('write_history', {
                'card_id': card_id,
                'data': written_data,
                'time': current_time,
                'status': status
            })
            
            return {
                'status': status,
                'card_id': card_id,
                'data': written_data,
                'time': current_time
            }
            
        except Exception as e:
            print(f"RFID yazma durumu işleme hatası: {e}")
            return None
    
    def _add_to_history(self, history_type, item):
        """Geçmiş listesine öğe ekle ve boyutu kontrol et"""
        if history_type in self.data:
            self.data[history_type].insert(0, item)
            if len(self.data[history_type]) > GENERAL_CONFIG['MAX_HISTORY_SIZE']:
                self.data[history_type].pop()
    
    def get_statistics(self):
        """RFID istatistiklerini döndür"""
        return {
            'total_reads': self.data['total_reads'],
            'total_writes': self.data['total_writes'],
            'last_activity': self.data.get('last_read_time', 'Henüz aktivite yok'),
            'success_rate': self._calculate_success_rate()
        }
    
    def _calculate_success_rate(self):
        """Yazma başarı oranını hesapla"""
        if not self.data['write_history']:
            return 100.0
        
        successful_writes = sum(1 for item in self.data['write_history'] 
                               if item.get('status') == 'success')
        total_writes = len(self.data['write_history'])
        
        return round((successful_writes / total_writes) * 100, 2)
    
    def clear_history(self, history_type=None):
        """Geçmişi temizle"""
        if history_type == 'read':
            self.data['card_history'] = []
        elif history_type == 'write':
            self.data['write_history'] = []
        else:
            self.data['card_history'] = []
            self.data['write_history'] = []
    
    def reset_data(self):
        """Tüm verileri sıfırla"""
        self.data = DEFAULT_RFID_DATA.copy()

# Global RFID veri yöneticisi
rfid_manager = RFIDDataManager()