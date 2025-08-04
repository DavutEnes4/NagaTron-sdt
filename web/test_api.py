#!/usr/bin/env python3
"""
RFID-I2C Web Sunucusu API Testleri
Bu dosya API endpoint'lerini test etmek için kullanılır.
"""

import requests
import json
import time
from datetime import datetime

class APITester:
    def __init__(self, base_url="http://localhost:5000"):
        self.base_url = base_url
        self.session = requests.Session()
    
    def test_health_endpoint(self):
        """Sağlık kontrolü endpoint'ini test et"""
        print("🔍 Sağlık kontrolü testi...")
        try:
            response = self.session.get(f"{self.base_url}/api/health")
            if response.status_code == 200:
                data = response.json()
                print(f"✅ Sağlık kontrolü başarılı: {data['status']}")
                return True
            else:
                print(f"❌ Sağlık kontrolü başarısız: {response.status_code}")
                return False
        except Exception as e:
            print(f"❌ Sağlık kontrolü hatası: {e}")
            return False
    
    def test_system_info(self):
        """Sistem bilgileri endpoint'ini test et"""
        print("🔍 Sistem bilgileri testi...")
        try:
            response = self.session.get(f"{self.base_url}/api/system-info")
            if response.status_code == 200:
                data = response.json()
                print(f"✅ Sistem bilgileri alındı: {data['system']['name']}")
                return True
            else:
                print(f"❌ Sistem bilgileri başarısız: {response.status_code}")
                return False
        except Exception as e:
            print(f"❌ Sistem bilgileri hatası: {e}")
            return False
    
    def test_mqtt_status(self):
        """MQTT durumu endpoint'ini test et"""
        print("🔍 MQTT durumu testi...")
        try:
            response = self.session.get(f"{self.base_url}/api/mqtt-status")
            if response.status_code == 200:
                data = response.json()
                status = "Bağlı" if data['connected'] else "Bağlı Değil"
                print(f"✅ MQTT durumu: {status}")
                return True
            else:
                print(f"❌ MQTT durumu başarısız: {response.status_code}")
                return False
        except Exception as e:
            print(f"❌ MQTT durumu hatası: {e}")
            return False
    
    def test_rfid_data(self):
        """RFID verileri endpoint'ini test et"""
        print("🔍 RFID verileri testi...")
        try:
            response = self.session.get(f"{self.base_url}/api/rfid-data")
            if response.status_code == 200:
                data = response.json()
                print(f"✅ RFID verileri alındı: {data['total_reads']} okuma, {data['total_writes']} yazma")
                return True
            else:
                print(f"❌ RFID verileri başarısız: {response.status_code}")
                return False
        except Exception as e:
            print(f"❌ RFID verileri hatası: {e}")
            return False
    
    def test_i2c_data(self):
        """I2C verileri endpoint'ini test et"""
        print("🔍 I2C verileri testi...")
        try:
            response = self.session.get(f"{self.base_url}/api/i2c-data")
            if response.status_code == 200:
                data = response.json()
                print(f"✅ I2C verileri alındı: Bağlantı {data['system_status']['i2c_connected']}")
                return True
            else:
                print(f"❌ I2C verileri başarısız: {response.status_code}")
                return False
        except Exception as e:
            print(f"❌ I2C verileri hatası: {e}")
            return False
    
    def test_rfid_write(self):
        """RFID yazma endpoint'ini test et"""
        print("🔍 RFID yazma testi...")
        try:
            test_data = {
                "card_id": "test_card_123",
                "data": f"Test verisi - {datetime.now().strftime('%H:%M:%S')}"
            }
            
            response = self.session.post(
                f"{self.base_url}/api/write-rfid",
                json=test_data,
                headers={'Content-Type': 'application/json'}
            )
            
            if response.status_code == 200:
                data = response.json()
                if data.get('success'):
                    print(f"✅ RFID yazma başarılı: {data['message']}")
                    return True
                else:
                    print(f"❌ RFID yazma başarısız: {data.get('message', 'Bilinmeyen hata')}")
                    return False
            else:
                print(f"❌ RFID yazma HTTP hatası: {response.status_code}")
                return False
        except Exception as e:
            print(f"❌ RFID yazma hatası: {e}")
            return False
    
    def test_pid_update(self):
        """PID güncelleme endpoint'ini test et"""
        print("🔍 PID güncelleme testi...")
        try:
            test_data = {
                "kp": 2.5,
                "ki": 0.2,
                "kd": 0.8,
                "setpoint": 0
            }
            
            response = self.session.post(
                f"{self.base_url}/api/update-pid",
                json=test_data,
                headers={'Content-Type': 'application/json'}
            )
            
            if response.status_code == 200:
                data = response.json()
                if data.get('success'):
                    print(f"✅ PID güncelleme başarılı: {data['message']}")
                    return True
                else:
                    print(f"❌ PID güncelleme başarısız: {data.get('message', 'Bilinmeyen hata')}")
                    return False
            else:
                print(f"❌ PID güncelleme HTTP hatası: {response.status_code}")
                return False
        except Exception as e:
            print(f"❌ PID güncelleme hatası: {e}")
            return False
    
    def test_motor_control(self):
        """Motor kontrol endpoint'ini test et"""
        print("🔍 Motor kontrol testi...")
        try:
            test_data = {
                "action": "stop",
                "speed": 0
            }
            
            response = self.session.post(
                f"{self.base_url}/api/motor-control",
                json=test_data,
                headers={'Content-Type': 'application/json'}
            )
            
            if response.status_code == 200:
                data = response.json()
                if data.get('success'):
                    print(f"✅ Motor kontrol başarılı: {data['message']}")
                    return True
                else:
                    print(f"❌ Motor kontrol başarısız: {data.get('message', 'Bilinmeyen hata')}")
                    return False
            else:
                print(f"❌ Motor kontrol HTTP hatası: {response.status_code}")
                return False
        except Exception as e:
            print(f"❌ Motor kontrol hatası: {e}")
            return False
    
    def test_ping(self):
        """Ping endpoint'ini test et"""
        print("🔍 Ping testi...")
        try:
            response = self.session.get(f"{self.base_url}/api/ping")
            if response.status_code == 200:
                data = response.json()
                if data.get('success'):
                    print(f"✅ Ping başarılı: {data['message']}")
                    return True
                else:
                    print(f"❌ Ping başarısız: {data.get('message', 'Bilinmeyen hata')}")
                    return False
            else:
                print(f"❌ Ping HTTP hatası: {response.status_code}")
                return False
        except Exception as e:
            print(f"❌ Ping hatası: {e}")
            return False
    
    def run_all_tests(self):
        """Tüm testleri çalıştır"""
        print("🚀 API Testleri Başlatılıyor...")
        print("=" * 50)
        
        tests = [
            self.test_ping,
            self.test_health_endpoint,
            self.test_system_info,
            self.test_mqtt_status,
            self.test_rfid_data,
            self.test_i2c_data,
            self.test_rfid_write,
            self.test_pid_update,
            self.test_motor_control
        ]
        
        passed = 0
        total = len(tests)
        
        for test in tests:
            if test():
                passed += 1
            print("-" * 30)
            time.sleep(0.5)  # Testler arası kısa bekleme
        
        print("=" * 50)
        print(f"📊 Test Sonuçları: {passed}/{total} başarılı")
        
        if passed == total:
            print("🎉 Tüm testler başarılı!")
        else:
            print("⚠️ Bazı testler başarısız oldu.")
        
        return passed == total

def main():
    """Ana test fonksiyonu"""
    print("RFID-I2C Web Sunucusu API Testleri")
    print("Sunucunun çalıştığından emin olun: python app.py")
    print()
    
    # Sunucu çalışıyor mu kontrol et
    try:
        response = requests.get("http://localhost:5000/api/ping", timeout=5)
        if response.status_code != 200:
            print("❌ Sunucu çalışmıyor! Lütfen önce 'python app.py' komutunu çalıştırın.")
            return
    except:
        print("❌ Sunucu çalışmıyor! Lütfen önce 'python app.py' komutunu çalıştırın.")
        return
    
    # Testleri başlat
    tester = APITester()
    success = tester.run_all_tests()
    
    if success:
        print("\n✅ API testleri tamamlandı. Sistem çalışıyor!")
    else:
        print("\n❌ Bazı API testleri başarısız oldu. Lütfen kontrol edin.")

if __name__ == "__main__":
    main() 