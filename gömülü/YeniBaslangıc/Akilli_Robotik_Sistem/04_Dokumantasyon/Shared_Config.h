/*
 * 🔧 Ortak Konfigürasyon Dosyası
 * Tüm modüller için merkezi ayar yönetimi
 * 
 * Bu dosyada yapılan değişiklikler tüm modüllerde geçerli olur
 * Tek bir yerden tüm ayarları yönetebilirsiniz
 */

#ifndef SHARED_CONFIG_H
#define SHARED_CONFIG_H

// ============================================================================
// 🌐 WiFi AYARLARI
// ============================================================================
#define WIFI_SSID_1 "WiFi_SSID_1"
#define WIFI_PASSWORD_1 "WiFi_Password_1"
#define WIFI_SSID_2 "WiFi_SSID_2"
#define WIFI_PASSWORD_2 "WiFi_Password_2"

// ============================================================================
// 📡 MQTT AYARLARI
// ============================================================================
#define MQTT_SERVER "broker.mqtt.com"
#define MQTT_PORT 1883
#define MQTT_USERNAME "your_username"
#define MQTT_PASSWORD "your_password"

// ============================================================================
// 🏷️ CİHAZ KİMLİKLERİ
// ============================================================================
#define DEVICE_ID_RFID "deneyap_kart_1"
#define DEVICE_ID_BRIDGE "deneyap_kart_2"
#define DEVICE_ID_ARDUINO "arduino_mega"

// ============================================================================
// 🔗 I²C AYARLARI
// ============================================================================
#define I2C_SLAVE_ADDRESS 0x10
#define ARDUINO_MEGA_ADDRESS 0x01

// ============================================================================
// 📍 RFID PIN KONFİGÜRASYONU
// ============================================================================
#define RFID_SDA_PIN 5
#define RFID_SCK_PIN 18
#define RFID_MOSI_PIN 23
#define RFID_MISO_PIN 19
#define RFID_RST_PIN 22
#define RFID_IRQ_PIN 21

// ============================================================================
// 🚗 ARDUINO MEGA PIN KONFİGÜRASYONU
// ============================================================================

// QTR-8A Line Sensor
#define QTR_EMITTER_PIN 2
#define QTR_SENSOR_PINS {A0, A1, A2, A3, A4, A5, A6, A7}

// BTS7960B Motor Driver
#define MOTOR_LEFT_RPWM_PIN 3
#define MOTOR_LEFT_LPWM_PIN 4
#define MOTOR_LEFT_R_EN_PIN 5
#define MOTOR_LEFT_L_EN_PIN 6
#define MOTOR_RIGHT_RPWM_PIN 7
#define MOTOR_RIGHT_LPWM_PIN 8
#define MOTOR_RIGHT_R_EN_PIN 9
#define MOTOR_RIGHT_L_EN_PIN 10

// GY-89 IMU (I²C)
#define IMU_SDA_PIN 20
#define IMU_SCL_PIN 21

// RadioLink R12DS v1.1
#define RADIOLINK_CH1_PIN 22  // Throttle
#define RADIOLINK_CH2_PIN 23  // Aileron
#define RADIOLINK_CH3_PIN 24  // Elevator
#define RADIOLINK_CH4_PIN 25  // Rudder
#define RADIOLINK_CH5_PIN 26  // Gear
#define RADIOLINK_CH6_PIN 27  // Aux1
#define RADIOLINK_CH7_PIN 28  // Aux2
#define RADIOLINK_CH8_PIN 29  // Aux3

// ============================================================================
// ⚙️ PID AYARLARI
// ============================================================================
#define PID_KP_DEFAULT 2.0
#define PID_KI_DEFAULT 0.1
#define PID_KD_DEFAULT 0.5
#define PID_SETPOINT_DEFAULT 3500

// ============================================================================
// 🎛️ KONTROL AYARLARI
// ============================================================================
#define MOTOR_MAX_SPEED 255
#define MOTOR_MIN_SPEED 0
#define RADIOLINK_TIMEOUT_MS 1000
#define RADIOLINK_PULSE_MIN 1000
#define RADIOLINK_PULSE_MAX 2000
#define RADIOLINK_PULSE_CENTER 1500

// ============================================================================
// 📊 SENSÖR AYARLARI
// ============================================================================
#define QTR_CALIBRATION_SAMPLES 10
#define QTR_CALIBRATION_SPEED 100
#define QTR_CALIBRATION_DURATION 1000
#define IMU_UPDATE_INTERVAL 50
#define MOTOR_UPDATE_INTERVAL 100

// ============================================================================
// 🔄 SİSTEM AYARLARI
// ============================================================================
#define MQTT_RECONNECT_INTERVAL 5000
#define WIFI_RECONNECT_INTERVAL 5000
#define DATA_TRANSMISSION_INTERVAL 100
#define EEPROM_CONFIG_ADDRESS 0

// ============================================================================
// 📋 MQTT KONULARI
// ============================================================================

// RFID Konuları
#define MQTT_TOPIC_RFID_UID "rfid/reader/uid"
#define MQTT_TOPIC_RFID_COMMAND "rfid/command/write"
#define MQTT_TOPIC_RFID_STATUS "rfid/status"
#define MQTT_TOPIC_RFID_CONFIG "config/pins/deneyap_kart_1"

// Arduino Mega Konuları
#define MQTT_TOPIC_IMU "sensors/arduino_mega/gy89"
#define MQTT_TOPIC_QTR "sensors/arduino_mega/qtr8a"
#define MQTT_TOPIC_MOTOR "sensors/arduino_mega/motors"
#define MQTT_TOPIC_RADIOLINK "sensors/arduino_mega/radiolink"
#define MQTT_TOPIC_COMMAND "command/motors/arduino_mega"
#define MQTT_TOPIC_CONFIG "config/pins/arduino_mega"
#define MQTT_TOPIC_PID "config/pid/arduino_mega"
#define MQTT_TOPIC_STATUS "system/deneyap_kart_2/status"

// ============================================================================
// 🎯 KONTROL MODLARI
// ============================================================================
#define MODE_AUTONOMOUS 0
#define MODE_MANUAL 1
#define MODE_MQTT 2
#define MODE_CALIBRATION 3

// ============================================================================
// 📝 KULLANIM TALİMATLARI
// ============================================================================
/*
 * 🔧 Bu dosyayı kullanmak için:
 * 
 * 1. Her .ino dosyasının başına ekleyin:
 *    #include "Shared_Config.h"
 * 
 * 2. Eski tanımlamaları kaldırın:
 *    // const char* ssid1 = "WiFi_SSID_1";  // ❌ Eski
 *    // const char* ssid1 = WIFI_SSID_1;    // ✅ Yeni
 * 
 * 3. Değişiklik yapmak için sadece bu dosyayı düzenleyin
 * 
 * 4. Tüm modüllerde aynı ayarlar kullanılacak
 * 
 * Örnek kullanım:
 *    wifiMulti.addAP(WIFI_SSID_1, WIFI_PASSWORD_1);
 *    client.setServer(MQTT_SERVER, MQTT_PORT);
 *    mfrc522.PCD_Init(RFID_SDA_PIN, RFID_RST_PIN);
 */

#endif // SHARED_CONFIG_H 