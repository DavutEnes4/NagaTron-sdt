#ifndef CONFIG_H
#define CONFIG_H

// ========================================
// ORTAK KONFİGÜRASYON DOSYASI
// ========================================

// WiFi Ağları (Birden fazla ağ desteği)
struct WiFiNetwork {
    const char* ssid;
    const char* password;
    bool isActive;
};

// WiFi ağları listesi
WiFiNetwork wifiNetworks[] = {
    {"Furkan's Galaxy S20 FE", "12345678", true},
    {"NAGATRON_WIFI", "nagatron2024", false},
    {"ESP32_AP", "esp32password", false},
    {"Home_WiFi", "homepassword", false}
};

const int WIFI_NETWORK_COUNT = sizeof(wifiNetworks) / sizeof(wifiNetworks[0]);

// MQTT Sunucu Ayarları
struct MQTTServer {
    const char* broker;
    int port;
    const char* username;
    const char* password;
    const char* clientId;
};

// MQTT sunucuları listesi
MQTTServer mqttServers[] = {
    {"nagatron-sdt.local", 1883, "", "", "NAGATRON_DEVICE"},
    {"192.168.1.100", 1883, "user", "pass", "NAGATRON_DEVICE"},
    {"broker.hivemq.com", 1883, "", "", "NAGATRON_DEVICE"}
};

const int MQTT_SERVER_COUNT = sizeof(mqttServers) / sizeof(mqttServers[0]);

// Sistem Ayarları
#define SYSTEM_STARTUP_DELAY 3000  // 3 saniye bekleme süresi
#define MQTT_RECONNECT_INTERVAL 5000
#define WIFI_RECONNECT_INTERVAL 10000

// RFID Sistemi Ayarları
#define RFID_RST_PIN D9
#define RFID_SS_PIN D10
#define RFID_WRITE_TIMEOUT 30000  // 30 saniye yazma timeout

// QTR/PID Sistemi Ayarları
#define I2C_SLAVE_ADDRESS 8
#define I2C_CLOCK_SPEED 100000
#define QTR_SENSOR_COUNT 8
#define PID_UPDATE_INTERVAL 100
#define QTR_UPDATE_INTERVAL 50

// Motor Pinleri (BTS7960B)
#define RPWM_R 10   // Sağ motor ileri PWM
#define LPWM_R 9    // Sağ motor geri PWM
#define R_EN_R 7    // Sağ motor R_EN
#define L_EN_R 8    // Sağ motor L_EN

#define RPWM_L 6    // Sol motor ileri PWM
#define LPWM_L 5    // Sol motor geri PWM
#define R_EN_L 3    // Sol motor R_EN
#define L_EN_L 4    // Sol motor L_EN

// QTR Sensör Pinleri
const uint8_t QTR_SENSOR_PINS[] = {A0, A1, A2, A3, A4, A5, A6, A7};

// PID Varsayılan Değerleri
#define DEFAULT_KP 0.4
#define DEFAULT_KI 0.0
#define DEFAULT_KD 2.0
#define DEFAULT_BASE_SPEED 50
#define DEFAULT_MAX_SPEED 120

// MQTT Topic'leri
#define MQTT_RFID_READ_TOPIC "rfid/data"
#define MQTT_RFID_WRITE_TOPIC "rfid/write"
#define MQTT_RFID_STATUS_TOPIC "rfid/status"
#define MQTT_QTR_DATA_TOPIC "qtr/data"
#define MQTT_MOTOR_CONTROL_TOPIC "motor/control"
#define MQTT_PID_CONFIG_TOPIC "pid/config"
#define MQTT_SYSTEM_STATUS_TOPIC "system/status"

// Debug Ayarları
#define DEBUG_SERIAL_BAUD 115200
#define DEBUG_ENABLED true

// Hata Kodları
#define ERROR_NONE 0
#define ERROR_WIFI_CONNECTION 1
#define ERROR_MQTT_CONNECTION 2
#define ERROR_RFID_READ 3
#define ERROR_RFID_WRITE 4
#define ERROR_I2C_COMMUNICATION 5
#define ERROR_MOTOR_CONTROL 6

// Sistem Durumları
enum SystemState {
    SYSTEM_INIT,
    SYSTEM_CONNECTING_WIFI,
    SYSTEM_CONNECTING_MQTT,
    SYSTEM_READY,
    SYSTEM_ERROR,
    SYSTEM_RUNNING
};

// Yardımcı Fonksiyonlar
String getWiFiStatus() {
    switch (WiFi.status()) {
        case WL_CONNECTED: return "CONNECTED";
        case WL_DISCONNECTED: return "DISCONNECTED";
        case WL_CONNECT_FAILED: return "CONNECT_FAILED";
        case WL_NO_SSID_AVAIL: return "NO_SSID_AVAIL";
        case WL_SCAN_COMPLETED: return "SCAN_COMPLETED";
        case WL_IDLE_STATUS: return "IDLE_STATUS";
        case WL_NO_SHIELD: return "NO_SHIELD";
        default: return "UNKNOWN";
    }
}

String getSystemStateString(SystemState state) {
    switch (state) {
        case SYSTEM_INIT: return "INIT";
        case SYSTEM_CONNECTING_WIFI: return "CONNECTING_WIFI";
        case SYSTEM_CONNECTING_MQTT: return "CONNECTING_MQTT";
        case SYSTEM_READY: return "READY";
        case SYSTEM_ERROR: return "ERROR";
        case SYSTEM_RUNNING: return "RUNNING";
        default: return "UNKNOWN";
    }
}

#endif // CONFIG_H 