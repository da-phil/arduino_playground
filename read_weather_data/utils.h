#ifndef READ_WEATHER_DATA_UTILS_H
#define READ_WEATHER_DATA_UTILS_H

#include <ArduinoMqttClient.h>
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#endif
#include <string>

struct MqttConfig
{
    const char *server_ip;
    int server_port;
    const char *username;
    const char *password;
    unsigned long mqtt_msg_send_delay_ms;
};

struct WifiConfig
{
    const char *ssid;
    const char *password;
    bool enablePrintMacAddress;
    bool enableScanAndListWifiNetworks;
    uint8_t max_retries_wifi;
    uint32_t retry_delay_ms;
};

bool isConnectedToWiFi(WiFiClass &wifi);

std::string IpToString(const uint32_t ip_addr);

std::string printMacAddress(WiFiClass &wifi);

std::string networkListToString(WiFiClass &wifi);

const char *mqttErrorCodeToString(int error_code);

const char *wifiStatusToString(uint8_t wifi_status);

bool connectToWiFi(WiFiClass &wifi, const WifiConfig &wifi_config);

bool connectToMqttBroker(MqttClient &mqttclient, const MqttConfig &mqtt_config, uint32_t &time_ready_for_data_transfer);

bool readyToSchedule(const unsigned long next_schedule_interval);

#endif // READ_WEATHER_DATA_UTILS_H
