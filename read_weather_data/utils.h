#ifndef READ_WEATHER_DATA_UTILS_H
#define READ_WEATHER_DATA_UTILS_H

#include <ArduinoMqttClient.h>
#include <RTCZero.h>
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#endif
#include <string>

#include "ringbuffer.h"

constexpr float PRESSURE_SEALEVEL_HPA{1013.25F};

struct MqttConfig
{
    const char *server_ip;
    uint16_t server_port;
    const char *username;
    const char *password;
};

struct WifiConfig
{
    const char *ssid;
    const char *password;
    bool enableListMacAddress;
    bool enableScanAndListWifiNetworks;
    uint8_t max_retries_wifi;
    uint32_t retry_delay_ms;
};

struct WeatherMeasurements
{
    uint32_t timestamp; /// seconds since unix epoch in UTC timezone
    bool is_valid;      /// Is measurement valid
    float temp_c;       /// Temperature in [°C]
    float humidity;     /// Relative humidity [%]
    float pressue_hpa;  /// Absolute air pressure [hPa]
    float heat_index;   /// Heat index [°C]
    float pv_voltage;   /// Photovoltaik panel voltage [V]
};

bool isConnectedToWiFi(WiFiClass &wifi);

std::string ipToString(const uint32_t ip_addr);

std::string macAddressToString(WiFiClass &wifi);

const char *wifiEncryptionToString(const uint8_t encryption);

std::string networkListToString(WiFiClass &wifi);

const char *mqttErrorCodeToString(int error_code);

const char *wifiStatusToString(uint8_t wifi_status);

bool connectToWiFi(WiFiClass &wifi, const WifiConfig &wifi_config);

bool connectToMqttBroker(MqttClient &mqttclient, const MqttConfig &mqtt_config);

void print(const WeatherMeasurements &measurements);

void print(RTCZero &rtc, const uint8_t timezone_offset_h);

std::string createJsonStringFromMeasurement(const WeatherMeasurements &measurements);

void sendWeatherMeasurements(MqttClient &mqttclient, const char *topic_measurements,
                             const WeatherMeasurements &measurements);

void sendWeatherMeasurements(MqttClient &mqttclient, const char *topic_measurements,
                             utils::Ringbuffer<WeatherMeasurements> &tx_buffer);

bool isFloatValueValid(const float value);

bool areMeasurementsPlausible(const WeatherMeasurements &measurements);

bool readyToSchedule(const unsigned long next_schedule_interval);

#endif // READ_WEATHER_DATA_UTILS_H
