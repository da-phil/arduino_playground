#ifndef READ_WEATHER_DATA_UTILS_H
#define READ_WEATHER_DATA_UTILS_H

#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#endif
#include <string>

bool isConnectedToWiFi(WiFiClass &wifi);

std::string IpToString(const uint32_t ip_addr);

std::string printMacAddress(WiFiClass &wifi);

std::string networkListToString(WiFiClass &wifi);

const char *mqttErrorCodeToString(int error_code);

const char *wifiStatusToString(uint8_t wifi_status);

#endif // READ_WEATHER_DATA_UTILS_H
