#ifndef READ_WEATHER_DATA_UTILS_H
#define READ_WEATHER_DATA_UTILS_H

#include <WiFi101.h>
#include <string>

std::string IpToString(const std::uint32_t ip_addr);

std::string printMacAddress(WiFiClass &wifi);

std::string networkListToString(WiFiClass &wifi);

const char *mqttErrorCodeToString(int error_code);

const char *wifiStatusToString(std::uint8_t wifi_status);

#endif // READ_WEATHER_DATA_UTILS_H