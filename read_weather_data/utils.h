#include <string>

std::string IpToString(const std::uint32_t ip_addr);

std::string printMacAddress(WiFiClass &wifi);

std::string networkListToString(WiFiClass &wifi);

const char *mqttErrorCodeToString(int error_code);

const char *wifiStatusToString(std::uint8_t wifi_status);