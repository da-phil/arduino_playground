#include <ArduinoMqttClient.h>
#include <WiFi101.h>
#include <cstdio>

#include "utils.h"

std::string IpToString(const std::uint32_t ip_addr)
{
    char ip_addr_str[16U];
    snprintf(ip_addr_str, sizeof(ip_addr_str), "%u.%u.%u.%u", //
             ip_addr & 0xFF, (ip_addr >> 8) & 0xFF, (ip_addr >> 16) & 0xFF, (ip_addr >> 24) & 0xFF);

    return std::string{ip_addr_str};
}

std::string printMacAddress(WiFiClass &wifi)
{
    std::uint8_t mac[6];
    char mac_addr_str[32U];

    wifi.macAddress(mac);
    snprintf(mac_addr_str, sizeof(mac_addr_str), "%x:%x:%x:%x:%x:%x", //
             mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);

    return std::string{mac_addr_str};
}

std::string networkListToString(WiFiClass &wifi)
{
    std::string output{"** Scan Networks **\n"};

    const auto found_networks_num = wifi.scanNetworks();

    // print the network number and name for each network found:
    for (int network_num = 0; network_num < found_networks_num; network_num++)
    {
        output += (std::to_string(network_num) + ") ");
        output += wifi.SSID(network_num);
        output += "\tSignal: ";
        output += std::to_string(wifi.RSSI(network_num));
        output += " dBm \tEncryption: ";
        output += wifi.encryptionType(network_num);
    }
    return output;
}

const char *wifiStatusToString(std::uint8_t wifi_status)
{
    switch (static_cast<wl_status_t>(wifi_status))
    {
    case WL_NO_SHIELD:
        return "No wifi shield";
    case WL_CONNECT_FAILED:
        return "Wifi connect failed";
    case WL_CONNECTION_LOST:
        return "Wifi connection lost";
    case WL_DISCONNECTED:
        return "Wifi disconnected";
    case WL_IDLE_STATUS:
        return "WiFi in IDLE mode";
    case WL_CONNECTED:
        [[fallthrough]];
    default:
        return "Wifi connected";
    }
}

const char *mqttErrorCodeToString(int error_code)
{
    switch (error_code)
    {
    case MQTT_CONNECTION_REFUSED:
        return "Connection refused";
    case MQTT_CONNECTION_TIMEOUT:
        return "Connection timeout";
    case MQTT_UNACCEPTABLE_PROTOCOL_VERSION:
        return "Unacceptable protocol version";
    case MQTT_IDENTIFIER_REJECTED:
        return "Identifier rejected";
    case MQTT_SERVER_UNAVAILABLE:
        return "Server unavailable";
    case MQTT_BAD_USER_NAME_OR_PASSWORD:
        return "Bad user name or password";
    case MQTT_NOT_AUTHORIZED:
        return "Not authenticated";
    case MQTT_SUCCESS:
        [[fallthrough]];
    default:
        return "Connection established successfully";
    }
}
