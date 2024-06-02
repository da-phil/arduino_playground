#ifndef READ_WEATHER_DATA_H
#define READ_WEATHER_DATA_H

struct MqttConfig
{
    const char *server_ip;
    int server_port;
    const char *username;
    const char *password;
};

struct WifiConfig
{
    const char *ssid;
    const char *password;
    bool enablePrintMacAddress;
    bool enableScanAndListWifiNetworks;
};

struct Measurements
{
    uint32_t timestamp; /// seconds since unix epoch in UTC timezone
    float temp_c;       /// Temperature in [°C]
    float humidity;     /// Relative humidity [%]
    float heat_index;   /// Heat index [°C]
    float pv_voltage;   /// Photovoltaik panel voltage [V]
};

#endif // READ_WEATHER_DATA_H
