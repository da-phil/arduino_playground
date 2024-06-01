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
    float temp_c;
    float humidity;
    float heat_index;
    float pv_voltage;
};