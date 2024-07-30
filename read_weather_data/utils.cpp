#include <cstdio>

#include "logging.h"
#include "utils.h"

bool isConnectedToWiFi(WiFiClass &wifi)
{
    return (wifi.status() == WL_CONNECTED);
}

std::string ipToString(const uint32_t ip_addr)
{
    char ip_addr_str[16U];
    snprintf(ip_addr_str, sizeof(ip_addr_str), "%lu.%lu.%lu.%lu", //
             ip_addr & 0xFF, (ip_addr >> 8) & 0xFF, (ip_addr >> 16) & 0xFF, (ip_addr >> 24) & 0xFF);

    return std::string{ip_addr_str};
}

std::string macAddressToString(WiFiClass &wifi)
{
    std::uint8_t mac[6];
    wifi.macAddress(mac);
    char mac_addr_str[60U];
    snprintf(mac_addr_str, sizeof(mac_addr_str), "MAC address: %02x:%02x:%02x:%02x:%02x:%02x", //
             mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);

    return std::string{mac_addr_str};
}

const char *wifiEncryptionToString(const uint8_t encryption)
{
    switch (encryption)
    {
#if defined(ARDUINO_SAMD_MKR1000)
    case M2M_WIFI_SEC_OPEN:
        return "Open";
    case M2M_WIFI_SEC_WPA_PSK:
        return "WPA_PSK";
    case M2M_WIFI_SEC_WEP:
        return "WEP";
    case M2M_WIFI_SEC_802_1X:
        return "802.1x";
    case M2M_WIFI_SEC_INVALID:
        [[fallthrough]];
    default:
        return "Invalid";
#elif defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
    case AUTH_MODE_OPEN_SYSTEM:
        return "Open";
    case AUTH_MODE_SHARED_KEY:
        return "Shared Key";
    case AUTH_MODE_WPA:
        return "WPA";
    case AUTH_MODE_WPA_PSK:
        return "WPA PSK";
    case AUTH_MODE_WPA2_PSK:
        return "WPA2 PSK";
    case AUTH_MODE_AUTO:
        return "AUTO";
    case AUTH_MODE_INVALID:
        [[fallthrough]];
    default:
        return "Invalid";
#endif
    }
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
        output += ",  Signal: ";
        output += std::to_string(wifi.RSSI(network_num));
        output += " dBm,  Encryption: ";
        output += wifiEncryptionToString(wifi.encryptionType(network_num));
        output += "\n";
    }
    return output;
}

const char *wifiStatusToString(uint8_t wifi_status)
{
    switch (static_cast<wl_status_t>(wifi_status))
    {
    case WL_NO_SHIELD:
        return "No wifi shield";
    case WL_CONNECT_FAILED:
        return "Connect failed";
    case WL_CONNECTION_LOST:
        return "Connection lost";
    case WL_DISCONNECTED:
        return "Disconnected";
    case WL_IDLE_STATUS:
        return "In IDLE mode";
    case WL_CONNECTED:
        [[fallthrough]];
    default:
        return "Connected";
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

bool connectToWiFi(WiFiClass &wifi, const WifiConfig &wifi_config)
{
    if (wifi_config.enableListMacAddress)
    {
        Logger::get().logInfo("WiFi MAC address: %s", macAddressToString(wifi).c_str());
    }

    if (wifi_config.enableScanAndListWifiNetworks)
    {
        Logger::get().logInfo(networkListToString(wifi).c_str());
    }

    Logger::get().logInfo("Attempting to connect to SSID: %s", wifi_config.ssid);
    unsigned int retries = 0U;
    while ((wifi.begin(wifi_config.ssid, wifi_config.password) != WL_CONNECTED) &&
           (retries < wifi_config.max_retries_wifi))
    {
        Logger::get().logWarning("Status: %s. Attempting to reconnect...", wifiStatusToString(wifi.status()));
        delay(wifi_config.retry_delay_ms);
    }

    const bool wifi_connected = isConnectedToWiFi(wifi);
    if (wifi_connected)
    {
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
        wifi.lowPowerMode();
#elif defined(ARDUINO_SAMD_MKR1000)
        wifi.maxLowPowerMode();
#endif
        Logger::get().logInfo("Connected to wifi with IP: %s", ipToString(wifi.localIP()).c_str());
    }
    else
    {
        Logger::get().logError("Could not connect, giving up");
    }

    return wifi_connected;
}

bool connectToMqttBroker(MqttClient &mqttclient, const MqttConfig &mqtt_config, uint32_t &time_ready_for_data_transfer)
{
    Logger::get().logInfo("Attempting to (re)connect to the MQTT broker %s:%u", mqtt_config.server_ip,
                          mqtt_config.server_port);

    mqttclient.setUsernamePassword(mqtt_config.username, mqtt_config.password);
    mqttclient.connect(mqtt_config.server_ip, mqtt_config.server_port);
    delay(100U);

    if (mqttclient.connected())
    {
        Logger::get().logInfo("Connected to MQTT broker");
        time_ready_for_data_transfer = millis() + mqtt_config.mqtt_msg_send_delay_ms;
    }
    else
    {
        Logger::get().logWarning("Connection to MQTT broker failed: %s",
                                 mqttErrorCodeToString(mqttclient.connectError()));
    }

    return mqttclient.connected();
}

void print(const WeatherMeasurements &measurements)
{
    Serial.print(F("Temperature: "));
    Serial.print(measurements.temp_c);
    Serial.print(F("°C, Humidity: "));
    Serial.print(measurements.humidity);
    Serial.print(F("°C, Pressure: "));
    Serial.print(measurements.pressue_hpa);
    Serial.print(F("%, Heat index: "));
    Serial.print(measurements.heat_index);
    Serial.print(F(", PV voltage: "));
    Serial.println(measurements.pv_voltage);
}

void print(RTCZero &rtc, const uint8_t timezone_offset_h)
{
    char date_time[100U];
    snprintf(date_time, sizeof(date_time), "Date & time in CET: 20%u-%02u-%02u %02u:%u:%u", //
             rtc.getYear(), rtc.getMonth(), rtc.getDay(), rtc.getHours() + timezone_offset_h, rtc.getMinutes(),
             rtc.getSeconds());
    Serial.println(date_time);
}

std::string createJsonStringFromMeasurement(const WeatherMeasurements &measurements)
{
    char json_str[200U];
    snprintf(json_str, sizeof(json_str),
             "{\"timestamp\": %lu, \"temperature\": %.2f,"
             "\"humidity\": %.2f, \"pressure\": %.2f,  \"heat_index\": %.2f,"
             "\"pv_voltage\": %.2f}",
             measurements.timestamp, measurements.temp_c, measurements.humidity, measurements.pressue_hpa,
             measurements.heat_index, measurements.pv_voltage);

    return std::string{json_str};
}

void sendWeatherMeasurements(MqttClient &mqttclient, const char *topic_measurements,
                             const WeatherMeasurements &measurements)
{
    if (!mqttclient.connected())
    {
        return;
    }

    mqttclient.beginMessage(topic_measurements);
    mqttclient.print(createJsonStringFromMeasurement(measurements).c_str());
    mqttclient.endMessage();
}

void sendWeatherMeasurements(MqttClient &mqttclient, const char *topic_measurements,
                             utils::Ringbuffer<WeatherMeasurements> &tx_buffer)
{
    if (!mqttclient.connected())
    {
        return;
    }

    while (!tx_buffer.empty())
    {
        sendWeatherMeasurements(mqttclient, topic_measurements, tx_buffer.peek());
        tx_buffer.pop();
    }
}

bool isFloatValueValid(const float value)
{
    return !(isnan(value) || isinf(value));
}

bool areMeasurementsPlausible(const WeatherMeasurements &measurements)
{
    return (isFloatValueValid(measurements.temp_c) &&                                                      //
            (measurements.temp_c > -40.0F) && (measurements.temp_c <= 50.0F) &&                            //
            isFloatValueValid(measurements.humidity) &&                                                    //
            (measurements.humidity > 0.0F) && (measurements.humidity <= 100.0F) &&                         //
            isFloatValueValid(measurements.pressue_hpa) &&                                                 //
            (measurements.pressue_hpa >= 900.0F) && (measurements.pressue_hpa <= PRESSURE_SEALEVEL_HPA) && //
            isFloatValueValid(measurements.pv_voltage) &&                                                  //
            (measurements.pv_voltage >= 0.0F && measurements.pv_voltage <= 14.0F));
}

bool readyToSchedule(const unsigned long next_schedule_interval)
{
    return millis() >= next_schedule_interval;
}
