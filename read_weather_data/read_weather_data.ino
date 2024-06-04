/*
  Read weather data from DHT22 data and photovoltaik panel voltage:
  * reads PV voltage from analog input on pin A0 (14.6V max, voltage divider ratio: 0.24)
  * reads DHT22 humidity and temperature on digital input

  Instruction how to connect the DHT22 sensor:
  * Connect pin 1 (on the left) of the sensor to +5V
    NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1 to 3.3V instead of 5V!
  * Connect pin 2 of the sensor to whatever your DHTPIN is
  * Connect pin 4 (on the right) of the sensor to GROUND
  * Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor
*/

#include <ArduinoMqttClient.h>
#include <DHT.h>
#include <NTPClient.h>
#include <RTCZero.h>
#include <SPI.h>
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#endif
#include <WiFiUdp.h>
#include <cstdio>

#include "arduino_secrets.h"
#include "read_weather_data.h"
#include "utils.h"

constexpr unsigned long RETRY_DELAY_MS{500U};
constexpr unsigned int MAX_RETRIES_WIFI{2U};
constexpr bool PRINT_MEASUREMENTS{true};

// Serial config
constexpr unsigned long SERIAL_BAUDRATE{9600U};
constexpr bool ENABLE_WAIT_FOR_CONNECTED_TERMINAL{false};

// ADC config
constexpr float VREF = 3.3F;
constexpr unsigned int ADC_RES_BITS = 12U;
constexpr unsigned int ADC_NUM_SAMPLES = 2U << (ADC_RES_BITS - 1U);
constexpr float VOLTAGE_DIVIDER_FACTOR = 0.224F;

// Wifi config
constexpr WifiConfig wifi_config{.ssid = SECRETS_WIFI_SSID,
                                 .password = SECRETS_WIFI_PASSWORD,
                                 .enablePrintMacAddress = true,
                                 .enableScanAndListWifiNetworks = false};

// MQTT config
constexpr MqttConfig mqtt_config{.server_ip = SECRETS_MQTT_SERVER_IP,
                                 .server_port = SECRETS_MQTT_SERVER_PORT,
                                 .username = SECRETS_MQTT_USERNAME,
                                 .password = SECRETS_MQTT_PASSWORD};

#define BRANCH "dev"
#define LOCATION "balkon"
constexpr const char *topic_measurements = "watering/" LOCATION "/" BRANCH "/measurements";

// DHT22 sensor config
constexpr std::uint8_t DHTPIN = 5U; // Digital pin connected to the DHT sensor

DHT dht(DHTPIN, DHT22);

// Initialize WiFi & MQTT broker clients
WiFiClient wificlient;
MqttClient mqttclient(wificlient);
WiFiUDP wifi_udp_client;

// Initialize NTP and RTC client
constexpr long UTC_TO_CET_OFFSET_H{2};
constexpr long GMT_TO_UTC_OFFSET_S{UTC_TO_CET_OFFSET_H * 60 * 60};
constexpr unsigned long NTP_UPDATE_INTERVAL{120U};
NTPClient ntp_client(wifi_udp_client, "pool.ntp.org", 0, NTP_UPDATE_INTERVAL);
RTCZero rtc;

bool connectToWiFi(WiFiClass &wifi, const WifiConfig &wifi_config)
{
    if (wifi_config.enablePrintMacAddress)
    {
        Serial.println(printMacAddress(WiFi).c_str());
    }

    if (wifi_config.enableScanAndListWifiNetworks)
    {
        Serial.println(networkListToString(WiFi).c_str());
    }

    Serial.print("Attempting to connect to SSID: ");
    Serial.println(wifi_config.ssid);
    unsigned int retries = 0U;
    while ((wifi.begin(wifi_config.ssid, wifi_config.password) != WL_CONNECTED) && (retries < MAX_RETRIES_WIFI))
    {
        Serial.println(wifiStatusToString(wifi.status()));
        Serial.println("Attempt to connect again...");
        delay(RETRY_DELAY_MS);
    }

    if (isConnectedToWiFi(WiFi))
    {
        Serial.print("Connected to wifi with IP: ");
        Serial.println(IpToString(wifi.localIP()).c_str());
    }
    else
    {
        Serial.println("Could not connect, giving up");
    }
    return isConnectedToWiFi(wifi);
}

bool connectToMqttBroker(MqttClient &mqttclient, const MqttConfig &mqtt_config)
{
    Serial.print("Attempting to connect to the MQTT broker: ");
    Serial.println(mqtt_config.server_ip);

    mqttclient.setUsernamePassword(mqtt_config.username, mqtt_config.password);
    mqttclient.connect(mqtt_config.server_ip, mqtt_config.server_port);
    delay(100U);

    if (mqttclient.connected())
    {
        Serial.println("Connected to MQTT broker!");
    }
    else
    {
        Serial.print("Connection to MQTT broker failed with: ");
        Serial.println(mqttErrorCodeToString(mqttclient.connectError()));
    }

    return mqttclient.connected();
}

void printMeasurements(const Measurements &measurements)
{
    Serial.print(F("PV voltage: "));
    Serial.println(measurements.pv_voltage);
    Serial.print(F("Humidity: "));
    Serial.print(measurements.humidity);
    Serial.print(F("%  Temperature: "));
    Serial.print(measurements.temp_c);
    Serial.print(F("°C  Heat index: "));
    Serial.println(measurements.heat_index);
}

void printDateTime(RTCZero &rtc, const uint32_t timezone_offset_h)
{
    char date_time[100U];
    snprintf(date_time, sizeof(date_time), "Date & time in CET: 20%u-%02u-%02u %02u:%u:%u", //
             rtc.getYear(), rtc.getMonth(), rtc.getDay(), rtc.getHours() + timezone_offset_h, rtc.getMinutes(),
             rtc.getSeconds());
    Serial.println(date_time);
}

std::string createJsonStringFromMeasurement(const Measurements &measurements)
{
    char json_str[128U];
    snprintf(json_str, sizeof(json_str),
             "{\"timestamp\": %u, \"temperature\": %.2f,"
             "\"humidity\": %.2f, \"heat_index\": %.2f,"
             "\"pv_voltage\": %.2f}",
             measurements.timestamp, measurements.temp_c, measurements.humidity, measurements.heat_index,
             measurements.pv_voltage);

    return std::string{json_str};
}

void sendWeatherMeasurements(MqttClient &mqttclient, const char *topic_measurements, const Measurements &measurements)
{
    if (!mqttclient.connected())
    {
        return;
    }

    mqttclient.beginMessage(topic_measurements);
    mqttclient.print(createJsonStringFromMeasurement(measurements).c_str());
    mqttclient.endMessage();
}

///////////////////////////////////////////////////////////////////////////////////////////
// The setup routine runs once when you press reset
///////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);

    if (ENABLE_WAIT_FOR_CONNECTED_TERMINAL)
    {
        while (!Serial)
        {
            delay(10);
        }
    }

    // attempt to connect to WiFi network and MQTT broker
    connectToWiFi(WiFi, wifi_config);
    connectToMqttBroker(mqttclient, mqtt_config);

    ntp_client.begin();
    rtc.begin();

    // Configure ADC
    analogReadResolution(ADC_RES_BITS);
    analogReference(AR_DEFAULT);

    // init DHT driver
    dht.begin();
}

///////////////////////////////////////////////////////////////////////////////////////////
// Main loop
///////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
    if (!isConnectedToWiFi(WiFi))
    {
        Serial.println(wifiStatusToString(WiFi.status()));
        connectToWiFi(WiFi, wifi_config);
    }
    else
    {
        // try to update time from NTP server (once every NTP_UPDATE_INTERVAL seconds)
        if (ntp_client.update())
        {
            // if updated, also update RTC
            rtc.setEpoch(static_cast<uint32_t>(ntp_client.getEpochTime()));
        }
    }

    if (!mqttclient.connected())
    {
        Serial.println(mqttErrorCodeToString(mqttclient.connectError()));
        connectToMqttBroker(mqttclient, mqtt_config);
    }
    else
    {
        // call poll() regularly to allow the library to send MQTT keep alives which
        // avoids being disconnected by the broker
        mqttclient.poll();
    }

    // read the input on analog pin 0 and convert to voltage range (0 - VREF):
    const int analog_val = analogRead(A0);
    const float adc_voltage = analog_val * (VREF / static_cast<float>(ADC_NUM_SAMPLES));
    const float pv_voltage = adc_voltage / VOLTAGE_DIVIDER_FACTOR;

    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float humidity = dht.readHumidity();
    float temp_c = dht.readTemperature();
    if (isnan(humidity) || isinf(humidity) || isnan(temp_c) || isinf(temp_c))
    {
        Serial.println(F("Failed to read from DHT sensor!"));
        humidity = 0.0;
        temp_c = 0.0;
    }
    // Compute heat index in Celsius (isFahreheit = false)
    const float heat_index = dht.computeHeatIndex(temp_c, humidity, false);

    const Measurements current_measurements{.timestamp = rtc.getEpoch(),
                                            .temp_c = temp_c,
                                            .humidity = humidity,
                                            .heat_index = heat_index,
                                            .pv_voltage = pv_voltage};

    if (PRINT_MEASUREMENTS)
    {
        printDateTime(rtc, UTC_TO_CET_OFFSET_H);
        printMeasurements(current_measurements);
    }

    sendWeatherMeasurements(mqttclient, TOPIC_MEASUREMENTS, current_measurements);

    delay(1000);
}
