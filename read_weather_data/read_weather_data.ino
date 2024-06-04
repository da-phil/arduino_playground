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
#include "ringbuffer.h"
#include "utils.h"

// General config
constexpr uint32_t RETRY_DELAY_MS{500U};
constexpr uint8_t MAX_RETRIES_WIFI{2U};
constexpr uint32_t TX_BUFFER_SIZE{100U};
constexpr bool PRINT_MEASUREMENTS{true};
constexpr uint32_t SAMPLING_TASK_INTERVAL_MS{2000U};
// This delay accounts for leaving MQTT clients (particularily Telegraf)
// enoough time to re-connect once connection to the broker was lost
constexpr uint32_t MQTT_MSG_SEND_DELAY_MS{8000U};

// Serial config
constexpr uint32_t SERIAL_BAUDRATE{9600U};
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
                                 .enableScanAndListWifiNetworks = false,
                                 .max_retries_wifi = MAX_RETRIES_WIFI,
                                 .retry_delay_ms = RETRY_DELAY_MS};

// MQTT config
constexpr MqttConfig mqtt_config{.server_ip = SECRETS_MQTT_SERVER_IP,
                                 .server_port = SECRETS_MQTT_SERVER_PORT,
                                 .username = SECRETS_MQTT_USERNAME,
                                 .password = SECRETS_MQTT_PASSWORD,
                                 .mqtt_msg_send_delay_ms = MQTT_MSG_SEND_DELAY_MS};
#define SW_VERSION "dev"
#define LOCATION "balkon"
constexpr const char *TOPIC_MEASUREMENTS = "watering/" LOCATION "/" SW_VERSION "/measurements";

// DHT22 sensor config
constexpr uint8_t DHTPIN = 5U; // Digital pin connected to the DHT sensor

///////////////////////////////////////////////////////////////////////////////////////////
// Global variables & helper functions
///////////////////////////////////////////////////////////////////////////////////////////
DHT dht(DHTPIN, DHT22);

// Initialize WiFi & MQTT broker clients
WiFiClient wificlient;
MqttClient mqttclient(wificlient);
WiFiUDP wifi_udp_client;

// Initialize NTP and RTC client
constexpr int8_t UTC_TO_CET_OFFSET_H{2};
constexpr long GMT_TO_UTC_OFFSET_S{UTC_TO_CET_OFFSET_H * 60 * 60};
constexpr unsigned long NTP_UPDATE_INTERVAL{120U};
NTPClient ntp_client(wifi_udp_client, "pool.ntp.org", 0, NTP_UPDATE_INTERVAL);
RTCZero rtc;

utils::Ringbuffer<WeatherMeasurements> tx_buffer{TX_BUFFER_SIZE};

uint32_t next_schedule_send_data = 0U;

WeatherMeasurements takeMeasurements(DHT &dht, RTCZero &rtc)
{
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

    return WeatherMeasurements{.timestamp = rtc.getEpoch(),
                               .temp_c = temp_c,
                               .humidity = humidity,
                               .pressue_hpa = 0.0F,
                               .heat_index = heat_index,
                               .pv_voltage = pv_voltage};
}

///////////////////////////////////////////////////////////////////////////////////////////
// Setup function - This routine runs once when you press reset
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
    connectToMqttBroker(mqttclient, mqtt_config, next_schedule_send_data);

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
    static uint32_t next_schedule_sampling_task = millis() + SAMPLING_TASK_INTERVAL_MS;

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
        connectToMqttBroker(mqttclient, mqtt_config, next_schedule_send_data);
    }
    else
    {
        // call poll() regularly to allow the library to send MQTT keep alives which
        // avoids being disconnected by the broker
        mqttclient.poll();
    }

    if (readyToSchedule(next_schedule_sampling_task))
    {
        next_schedule_sampling_task += SAMPLING_TASK_INTERVAL_MS;

        const auto current_measurements = takeMeasurements(dht, rtc);
        tx_buffer.push(current_measurements);

        if (PRINT_MEASUREMENTS)
        {
            print(rtc, UTC_TO_CET_OFFSET_H);
            print(current_measurements);
        }
    }

    if (readyToSchedule(next_schedule_send_data))
    {
        next_schedule_send_data += SAMPLING_TASK_INTERVAL_MS;
        sendWeatherMeasurements(mqttclient, TOPIC_MEASUREMENTS, tx_buffer);
    }
}
