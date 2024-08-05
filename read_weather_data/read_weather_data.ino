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

#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>

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
#include "logging.h"
#include "read_weather_data.h"
#include "ringbuffer.h"
#include "scheduler.h"
#include "utils.h"

// General config
constexpr uint32_t BUSY_LOOP_DELAY_MS{10U};
constexpr uint32_t RETRY_DELAY_MS{500U};
constexpr uint8_t MAX_RETRIES_WIFI{2U};
constexpr uint32_t TX_BUFFER_SIZE{200U};
constexpr bool PRINT_MEASUREMENTS{true};
constexpr uint32_t SAMPLING_TASK_INTERVAL_MS{2U * 1000U};
constexpr uint32_t DATA_TRANSMISSION_TASK_INTERVAL_MS{15U * 1000U};
constexpr uint8_t SAMPLING_TASK_SLOWDOWN_FACTOR_WHILE_DISCONNECTED{2U};
constexpr uint32_t MAX_NTP_RTC_TIMEDIFF_S{10U};

// This delay accounts for leaving MQTT clients (particularily Telegraf)
// enoough time to re-connect once connection to the broker was lost
constexpr uint32_t MQTT_MSG_SEND_DELAY_MS{8000U};
constexpr WeatherSensor WEATHER_SENSOR{WeatherSensor::BME280};

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
                                 .enableListMacAddress = true,
                                 .enableScanAndListWifiNetworks = false,
                                 .max_retries_wifi = MAX_RETRIES_WIFI,
                                 .retry_delay_ms = RETRY_DELAY_MS};

// MQTT config
constexpr MqttConfig mqtt_config{.server_ip = SECRETS_MQTT_SERVER_IP,
                                 .server_port = SECRETS_MQTT_SERVER_PORT,
                                 .username = SECRETS_MQTT_USERNAME,
                                 .password = SECRETS_MQTT_PASSWORD};
#define SW_VERSION "dev"
#define LOCATION "balkon"
constexpr const char *TOPIC_MEASUREMENTS = "watering/" LOCATION "/" SW_VERSION "/measurements";
constexpr const char *TOPIC_LOGGING = "watering/" LOCATION "/" SW_VERSION "/logging";

// DHT22 sensor config
constexpr uint8_t DHTPIN = 5U; // Digital pin connected to the DHT sensor

///////////////////////////////////////////////////////////////////////////////////////////
// Global variables & helper functions
///////////////////////////////////////////////////////////////////////////////////////////
// Either use DHT22 (relatively inaccurate) or BME280 sensor:
DHT dht(DHTPIN, DHT22);
Adafruit_BME280 bme_sensor;

// Initialize WiFi & MQTT broker clients
WiFiClient wificlient;
MqttClient mqttclient(wificlient);
WiFiUDP wifi_udp_client;

// Initialize NTP and RTC client
constexpr int8_t UTC_TO_CET_OFFSET_H{2};
constexpr long GMT_TO_UTC_OFFSET_S{UTC_TO_CET_OFFSET_H * 60 * 60};
constexpr unsigned long NTP_UPDATE_INTERVAL{10U * 60U * 1000U};
NTPClient ntp_client(wifi_udp_client, "pool.ntp.org", 0, NTP_UPDATE_INTERVAL);
RTCZero rtc;

utils::Ringbuffer<WeatherMeasurements> tx_buffer{TX_BUFFER_SIZE};
SerialLoggingBackend serial_logging{LogLevel::INFO};
MqttLoggingBackend mqtt_logging{LogLevel::INFO, mqttclient, TOPIC_LOGGING};

float getSolarPanelVoltage()
{
    // read the input on analog pin 0 and convert to voltage range (0 - VREF):
    const int analog_val = analogRead(A0);
    const float adc_voltage = analog_val * (VREF / static_cast<float>(ADC_NUM_SAMPLES));
    const float pv_voltage = adc_voltage / VOLTAGE_DIVIDER_FACTOR;
    return pv_voltage;
}

WeatherMeasurements takeMeasurements(DHT &dht, RTCZero &rtc)
{
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    const float humidity = dht.readHumidity();
    const float temp_c = dht.readTemperature();

    WeatherMeasurements measurements{.timestamp = rtc.getEpoch(),
                                     .is_valid = false,
                                     .temp_c = temp_c,
                                     .humidity = humidity,
                                     .pressue_hpa = PRESSURE_SEALEVEL_HPA,
                                     .heat_index = dht.computeHeatIndex(temp_c, humidity, false),
                                     .pv_voltage = getSolarPanelVoltage()};
    measurements.is_valid = areMeasurementsPlausible(measurements);

    return measurements;
}

WeatherMeasurements takeMeasurements(Adafruit_BME280 &bme_sensor, RTCZero &rtc)
{
    const bool read_sensor_success = bme_sensor.takeForcedMeasurement();
    const float temp_c = bme_sensor.readTemperature();
    const float pressure_pha = bme_sensor.readPressure() / 100.0F;
    const float humidity = bme_sensor.readHumidity();

    WeatherMeasurements measurements{.timestamp = rtc.getEpoch(),
                                     .is_valid = false,
                                     .temp_c = temp_c,
                                     .humidity = humidity,
                                     .pressue_hpa = pressure_pha,
                                     .heat_index = dht.computeHeatIndex(temp_c, humidity, false),
                                     .pv_voltage = getSolarPanelVoltage()};
    measurements.is_valid = read_sensor_success && areMeasurementsPlausible(measurements);

    return measurements;
}

bool initializeWeatherSensor(DHT &dht_sensor, Adafruit_BME280 &bme280_sensor, WeatherSensor weather_sensor_type)
{
    // init weather sensor of choice
    bool sensor_found = false;
    if (weather_sensor_type == WeatherSensor::DHT22)
    {
        dht_sensor.begin();
        sensor_found = true;
    }
    else if (weather_sensor_type == WeatherSensor::BME280)
    {
        sensor_found = bme_sensor.begin();
        if (sensor_found)
        {
            bme_sensor.setSampling(Adafruit_BME280::MODE_FORCED,
                                   Adafruit_BME280::SAMPLING_X4, // temperature
                                   Adafruit_BME280::SAMPLING_X1, // pressure
                                   Adafruit_BME280::SAMPLING_X4, // humidity
                                   Adafruit_BME280::FILTER_X4,   // IIR filter
                                   Adafruit_BME280::STANDBY_MS_250);
        }
        else
        {
            Logger::get().logError(
                "Couldn't find a valid BME280 sensor, check wiring, address, sensor ID! SensorID was: 0x%x",
                bme_sensor.sensorID());
            //  ID of 0xFF probably means a bad address, a BMP 180 or BMP 085
            //  ID of 0x56-0x58 represents a BMP 280
            //  ID of 0x60 represents a BME 280
            //  ID of 0x61 represents a BME 680
        }
    }
    else
    {
        sensor_found = false;
        Logger::get().logError("No weather sensor was chosen in config!!!");
    }

    return sensor_found;
}

void updateRtcFromNtp(NTPClient &ntp_client, RTCZero &rtc, bool is_first_update = false)
{
    // try to update time from NTP server (once every NTP_UPDATE_INTERVAL seconds)
    const bool ntp_update_is_due{ntp_client.update()};
    const bool ntp_update_successfull{ntp_client.isTimeSet()};

    if (ntp_update_is_due && ntp_update_successfull)
    {
        const auto epoch_update_from_ntp_s{ntp_client.getEpochTime()};
        const auto epoch_time_rtc_s{rtc.getEpoch()};
        const auto time_diff_s{static_cast<int32_t>(epoch_update_from_ntp_s) - static_cast<int32_t>(epoch_time_rtc_s)};
        if ((static_cast<uint32_t>(abs(time_diff_s)) > MAX_NTP_RTC_TIMEDIFF_S) && !is_first_update)
        {
            Logger::get().logWarning("NTP time (%lu) significantly differs from RTC time (%ld)",
                                     epoch_update_from_ntp_s, epoch_time_rtc_s);
        }

        // if updated, also update RTC
        rtc.setEpoch(epoch_update_from_ntp_s);
        Logger::get().logInfo("RTC time was updated from NTP server");
    }
}

void samplingTask()
{
    WeatherMeasurements current_measurements;
    if (WEATHER_SENSOR == WeatherSensor::DHT22)
    {
        current_measurements = takeMeasurements(dht, rtc);
    }
    else
    {
        current_measurements = takeMeasurements(bme_sensor, rtc);
    }

    if (current_measurements.is_valid)
    {
        tx_buffer.push(current_measurements);
        if (PRINT_MEASUREMENTS)
        {
            print(rtc, UTC_TO_CET_OFFSET_H);
            print(current_measurements);
        }
    }
    else
    {
        Logger::get().logError("Failed to read a valid measurement from weather sensor!");
    }
}

void networkConnectionTask()
{
    if (!isConnectedToWiFi(WiFi))
    {
        Logger::get().logWarning(wifiStatusToString(WiFi.status()));
        connectToWiFi(WiFi, wifi_config);
    }
    else
    {
        updateRtcFromNtp(ntp_client, rtc, false);
    }

    bool connection_established{mqttclient.connected()};
    if (!connection_established)
    {
        Logger::get().logWarning(mqttErrorCodeToString(mqttclient.connectError()));
        connection_established = connectToMqttBroker(mqttclient, mqtt_config);
    }
    else
    {
        // call poll() regularly to allow the library to send MQTT keep alives which
        // avoids being disconnected by the broker
        mqttclient.poll();
    }
}
Scheduler<unsigned long> network_connection_task{BUSY_LOOP_DELAY_MS, millis, networkConnectionTask};
Scheduler<unsigned long> sampling_task{SAMPLING_TASK_INTERVAL_MS, millis, samplingTask};
Scheduler<unsigned long> mqtt_sender_task{DATA_TRANSMISSION_TASK_INTERVAL_MS, millis,
                                          []() { sendWeatherMeasurements(mqttclient, TOPIC_MEASUREMENTS, tx_buffer); }};

///////////////////////////////////////////////////////////////////////////////////////////
//
// Setup function - This routine runs once when you press reset
//
///////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);

    if (ENABLE_WAIT_FOR_CONNECTED_TERMINAL)
    {
        while (!Serial)
        {
            delay(BUSY_LOOP_DELAY_MS);
        }
    }

    rtc.begin();

    Logger::get().setLoggingBackends({&serial_logging, &mqtt_logging});
    Logger::get().setTimeFunction([]() { return rtc.getEpoch(); });

    // Configure ADC
    analogReadResolution(ADC_RES_BITS);
    analogReference(AR_DEFAULT);

    // Configure weather sensor
    const bool sensor_initialized = initializeWeatherSensor(dht, bme_sensor, WEATHER_SENSOR);
    if (!sensor_initialized)
    {
        Logger::get().logFatal("Weather sensor could not be initialized, can not continue!");
        while (true)
        {
            delay(BUSY_LOOP_DELAY_MS);
        }
    }

    connectToWiFi(WiFi, wifi_config);
    ntp_client.begin();
    connectToMqttBroker(mqttclient, mqtt_config);
    updateRtcFromNtp(ntp_client, rtc, true);
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Main loop
//
///////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
    // Wifi & MQTT connection task - freerunning
    bool connection_established_before{mqttclient.connected()};
    network_connection_task.executeIfReady();
    if (!connection_established_before && mqttclient.connected())
    {
        mqtt_sender_task.extendCurrentInterval(MQTT_MSG_SEND_DELAY_MS);
    }

    // Measurement sampling task
    sampling_task.executeIfReady();

    sampling_task.setInterval(mqttclient.connected()
                                  ? SAMPLING_TASK_INTERVAL_MS
                                  : SAMPLING_TASK_SLOWDOWN_FACTOR_WHILE_DISCONNECTED * SAMPLING_TASK_INTERVAL_MS);

    // MQTT sender task
    mqtt_sender_task.executeIfReady();
}
