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

#include <DHT.h>
#include <SPI.h>
#include <WiFi101.h>
#include <ArduinoMqttClient.h>

#include <cstdio>
#include "arduino_secrets.h"
#include "utils.h"
#include "read_weather_data.h"

// Serial config
constexpr unsigned long SERIAL_BAUDRATE{9600};
constexpr bool ENABLE_WAIT_FOR_CONNECTED_TERMINAL = false;

// ADC config
constexpr float VREF = 3.3F;
constexpr unsigned int ADC_RES_BITS = 12;
constexpr float ADC_NUM_SAMPLES = 2 << (ADC_RES_BITS - 1);
constexpr float VOLTAGE_DIVIDER_FACTOR = 0.224F;

// Wifi config
constexpr WifiConfig wifi_config{
    .ssid = SECRETS_WIFI_SSID,
    .password = SECRETS_WIFI_PASSWORD,
    .enablePrintMacAddress = true,
    .enableScanAndListWifiNetworks = false};

// MQTT config
constexpr MqttConfig mqtt_config{
    .server_ip = SECRETS_MQTT_SERVER_IP,
    .server_port = SECRETS_MQTT_SERVER_PORT,
    .username = SECRETS_MQTT_USERNAME,
    .password = SECRETS_MQTT_PASSWORD};

constexpr const char *topic_temperature = "watering/dev/temperature";
constexpr const char *topic_humidity = "watering/dev/humidity";
constexpr const char *topic_heat_index = "watering/dev/heat_index";
constexpr const char *topic_pv_voltage = "watering/dev/pv_voltage";

// DHT22 sensor config
constexpr std::uint8_t DHTPIN = 5; // Digital pin connected to the DHT sensor

DHT dht(DHTPIN, DHT22);

// Initialize WiFi & MQTT broker connection
WiFiClient wificlient;
MqttClient mqttclient(wificlient);

void connectToWiFi(WiFiClass &wifi, const WifiConfig &wifi_config)
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
    while (wifi.begin(wifi_config.ssid, wifi_config.password) != WL_CONNECTED)
    {
        Serial.println(wifiStatusToString(wifi.status()));
        Serial.println("Attempt to connect again...");
        delay(50);
    }

    Serial.print("Connected to wifi with IP: ");
    Serial.println(IpToString(wifi.localIP()).c_str());
}

void connectToMqttBroker(MqttClient &mqttclient, const MqttConfig &mqtt_config)
{
    Serial.print("Attempting to connect to the MQTT broker: ");
    Serial.println(mqtt_config.server_ip);

    mqttclient.setUsernamePassword(mqtt_config.username, mqtt_config.password);
    while (!mqttclient.connect(mqtt_config.server_ip, mqtt_config.server_port))
    {
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttclient.connectError());
        delay(1000);
    }
    Serial.println("MQTT connection established!");
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

    // attempt to connect to WiFi network:
    connectToWiFi(WiFi, wifi_config);

    // attempt to connect to MQTT broker
    connectToMqttBroker(mqttclient, mqtt_config);

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
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println(wifiStatusToString(WiFi.status()));
        connectToWiFi(WiFi, wifi_config);
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
    const float adc_voltage = analog_val * (VREF / ADC_NUM_SAMPLES);
    const float pv_voltage = adc_voltage / VOLTAGE_DIVIDER_FACTOR;
    Serial.print("PV voltage: ");
    Serial.println(pv_voltage);

    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float humidity = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float temp_c = dht.readTemperature();

    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity) || isinf(humidity) || isnan(temp_c) || isinf(temp_c))
    {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }

    // Compute heat index in Celsius (isFahreheit = false)
    float heat_index = dht.computeHeatIndex(temp_c, humidity, false);

    Serial.print(F("Humidity: "));
    Serial.print(humidity);
    Serial.print(F("%  Temperature: "));
    Serial.print(temp_c);
    Serial.print(F("°C  Heat index: "));
    Serial.println(heat_index);

    mqttclient.beginMessage(topic_temperature);
    mqttclient.print(temp_c);
    mqttclient.endMessage();
    mqttclient.beginMessage(topic_humidity);
    mqttclient.print(humidity);
    mqttclient.endMessage();
    mqttclient.beginMessage(topic_heat_index);
    mqttclient.print(heat_index);
    mqttclient.endMessage();
    mqttclient.beginMessage(topic_pv_voltage);
    mqttclient.print(pv_voltage);
    mqttclient.endMessage();

    delay(1000);
}