/*
  Read weather data from DHT22 data and photovoltaik panel voltage:
  * reads PV voltage from analog input on pin A0 (14.6V max, voltage divider ratio: 0.24)
  * reads DHT22 humidity and temperature on digital input
*/

#include <DHT.h>
#include <SPI.h>
#include <WiFi101.h>
#include <ArduinoMqttClient.h>

#include "arduino_secrets.h"
#include "utils.h"

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1 to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor
#define DHTPIN 5      // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22 // DH11 / DHT21 / DHT22

// ADC Config
const float VREF = 3.3F;
const unsigned int ADC_RES_BITS = 12;
const float ADC_NUM_SAMPLES = 2 << (ADC_RES_BITS - 1);
const float VOLTAGE_DIVIDER_FACTOR = 0.225F;

// WiFi config
const char *ssid = secrets_ssid; // network SSID (name)
const char *pass = secrets_pass; // network password

const char *mqtt_server_ip = secrets_mqtt_server_ip;   // IP address of the MQTT broker
const int mqtt_server_port = secrets_mqtt_server_port; // Port of MQTT broker
const char *mqtt_username = secrets_mqtt_username;     // MQTT username
const char *mqtt_password = secrets_mqtt_password;     // MQTT password
const char *topic_temperature = "watering/dev/temperature";
const char *topic_humidity = "watering/dev/humidity";
const char *topic_heat_index = "watering/dev/heat_index";
const char *topic_pv_voltage = "watering/dev/pv_voltage";

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);

// Initialize WiFi & MQTT broker connection
WiFiClient wificlient;
MqttClient mqttclient(wificlient);

bool enablePrintMacAddress = true;
bool enableScanAndListWifiNetworks = false;

void printMacAddress()
{
    // the MAC address of your Wifi shield
    byte mac[6];

    // print your MAC address:
    WiFi.macAddress(mac);

    Serial.print("MAC: ");
    Serial.print(mac[5], HEX);
    Serial.print(":");
    Serial.print(mac[4], HEX);
    Serial.print(":");
    Serial.print(mac[3], HEX);
    Serial.print(":");
    Serial.print(mac[2], HEX);
    Serial.print(":");
    Serial.print(mac[1], HEX);
    Serial.print(":");
    Serial.println(mac[0], HEX);
}

void listNetworks()
{
    // scan for nearby networks:
    Serial.println("** Scan Networks **");
    byte numSsid = WiFi.scanNetworks();

    // print the network number and name for each network found:
    for (int thisNet = 0; thisNet < numSsid; thisNet++)
    {
        Serial.print(thisNet);
        Serial.print(") ");
        Serial.print(WiFi.SSID(thisNet));
        Serial.print("\tSignal: ");
        Serial.print(WiFi.RSSI(thisNet));
        Serial.print(" dBm");
        Serial.print("\tEncryption: ");
        Serial.println(WiFi.encryptionType(thisNet));
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// The setup routine runs once when you press reset
void setup()
{
    // initialize serial communication at 9600 bits per second:
    Serial.begin(9600);
    while (!Serial)
    {
        delay(10);
    }

    if (enablePrintMacAddress)
    {
        printMacAddress();
    }
    if (enableScanAndListWifiNetworks)
    {
        listNetworks();
    }

    // attempt to connect to WiFi network:
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    while (WiFi.begin(ssid, pass) != WL_CONNECTED)
    {
        Serial.println("Establishing connection failed... Attempt again...");
        delay(50);
    }

    Serial.print("Connected to wifi with IP: ");
    Serial.println(IpToString(WiFi.localIP()).c_str());

    // attempt to connect to MQTT broker
    Serial.print("Attempting to connect to the MQTT broker: ");
    Serial.println(mqtt_server_ip);

    mqttclient.setUsernamePassword(mqtt_username, mqtt_password);
    while (!mqttclient.connect(mqtt_server_ip, mqtt_server_port))
    {
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttclient.connectError());
        delay(1000);
    }
    Serial.println("MQTT connection established!");

    // Configure ADC
    analogReadResolution(ADC_RES_BITS);
    analogReference(AR_DEFAULT);

    // init DHT driver
    dht.begin();
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Main loop
void loop()
{
    // call poll() regularly to allow the library to send MQTT keep alives which
    // avoids being disconnected by the broker
    mqttclient.poll();

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