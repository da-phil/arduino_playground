#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>

constexpr float AVERAGE_PRESSURE_AT_SEALEVEL_HPA{1013.25};

Adafruit_BME280 bme_sensor;

void setup()
{
    Serial.begin(9600);

    while (!Serial)
    {
        delay(10);
    }

    Serial.print("status: ");
    bool status = bme_sensor.begin();
    Serial.println(status);
    while (!status)
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x");
        Serial.println(bme_sensor.sensorID(), 16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("        ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");

        delay(1500);
        status = bme_sensor.begin();
    }
}

void loop()
{
    if (isnan(bme_sensor.readTemperature()) || isnan(bme_sensor.readHumidity()) || isnan(bme_sensor.readPressure()))
    {
        Serial.println("Some measurements contain NaNs...");
    }

    const float temp_c = bme_sensor.readTemperature();
    const float pressure = bme_sensor.readPressure() / 100.0F;
    const float humidity = bme_sensor.readHumidity();
    const String measurements = String("temp: ") + String(temp_c) + String(", humidity: ") + String(humidity) +
                                String(", pressure: ") + String(pressure) + String(", altitude: ") +
                                String(bme_sensor.readAltitude(AVERAGE_PRESSURE_AT_SEALEVEL_HPA));
    Serial.println(measurements);

    delay(1000U);
}
