#ifndef READ_WEATHER_DATA_H
#define READ_WEATHER_DATA_H

enum class WeatherSensor
{
    DHT22,
    BME280
};

struct AdcConfig
{
    float v_ref;
    unsigned int adc_bit_resolution;
    unsigned int num_samples;
    float voltage_divider_factor[8];
};

#endif // READ_WEATHER_DATA_H
