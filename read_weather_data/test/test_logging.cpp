#include "../logging.h"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include <string>

namespace
{

constexpr uint32_t kTimestamp = 10042U;

TEST_CASE("Test serial logging")
{
    Serial.output.clear();
    Logger::get().setTimeFunction([]() { return kTimestamp; });

    GIVEN("Serial backend with log level INFO")
    {
        SerialLoggingBackend serial_logging{LogLevel::INFO};
        Logger::get().setLoggingBackends({&serial_logging});

        THEN("messages at or above INFO are printed with their level")
        {
            Logger::get().logInfo("temperature %d C", 21);
            Logger::get().logError("sensor %s failed", "BME280");
            CHECK(Serial.output == "INFO: temperature 21 C\nERROR: sensor BME280 failed\n");
        }

        THEN("messages below INFO are dropped")
        {
            Logger::get().logDebug("not shown");
            CHECK(Serial.output.empty());
        }
    }

    GIVEN("Serial backend that hides the log level")
    {
        SerialLoggingBackend serial_logging{LogLevel::DEBUG, false};
        Logger::get().setLoggingBackends({&serial_logging});

        THEN("only the message is printed")
        {
            Logger::get().logWarning("low battery");
            CHECK(Serial.output == "low battery\n");
        }
    }

    Logger::get().setLoggingBackends({});
}

TEST_CASE("Test MQTT logging")
{
    Logger::get().setTimeFunction([]() { return kTimestamp; });
    MqttClient mqtt_client;
    MqttLoggingBackend mqtt_logging{LogLevel::WARNING, mqtt_client, "weather/log"};
    Logger::get().setLoggingBackends({&mqtt_logging});

    GIVEN("connected MQTT client")
    {
        THEN("messages at or above WARNING are published as JSON")
        {
            Logger::get().logWarning("wifi rssi %d", -80);
            REQUIRE(mqtt_client.messages.size() == 1);
            CHECK(mqtt_client.messages[0].topic == "weather/log");
            CHECK(mqtt_client.messages[0].payload == R"({"timestamp":10042,"level":"WARNING","msg":"wifi rssi -80"})");
        }

        THEN("messages below WARNING are dropped")
        {
            Logger::get().logInfo("not published");
            CHECK(mqtt_client.messages.empty());
        }
    }

    GIVEN("disconnected MQTT client")
    {
        mqtt_client.is_connected = false;

        THEN("messages are buffered and sent in order once connected again")
        {
            Logger::get().logError("first");
            Logger::get().logFatal("second");
            CHECK(mqtt_client.messages.empty());

            mqtt_client.is_connected = true;
            mqtt_logging.flushData();
            REQUIRE(mqtt_client.messages.size() == 2);
            CHECK(mqtt_client.messages[0].payload == R"({"timestamp":10042,"level":"ERROR","msg":"first"})");
            CHECK(mqtt_client.messages[1].payload == R"({"timestamp":10042,"level":"FATAL","msg":"second"})");
        }
    }

    Logger::get().setLoggingBackends({});
}

} // namespace
