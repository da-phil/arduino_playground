// Stand-in for the ArduinoMqttClient library so sketch headers compile in host unit tests.
#ifndef ARDUINO_PLAYGROUND_TEST_MOCK_ARDUINO_MQTT_CLIENT_H
#define ARDUINO_PLAYGROUND_TEST_MOCK_ARDUINO_MQTT_CLIENT_H

#include "Arduino.h"

#include <string>
#include <vector>

// Records every published message so tests can check it.
class MqttClient
{
  public:
    struct Message
    {
        std::string topic;
        std::string payload;
    };

    int connected()
    {
        return is_connected ? 1 : 0;
    }

    int beginMessage(const char *topic)
    {
        current_ = Message{topic, ""};
        return 1;
    }

    size_t print(const char *text)
    {
        current_.payload += text;
        return current_.payload.size();
    }

    int endMessage()
    {
        messages.push_back(current_);
        return 1;
    }

    bool is_connected{true};
    std::vector<Message> messages;

  private:
    Message current_;
};

#endif // ARDUINO_PLAYGROUND_TEST_MOCK_ARDUINO_MQTT_CLIENT_H
