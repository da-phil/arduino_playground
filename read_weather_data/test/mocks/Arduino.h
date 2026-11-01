// Stand-in for the Arduino core so sketch headers compile in host unit tests.
// Only provides what the tested headers use.
#ifndef ARDUINO_PLAYGROUND_TEST_MOCK_ARDUINO_H
#define ARDUINO_PLAYGROUND_TEST_MOCK_ARDUINO_H

#include <cstdint>
#include <cstdio>
#include <string>
#include <tuple>

// Records everything printed so tests can check it.
class FakeSerial
{
  public:
    void print(const char *text)
    {
        output += text;
    }

    void println(const char *text)
    {
        output += text;
        output += "\n";
    }

    std::string output;
};

inline FakeSerial Serial;

#endif // ARDUINO_PLAYGROUND_TEST_MOCK_ARDUINO_H
