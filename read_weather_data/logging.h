#ifndef ARDUINO_PLAYGROUND_LOGGING_H
#define ARDUINO_PLAYGROUND_LOGGING_H

#include <ArduinoMqttClient.h>
#include <api/String.h>
#include <functional>
#include <vector>

#include "ringbuffer.h"

enum class LogLevel
{
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    FATAL,
    SILENT
};

inline const char *toString(LogLevel log_level)
{
    switch (log_level)
    {
    case LogLevel::DEBUG:
        return "DEBUG";
    case LogLevel::INFO:
        return "INFO";
    case LogLevel::WARNING:
        return "WARNING";
    case LogLevel::ERROR:
        return "ERROR";
    case LogLevel::FATAL:
        return "FATAL";
    case LogLevel::SILENT:
        return "SILENT";
    default:
        return "BUG: Verbosity level unknown";
    }
}

class ILoggingBackend
{
  public:
    ILoggingBackend() = default;
    virtual ~ILoggingBackend() = default;

    virtual void log(const LogLevel verbosity, const uint32_t timestamp, const char *msg) = 0;
    virtual LogLevel getLogLevel() const = 0;
};

class SerialLoggingBackend : public ILoggingBackend
{
  public:
    SerialLoggingBackend(const LogLevel log_level, const bool show_log_level = true)
        : min_allowed_log_level_{log_level}, show_log_level_{show_log_level}
    {
    }

    void log(const LogLevel log_level, const uint32_t timestamp, const char *msg) override
    {
        std::ignore = timestamp;
        if (log_level >= min_allowed_log_level_)
        {
            if (show_log_level_)
            {
                Serial.print(toString(log_level));
                Serial.print(": ");
            }
            Serial.println(msg);
        }
    }

    LogLevel getLogLevel() const override
    {
        return min_allowed_log_level_;
    }

  private:
    LogLevel min_allowed_log_level_;
    bool show_log_level_;
};

class MqttLoggingBackend : public ILoggingBackend
{
  public:
    MqttLoggingBackend(const LogLevel log_level, MqttClient &mqttclient, const char *mqtt_logging_topic)
        : min_allowed_log_level_{log_level}, mqttclient_{mqttclient}, mqtt_logging_topic_{mqtt_logging_topic}
    {
    }

    void log(const LogLevel log_level, const uint32_t timestamp, const char *msg) override
    {
        if (log_level >= min_allowed_log_level_)
        {
            char json_str[MAX_MSG_LENGTH];
            snprintf(json_str, sizeof(json_str), "{\"timestamp\":%lu,\"level\":\"%s\",\"msg\":\"%s\"}", timestamp,
                     toString(log_level), msg);
            tx_buffer_.push(String{json_str, MAX_MSG_LENGTH});
        }

        flushData();
    }

    void flushData()
    {
        if (!mqttclient_.connected())
        {
            return;
        }
        
        String str;
        while (tx_buffer_.pop(str))
        {
            mqttclient_.beginMessage(mqtt_logging_topic_);
            mqttclient_.print(str.c_str());
            mqttclient_.endMessage();
        }
    }

    LogLevel getLogLevel() const override
    {
        return min_allowed_log_level_;
    }

  private:
    static const unsigned int MAX_MSG_LENGTH{160U};

    LogLevel min_allowed_log_level_;
    MqttClient &mqttclient_;
    const char *mqtt_logging_topic_;
    utils::Ringbuffer<String> tx_buffer_{16U};
};

class Logger
{
  public:
    using LoggingBackends = std::vector<ILoggingBackend *>;
    using UnixEpochTimestamp = uint32_t;

    Logger(Logger &) = delete;
    Logger(Logger &&) = delete;
    Logger operator=(Logger &) = delete;
    Logger operator=(Logger &&) = delete;

    static Logger &get()
    {
        static Logger instance;
        return instance;
    }

    void setLoggingBackends(const LoggingBackends &logger_backends)
    {
        logger_backends_ = logger_backends;
    }

    void setTimeFunction(const std::function<UnixEpochTimestamp(void)> &time_function)
    {
        time_function_ = time_function;
    }

    template <typename... Args>
    void log(const LogLevel log_level, const char *format, const Args... args)
    {
        char buffer[LOG_MSG_MAX_SIZE];
        const uint32_t timestamp = time_function_ ? time_function_() : 0U;
        snprintf(buffer, sizeof(buffer), format, args...);
        for (ILoggingBackend *logger_backend : logger_backends_)
            logger_backend->log(log_level, timestamp, buffer);
    }

    template <typename... Args>
    void logDebug(const char *format, const Args... args)
    {
        log(LogLevel::DEBUG, format, args...);
    }

    template <typename... Args>
    void logInfo(const char *format, const Args... args)
    {
        log(LogLevel::INFO, format, args...);
    }

    template <typename... Args>
    void logWarning(const char *format, const Args... args)
    {
        log(LogLevel::WARNING, format, args...);
    }

    template <typename... Args>
    void logError(const char *format, const Args... args)
    {
        log(LogLevel::ERROR, format, args...);
    }

    template <typename... Args>
    void logFatal(const char *format, const Args... args)
    {
        log(LogLevel::FATAL, format, args...);
    }

  private:
    Logger() = default;

    const unsigned int LOG_MSG_MAX_SIZE{120U};
    LoggingBackends logger_backends_;
    std::function<UnixEpochTimestamp(void)> time_function_;
};

#endif // ARDUINO_PLAYGROUND_LOGGING_H
