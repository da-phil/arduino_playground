#ifndef ARDUINO_PLAYGROUND_LOGGING_H
#define ARDUINO_PLAYGROUND_LOGGING_H

#include <ArduinoMqttClient.h>
#include <vector>

enum class VerbosityLevel
{
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    FATAL
};

struct VerbosityMask
{
    bool debug;
    bool info;
    bool warning;
    bool error;
    bool fatal;
};

inline const char *toString(VerbosityLevel verbosity_level)
{
    switch (verbosity_level)
    {
    case VerbosityLevel::DEBUG:
        return "DEBUG";
    case VerbosityLevel::INFO:
        return "INFO";
    case VerbosityLevel::WARNING:
        return "WARNING";
    case VerbosityLevel::ERROR:
        return "ERROR";
    case VerbosityLevel::FATAL:
        return "FATAL";
    default:
        return "BUG: Verbosity level unknown";
    }
}

class ILoggingBackend
{
  public:
    ILoggingBackend() = default;
    virtual ~ILoggingBackend() = default;

    virtual void log(const VerbosityLevel verbosity, const char *msg) = 0;
    virtual VerbosityMask getVerbosityMask() const = 0;
};

class SerialLoggingBackend : public ILoggingBackend
{
  public:
    SerialLoggingBackend(const VerbosityMask verbosity_mask) : verbosity_mask_{verbosity_mask}
    {
    }

    void log(const VerbosityLevel verbosity_level, const char *msg) override
    {
        if (((verbosity_level == VerbosityLevel::DEBUG) && verbosity_mask_.debug) ||
            ((verbosity_level == VerbosityLevel::INFO) && verbosity_mask_.info) ||
            ((verbosity_level == VerbosityLevel::WARNING) && verbosity_mask_.warning) ||
            ((verbosity_level == VerbosityLevel::ERROR) && verbosity_mask_.error) ||
            ((verbosity_level == VerbosityLevel::FATAL) && verbosity_mask_.fatal))
        {
            Serial.print(toString(verbosity_level));
            Serial.print(": ");
            Serial.println(msg);
        }
    }

    VerbosityMask getVerbosityMask() const override
    {
        return verbosity_mask_;
    }

  private:
    VerbosityMask verbosity_mask_;
};

class MqttLoggingBackend : public ILoggingBackend
{
  public:
    MqttLoggingBackend(const VerbosityMask verbosity_mask, MqttClient &mqttclient, const char *mqtt_logging_topic)
        : verbosity_mask_{verbosity_mask}, mqttclient_{mqttclient}, mqtt_logging_topic_{mqtt_logging_topic}
    {
    }

    void log(const VerbosityLevel verbosity_level, const char *msg) override
    {
        if (((verbosity_level == VerbosityLevel::DEBUG) && verbosity_mask_.debug) ||
            ((verbosity_level == VerbosityLevel::INFO) && verbosity_mask_.info) ||
            ((verbosity_level == VerbosityLevel::WARNING) && verbosity_mask_.warning) ||
            ((verbosity_level == VerbosityLevel::ERROR) && verbosity_mask_.error) ||
            ((verbosity_level == VerbosityLevel::FATAL) && verbosity_mask_.fatal))
        {
            if (!mqttclient_.connected())
            {
                return;
            }
            char json_str[200U];
            snprintf(json_str, sizeof(json_str), "{\"timestamp\": %u, \"severity\": %s, \"text\": %s}\n", 0U,
                     toString(verbosity_level), msg);
            mqttclient_.beginMessage(mqtt_logging_topic_);
            // mqttclient_.print(toString(verbosity_level));
            // mqttclient_.print(": ");
            // mqttclient_.println(msg);
            mqttclient_.print(json_str);
            mqttclient_.endMessage();
        }
    }

    VerbosityMask getVerbosityMask() const override
    {
        return verbosity_mask_;
    }

  private:
    VerbosityMask verbosity_mask_;
    MqttClient &mqttclient_;
    const char *mqtt_logging_topic_;
};

class Logger
{
  public:
    using LoggingBackends = std::vector<ILoggingBackend *>;

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

    template <typename... Args>
    void log(const VerbosityLevel verbosity_level, const char *format, const Args... args)
    {
        char buffer[LOG_MSG_MAX_SIZE];
        snprintf(buffer, sizeof(buffer), format, args...);
        for (ILoggingBackend *logger_backend : logger_backends_)
            logger_backend->log(verbosity_level, buffer);
    }

    template <typename... Args>
    void logDebug(const char *format, const Args... args)
    {
        log(VerbosityLevel::DEBUG, format, args...);
    }

    template <typename... Args>
    void logInfo(const char *format, const Args... args)
    {
        log(VerbosityLevel::INFO, format, args...);
    }

    template <typename... Args>
    void logWarning(const char *format, const Args... args)
    {
        log(VerbosityLevel::WARNING, format, args...);
    }

    template <typename... Args>
    void logError(const char *format, const Args... args)
    {
        log(VerbosityLevel::ERROR, format, args...);
    }

    template <typename... Args>
    void logFatal(const char *format, const Args... args)
    {
        log(VerbosityLevel::FATAL, format, args...);
    }

  private:
    Logger() = default;

    const unsigned int LOG_MSG_MAX_SIZE{120U};
    LoggingBackends logger_backends_;
};

#endif // ARDUINO_PLAYGROUND_LOGGING_H
