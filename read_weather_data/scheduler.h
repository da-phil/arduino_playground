#ifndef ARDUINO_PLAYGROUND_SCHEDULER_H
#define ARDUINO_PLAYGROUND_SCHEDULER_H

#include <functional>

template <typename T = unsigned long>
class Scheduler
{
  public:
    using TimeType = T;
    using TimeFunction = std::function<TimeType(void)>;
    using Callback = std::function<void(void)>;

    Scheduler(const TimeType interval_ms, const TimeFunction &time_function_ms, const Callback &callback)
        : interval_regular_ms_{interval_ms}, interval_current_cycle_ms_{interval_ms},
          time_function_ms_{time_function_ms}, callback_{callback},
          last_schedule_ms_{time_function_ms_ ? time_function_ms_() : 0}
    {
    }

    void setInterval(const T interval_ms)
    {
        interval_regular_ms_ = interval_ms;
    }

    T getInterval() const
    {
        return interval_regular_ms_;
    }

    void extendCurrentInterval(const T additional_time_ms)
    {
        interval_current_cycle_ms_ += additional_time_ms;
    }

    void executeIfReady()
    {        
        if (callback_ && isReadyForScheduling())
        {

            last_schedule_ms_ = time_function_ms_();
            callback_();
            interval_current_cycle_ms_ = interval_regular_ms_;
        }
    }

  private:
    bool isReadyForScheduling()
    {
        return (time_function_ms_() - last_schedule_ms_) >= interval_current_cycle_ms_;
    }

    TimeType interval_regular_ms_;
    TimeType interval_current_cycle_ms_;
    TimeFunction time_function_ms_;
    Callback callback_;
    TimeType last_schedule_ms_;
};

#endif // ARDUINO_PLAYGROUND_SCHEDULER_H
