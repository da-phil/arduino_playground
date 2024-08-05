#include "../scheduler.h"
#include "doctest.h"

namespace
{

template <typename T = unsigned long>
class TestTimer
{
  public:
    TestTimer(T time_init) : time_{time_init}
    {
    }

    T getTime() const
    {
        return time_;
    }

    void incrementTimeBy(const T time_increment)
    {
        time_ += time_increment;
    }

  private:
    T time_;
};

TEST_CASE("Test scheduler")
{
    GIVEN("Timer witih start time = 100 and schdule interval = 10")
    {
        TestTimer timer{100U};
        const unsigned long schedule_interval{10U};
        bool executed{false};

        Scheduler<unsigned long> test_scheduler{schedule_interval,
                                                [&timer]() -> unsigned int { return timer.getTime(); },
                                                [&executed]() { executed = true; }};
        WHEN("interval is updated to 12")
        {
            test_scheduler.setInterval(12U);
            THEN("interval is correctly set")
            {
                CHECK(test_scheduler.getInterval() == 12U);
            }
        }

        WHEN("time is incremented by 3 and executeIfReady() is called")
        {
            executed = false;
            timer.incrementTimeBy(3);
            test_scheduler.executeIfReady();
            THEN("not executed")
            {
                CHECK(!executed);
            }
        }

        WHEN("time is incremented by interval time (10) and executeIfReady() is called")
        {
            executed = false;
            timer.incrementTimeBy(10);
            test_scheduler.executeIfReady();
            THEN("executed")
            {
                CHECK(executed);
            }
        }

        WHEN("time is incremented by 100 and executeIfReady() is called")
        {
            executed = false;
            timer.incrementTimeBy(100);
            test_scheduler.executeIfReady();
            THEN("executed")
            {
                CHECK(executed);
            }

            AND_WHEN("interval time is extended significantly")
            {
                executed = false;
                test_scheduler.extendCurrentInterval(150);

                THEN("not executed")
                {
                    CHECK(!executed);
                }
            }
        }
    }
}

} // namespace
