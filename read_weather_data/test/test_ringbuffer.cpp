#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "../ringbuffer.h"
#include "doctest.h"
#include <string>

namespace
{

TEST_CASE("Test simple ringbuffer functionality")
{
    GIVEN("Simple int ringbuffer")
    {
        utils::Ringbuffer<int> ringbuffer{12};
        const int random_value = 42;

        THEN("expect same max size as given during initialization")
        {
            CHECK(ringbuffer.getMaxSize() == 12);
            CHECK(ringbuffer.capacity() == 12);
        }

        THEN("expect empty ringbuffer")
        {
            CHECK(ringbuffer.empty());
            CHECK(!ringbuffer.full());
            CHECK(ringbuffer.getFillLevel() == 0);
        }

        WHEN("adding an element")
        {
            ringbuffer.push(random_value);

            THEN("fill level increases by 1")
            {
                CHECK(!ringbuffer.empty());
                CHECK(!ringbuffer.full());
                CHECK(ringbuffer.getFillLevel() == 1);
            }

            AND_WHEN("popping one value")
            {
                const int last_value = ringbuffer.peek();
                const bool popped_successful = ringbuffer.pop();

                THEN("popping succeeds, value is last one and fill level is zero again")
                {
                    CHECK(popped_successful);
                    CHECK(last_value == random_value);
                    CHECK(ringbuffer.getFillLevel() == 0);
                    CHECK(ringbuffer.empty());
                    CHECK(!ringbuffer.full());
                }
            }
        }

        WHEN("adding 9 elements")
        {
            for (int i = 0; i < 9; i++)
                ringbuffer.push(i);

            THEN("buffer contains 9 elements")
            {
                CHECK(ringbuffer.getFillLevel() == 9);
            }

            AND_WHEN("reset() is called")
            {
                ringbuffer.reset();

                THEN("buffer is empty")
                {
                    CHECK(ringbuffer.empty());
                }
            }
        }
    }
}

TEST_CASE("Test ringbuffer implementation edge cases")
{
    GIVEN("Simple int ringbuffer")
    {
        utils::Ringbuffer<int> ringbuffer{12};
        const int random_value = 42;

        WHEN("ringbuffer is filled to its capacity")
        {
            for (int i = 0; i < 12; i++)
                ringbuffer.push(i);

            THEN("fill level is equal to capacity")
            {
                CHECK(ringbuffer.getFillLevel() == 12);
                CHECK(ringbuffer.full());
            }

            AND_WHEN("one more value is pushed")
            {
                ringbuffer.push(42);
                THEN("ringbuffer is still full")
                {
                    CHECK(ringbuffer.full());
                }

                AND_WHEN("popping all values")
                {
                    for (int i = 0; i < 11; i++)
                    {
                        INFO("Evaluation at i=", i);
                        CHECK(ringbuffer.peek() == i + 1);
                        CHECK(ringbuffer.pop());
                    }

                    CHECK(ringbuffer.peek() == 42);
                    CHECK(ringbuffer.pop());

                    THEN("ringbuffer is empty")
                    {
                        CHECK(ringbuffer.empty());
                    }
                }
            }
        }
    }
}

} // namespace
