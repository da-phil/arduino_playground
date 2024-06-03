#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "../ringbuffer.h"
#include "doctest.h"

TEST_CASE("Test simple ringbuffer implementation")
{
    GIVEN("Simple int ringbuffer")
    {
        Ringbuffer<int, 12> ringbuffer;
        const int random_value = 42;

        THEN("expect same max size as given as template arg")
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
                int popped_value{0};
                const bool popped_successful = ringbuffer.pop(popped_value);

                THEN("popping succeeds, value is last one and fill level is zero again")
                {
                    CHECK(popped_successful);
                    CHECK(popped_value == random_value);
                    CHECK(ringbuffer.getFillLevel() == 0);
                    CHECK(ringbuffer.empty());
                    CHECK(!ringbuffer.full());
                }
            }
        }
    }
}

TEST_CASE("Test ringbuffer implementation edge cases")
{
    GIVEN("Simple int ringbuffer")
    {
        Ringbuffer<int, 12> ringbuffer;
        const int random_value = 42;

        WHEN("ringbuffer is filled to its capacity")
        {
            for (int i = 0; i < 12; i++)
            {
                ringbuffer.push(i);
            }

            THEN("fill level is equal to capacity")
            {
                CHECK(ringbuffer.getFillLevel() == 12);
                CHECK(ringbuffer.full());
            }

            AND_WHEN("one more value is pushed")
            {
                ringbuffer.push(42);
                THEN("ringbuffer is still full and first element is now 1")
                {
                    CHECK(ringbuffer.full());
                }

                AND_WHEN("popping all values")
                {
                    int popped_value;

                    for (int i = 0; i < 11; i++)
                    {
                        CHECK(ringbuffer.pop(popped_value));
                        CHECK(popped_value == i + 1);
                    }

                    CHECK(ringbuffer.pop(popped_value));
                    CHECK(popped_value == 42);

                    THEN("ringbuffer is empty")
                    {
                        CHECK(ringbuffer.empty());
                    }
                }
            }
        }
    }
}
