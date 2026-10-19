#include <Arduino.h>

#include "watchdog.h"

#if defined(ARDUINO_ARCH_SAMD) && !defined(__SAMD51__)

namespace
{
// GCLK0-3 are used by the Arduino core (GCLK2 is also re-configured by RTCZero)
constexpr uint8_t WDT_CLOCK_GENERATOR{4U};

volatile uint32_t last_feed_ms{0U};
volatile uint32_t feed_timeout_ms{0U};
} // namespace

// Early-warning interrupt, fires 8 s after the hardware watchdog was last cleared
extern "C" void WDT_Handler(void)
{
    WDT->INTFLAG.reg = WDT_INTFLAG_EW;
    if ((millis() - last_feed_ms) < feed_timeout_ms)
    {
        WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
    }
    // Otherwise the hardware watchdog expires 8 s later and resets the board
}

namespace watchdog
{

void enable(uint32_t timeout_ms)
{
    feed_timeout_ms = timeout_ms;
    last_feed_ms = millis();

    // Clock the watchdog with 1024 Hz: ultra low power 32.768 kHz oscillator divided by 2^(4+1)
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(WDT_CLOCK_GENERATOR) | GCLK_GENDIV_DIV(4);
    while (GCLK->STATUS.bit.SYNCBUSY)
    {
    }
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(WDT_CLOCK_GENERATOR) | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K |
                        GCLK_GENCTRL_DIVSEL;
    while (GCLK->STATUS.bit.SYNCBUSY)
    {
    }
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(WDT_CLOCK_GENERATOR);
    while (GCLK->STATUS.bit.SYNCBUSY)
    {
    }

    // The watchdog can only be configured while it is disabled
    WDT->CTRL.reg = 0U;
    while (WDT->STATUS.bit.SYNCBUSY)
    {
    }

    WDT->CONFIG.reg = WDT_CONFIG_PER_16K;     // reset after 16384 cycles = 16 s
    WDT->EWCTRL.reg = WDT_EWCTRL_EWOFFSET_8K; // early-warning interrupt after 8192 cycles = 8 s
    WDT->INTFLAG.reg = WDT_INTFLAG_EW;
    WDT->INTENSET.reg = WDT_INTENSET_EW;

    // Lowest priority (same as SysTick): if the code is stuck in any other interrupt handler,
    // the watchdog interrupt can't run and the board resets.
    NVIC_ClearPendingIRQ(WDT_IRQn);
    NVIC_SetPriority(WDT_IRQn, (1U << __NVIC_PRIO_BITS) - 1U);
    NVIC_EnableIRQ(WDT_IRQn);

    WDT->CTRL.reg = WDT_CTRL_ENABLE;
    while (WDT->STATUS.bit.SYNCBUSY)
    {
    }
}

void feed()
{
    last_feed_ms = millis();
}

const char *getResetCause()
{
    const uint8_t reset_cause = PM->RCAUSE.reg;
    if (reset_cause & PM_RCAUSE_WDT)
    {
        return "watchdog";
    }
    if (reset_cause & PM_RCAUSE_SYST)
    {
        return "software reset";
    }
    if (reset_cause & PM_RCAUSE_EXT)
    {
        return "reset pin";
    }
    if (reset_cause & PM_RCAUSE_BOD33)
    {
        return "brown-out 3.3V";
    }
    if (reset_cause & PM_RCAUSE_BOD12)
    {
        return "brown-out 1.2V";
    }
    if (reset_cause & PM_RCAUSE_POR)
    {
        return "power-on";
    }
    return "unknown";
}

} // namespace watchdog

#else

// No watchdog support for other architectures yet
namespace watchdog
{
void enable(uint32_t)
{
}

void feed()
{
}

const char *getResetCause()
{
    return "unknown";
}
} // namespace watchdog

#endif
