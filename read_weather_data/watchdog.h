#ifndef ARDUINO_PLAYGROUND_WATCHDOG_H
#define ARDUINO_PLAYGROUND_WATCHDOG_H

#include <stdint.h>

/// Hardware watchdog which resets the board if the main loop gets stuck.
///
/// The SAMD21 hardware watchdog can wait at most 16 s, but some library calls (WiFi.begin(),
/// MQTT connect) can legitimately block for longer. Therefore the hardware watchdog is cleared
/// from its own early-warning interrupt every 8 s, but only as long as feed() was called within
/// the last `timeout_ms`. If the main code hangs, e.g. in a busy-wait loop of a driver, feed() is
/// no longer called, the interrupt stops clearing the hardware watchdog and the board resets.
/// A hard fault also leads to a reset, because the fault handler blocks the interrupt.
namespace watchdog
{

/// Start the watchdog. The board resets if feed() is not called for `timeout_ms` (+ up to 16 s).
void enable(uint32_t timeout_ms);

/// Signal that the main loop is still alive.
void feed();

/// Human readable cause of the last reset, e.g. "watchdog" or "power-on".
const char *getResetCause();

} // namespace watchdog

#endif // ARDUINO_PLAYGROUND_WATCHDOG_H
