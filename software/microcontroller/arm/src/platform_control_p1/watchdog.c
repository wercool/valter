#include "watchdog.h"
#include "Board.h"

/**
  \defgroup Watchdog
  The Watchdog timer resets the board in the event that it's not behaving as expected.
  This is more robust than using other kinds of timers, because in the worst case, when
  your app has malfunctioned, it can still reset since it's not relying on your app to
  actually be running.

  If you want to use it, specify the length of the countdown to watchdogEnable() and then
  periodically call watchdogReset() to reset the countdown.  If the countdown ever gets
  all the way to zero, the board will reset.

  \b Example
  \code
  watchdogEnable(2000); // start the countdown

  void MyTask()
  {
    while (1) {
      if (everything_is_normal()) {
        watchdogReset();
      }
      else {
        // if things are not normal, the timer is not reset and will eventually expire
      }
    }
  }
  \endcode
  \ingroup Core
  @{
*/

/**
  Enable the watchdog timer.
  Specify the number of milliseconds that the watchdog should wait before 
  resetting.  Remember that watchdogEnable() or watchdogDisable() can only be called
  once until the processor is reset again.

  The maximum countdown length is 16 seconds (16000 ms).
  @param millis The number of milliseconds in which a reset will occur.
*/
void watchdogEnable(int millis)
{
  int period = (millis * 256) / 1000;
  AT91C_BASE_WDTC->WDTC_WDMR =  AT91C_WDTC_WDRSTEN |        // enable reset on timeout
                                AT91C_WDTC_WDDBGHLT |       // respect debug mode
                                AT91C_WDTC_WDIDLEHLT |      // respect idle mode
                                ((period << 16 ) & AT91C_WDTC_WDD) | // delta is as wide as the period, so we can restart anytime
                                (period & AT91C_WDTC_WDV);  // set the period
}

/**
  Reset the watchdog timer countdown.
  Call watchdogEnable() first, and then call this occasionally to reset
  the watchdog countdown so that it doesn't expire.
*/
void watchdogReset()
{
  AT91C_BASE_WDTC->WDTC_WDCR = 0xA5000001;
}

/**
  Disable the watchdog timer.
  Turn the watchdog off completely if you don't need it.
*/
void watchdogDisable()
{
  AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDDIS;
}

/** @}
*/
