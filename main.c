//                  +-\/-+
//    (reset) PB5  1|    |8  Vcc
//  LED debug PB3  2|    |7  PB2
//            PB4  3|    |6  PB1 (OCB1)  IR out
//            GND  4|    |5  PB0 (PCINT0)IR in
//                  +----+

#ifndef __AVR_ATtiny13A__
#define __AVR_ATtiny13A__
#endif // not needed when compile but needed when auto-complete of VSCode

#include <avr/io.h>
#include <avr/sleep.h>

#include "ir.h"
#include "mydelay.h"

void disableUnusedFunctions(void)
{
    // ADC is disabled by default
    ACSR |= _BV(ACD); // Analog Comparator Disable
    MCUSR &= ~_BV(WDRF);
    WDTCR |= _BV(WDCE) | _BV(WDE);
    WDTCR = 0x00; // WDT Disable
}

int main(void)
{
    DDRB = _BV(LED) | _BV(IROUT) | _BV(DDB4) | _BV(DDB2);
    initTimer();
    initIRIn();
    disableUnusedFunctions();
    disablePWM();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sei();
    while(1) {
        sleep_bod_disable();
        sleep_mode();
    }
}
