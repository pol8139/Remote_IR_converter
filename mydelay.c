#include <avr/interrupt.h>

#include "mydelay.h"

volatile unsigned char time_26micros = 0;

ISR(TIM0_OVF_vect) // Interrupts every 26.3us(38kHz)
{
    time_26micros++;
}

unsigned char my26Micros(void)
{
    return time_26micros;
}

void delay26nMicros(unsigned char duration)
{
    unsigned char time = my26Micros();
    while((unsigned char)(my26Micros() - time) < duration);
}

void delay4500Micros(void)
{
    delay26nMicros(173);
}
