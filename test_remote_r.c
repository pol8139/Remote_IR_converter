//                  +-\/-+
//    (reset) PB5  1|    |8  Vcc
//  LED debug PB3  2|    |7  PB2
// UART 38400 PB4  3|    |6  PB1 (OCB1) IR out
//            GND  4|    |5  PB0 IR in
//                  +----+

#ifndef __AVR_ATtiny13A__
#define __AVR_ATtiny13A__
#endif // not needed when compile but needed when auto-complete of VSCode

#define LED 3
#define IRIN 0

#define READ_FAILED 0xFFFF

#define sbi(PORT, BIT) PORT |= _BV(BIT)
#define cbi(PORT, BIT) PORT &= ~_BV(BIT)

#include <avr/io.h>
#include <avr/interrupt.h>
// #include <stdio.h>
#include <util/delay.h>
// #include "BasicSerial3.h"
// #include "xitoa.h"

volatile unsigned int time_micros = 0;

ISR(TIM0_OVF_vect) // interrupts every 26.3us(38kHz)
{
    time_micros += 26;
}

void initTimer(void)
{
    TCCR0A = _BV(COM0B1) | _BV(COM0B0) | _BV(WGM01) | _BV(WGM00); // Set OC0B at Compare Match (=OCR0B) and clear at TOP
	TCCR0B = _BV(WGM02) | _BV(CS01); // Prescale 1/8, Fast PWM with TOP = OCR0A
	//TCNT0 = 0;
	OCR0A = 32; // PWM interval 560uS
	OCR0B = 21; // Duty 1/3 (clear=21 clocks, set=32-21=11 clocks)
	//GIMSK  = _BV(INT0) | _BV(PCIE); // External Interrupt Request 0 Enable, Pin Change Interrupt Enable
	TIMSK0 = /*_BV(OCIE0B) | */_BV(TOIE0); // Counter Overflow Interrupt Enable
}

// void send(char c)
// {
//     TxByte(c);
// }

unsigned int myMicros(void)
{
    return time_micros;
}

unsigned long readIR(void)
{
    unsigned int time;
    loop_until_bit_is_set(PINB, IRIN); // wait until the code starts -- the output of IR receiver rodules is active low
    loop_until_bit_is_clear(PINB, IRIN);
    time = myMicros();
    loop_until_bit_is_set(PINB, IRIN);
    if((myMicros() - time) < 7200){
        return READ_FAILED;
    }
    time = myMicros();
    loop_until_bit_is_clear(PINB, IRIN);
    if((myMicros() - time) < 3600){
        return READ_FAILED;
    }
    sbi(PORTB, LED);
    _delay_ms(500);
    cbi(PORTB, LED);
    return 0;
}

int main(void)
{
    // unsigned char t = 0;
    unsigned int time, diff;
    unsigned t = 0;
    DDRB |= _BV(DDB3);
    initTimer();
    // xdev_out(send);
    sei();
    while(1) {
        // time = myMicros();
        // diff = 0;
        // while(diff < 50000) {
        //     diff = myMicros() - time;
        // }
        // t ^= 1;
        // if(t) {
        //     sbi(PORTB, PB3);
        // } else {
        //     cbi(PORTB, PB3);
        // }
        readIR();

        // xitoa(diff, 10, 0);
        // TxByte('\t');
        // xitoa(time, 10, 0);
        // TxByte('\r');
        // TxByte('\n');
        // serOut("hogehoge\n\r");
        // _delay_ms(500);
    }
}
