//                  +-\/-+
//    (reset) PB5  1|    |8  Vcc
//  LED debug PB3  2|    |7  PB2
// UART 38400 PB4  3|    |6  PB1 (OCB1)  IR out
//            GND  4|    |5  PB0 (PCINT0)IR in
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
#include "BasicSerial3.h"
// #include "xitoa.h"

volatile unsigned int time_micros = 0;

void initTimer(void);
void initIRIn(void);
unsigned int myMicros(void);
unsigned long readIR(void);

ISR(TIM0_OVF_vect) // interrupts every 26.3us(38kHz)
{
    time_micros += 26;
}

ISR(PCINT0_vect)
{
    cbi(PCMSK, PCINT0);
    sei(); // 多重割り込みをピン変化以外許可
    readIR();
    initIRIn();
}

void initTimer(void)
{
    TCCR0A = _BV(COM0B1) | _BV(COM0B0) | _BV(WGM01) | _BV(WGM00); // Set OC0B at Compare Match (=OCR0B) and clear at TOP
	TCCR0B = _BV(WGM02) | _BV(CS01); // Prescale 1/8, Fast PWM with TOP = OCR0A
	//TCNT0 = 0;
	OCR0A = 32; // PWM interval 560uS
	OCR0B = 21; // Duty 1/3 (clear=21 clocks, set=32-21=11 clocks)
	TIMSK0 = /*_BV(OCIE0B) | */_BV(TOIE0); // Counter Overflow Interrupt Enable
}

void initIRIn(void)
{
    GIMSK = _BV(PCIE); // Pin Change Interrupt Enable
    PCMSK = _BV(PCINT0); //Pin 0 Change Enable
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
    //loop_until_bit_is_set(PINB, IRIN); // wait until the code starts -- the output of IR receiver rodules is active low
    //loop_until_bit_is_clear(PINB, IRIN);
    unsigned int time/* = myMicros(), elapsed*/;
    unsigned long code = 0;
    loop_until_bit_is_set(PINB, IRIN);
    // elapsed = myMicros() - time;
    // if(elapsed < 7200 || 10800 < elapsed){
    //     return READ_FAILED;
    // }
    // time = myMicros();
    loop_until_bit_is_clear(PINB, IRIN);
    // elapsed = myMicros() - time;
    // if(elapsed < 3600 || 5400 < elapsed){
    //     return READ_FAILED;
    // }
    for(char i = 0; i < 32; i++) {
        loop_until_bit_is_set(PINB, IRIN);
        time = myMicros();
        loop_until_bit_is_clear(PINB, IRIN);
        //elapsed = myMicros() - time;
        if(myMicros() - time > 1000) {
            // code |= 1 << ((3 - (i / 8)) * 8 + i % 8);
            code |= 1 << (31 - i);
        }
    }
    sbi(PORTB, LED);
    _delay_ms(500);
    cbi(PORTB, LED);
    for(char i = 0; i < 4; i++) {
        TxByte((code >> (8 * (3 - i))) & 0xFF);
    }
    return 0;
}

int main(void)
{
    // unsigned char t = 0;
    // unsigned int time, diff;
    // unsigned char t = 0;

    DDRB |= _BV(DDB3);

    initTimer();
    initIRIn();
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

        // readIR();
        ;
        // xitoa(diff, 10, 0);
        // TxByte('\t');
        // xitoa(time, 10, 0);
        // TxByte('\r');
        // TxByte('\n');
        // serOut("hogehoge\n\r");
        // _delay_ms(500);
    }
}
