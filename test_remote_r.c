//                  +-\/-+
//    (reset) PB5  1|    |8  Vcc
//  LED debug PB3  2|    |7  PB2
// UART 38400 PB4  3|    |6  PB1 (OCB1)  IR out
//            GND  4|    |5  PB0 (PCINT0)IR in
//                  +----+

#ifndef __AVR_ATtiny13A__
#define __AVR_ATtiny13A__
#endif // not needed when compile but needed when auto-complete of VSCode

#define IRIN 0
#define IROUT 1
#define LED 3

#define READ_FAILED 0xFFFF

#define sbi(PORT, BIT) PORT |= _BV(BIT)
#define cbi(PORT, BIT) PORT &= ~_BV(BIT)

#include <avr/io.h>
#include <avr/interrupt.h>
// #include <stdio.h>
#include <util/delay.h>
// #include "BasicSerial3.h"
// #include "xitoa.h"

volatile unsigned int time_26micros = 0;

void initTimer(void);
void initIRIn(void);
unsigned char my26Micros(void);
void delay4500us(void);
unsigned long readIR(void);
void enablePWM(void);
void disablePWM(void);
void sendLeader(void);
void sendIR1Bit(unsigned char);
void sendIR(unsigned long);

ISR(TIM0_OVF_vect) // interrupts every 26.3us(38kHz)
{
    time_26micros++;
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
	OCR0A = 29; // 38kHz
	OCR0B = 20; // Duty 1/3
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

unsigned char my26Micros(void)
{
    return time_26micros;
}

void delay4500us(void)
{
    unsigned char time = my26Micros();
    while((unsigned char)(my26Micros() - time) < 173);
}

unsigned long readIR(void)
{
    //loop_until_bit_is_set(PINB, IRIN); // wait until the code starts -- the output of IR receiver rodules is active low
    //loop_until_bit_is_clear(PINB, IRIN);
    unsigned char time/* = myMicros(), elapsed*/;
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
    for(char i = 31; i >= 0; i--) {
        loop_until_bit_is_set(PINB, IRIN);
        time = my26Micros();
        loop_until_bit_is_clear(PINB, IRIN);
        time = my26Micros() - time;
        if(time > 40) {
            // code |= 1 << ((3 - (i / 8)) * 8 + i % 8);
            code |= 1 << i;
        }
    }
    sbi(PORTB, LED);
    _delay_ms(250);
    cbi(PORTB, LED);
    _delay_ms(250);
    // TxByte(code >> 24);
    // TxByte(code >> 16);
    // TxByte(code >> 8);
    // TxByte(code);
    sendIR(0x4BB6C13E);
    return code;
}

void enablePWM(void)
{
    TCCR0A |= _BV(COM0B1) | _BV(COM0B0);
}

void disablePWM(void)
{
    TCCR0A &= ~(_BV(COM0B1) | _BV(COM0B0));
    cbi(PORTB, IROUT);
}

void sendLeader(void)
{
    enablePWM();
    delay4500us();
    delay4500us();
    disablePWM();
    delay4500us();
}

void sendIR1Bit(unsigned char data)
{
    unsigned char time, off_time;
    if(data) {
        off_time = 63;
    } else {
        off_time = 21;
    }
    enablePWM();
    time = my26Micros();
    while((unsigned char)(my26Micros() - time) < 21);
    disablePWM();
    time = my26Micros();
    while((unsigned char)(my26Micros() - time) < off_time);
}

void sendIR(unsigned long data)
{
    sendLeader();
    for(char i = 31; i >= 0; i--) {
        sendIR1Bit((data >> i) & 0x01);
    }
    sendIR1Bit(0); // stop bit
}

int main(void)
{
    // unsigned char t = 0;
    // unsigned int time, diff;
    // unsigned char t = 0;

    DDRB = _BV(LED) | _BV(IROUT);

    initTimer();
    initIRIn();
    // xdev_out(send);
    disablePWM();
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
