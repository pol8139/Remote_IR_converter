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
#include <util/delay.h>
// #include "BasicSerial3.h"
// #include "xitoa.h"

volatile unsigned int time_26micros = 0;

void initTimer(void);
void initIRIn(void);
unsigned char my26Micros(void);
void delay26nMicros(unsigned char);
void delay4500us(void);
unsigned long readIR(void);
void enablePWM(void);
void disablePWM(void);
void sendLeader(void);
void sendIR1Bit(unsigned char);
void sendIR(unsigned long);

ISR(TIM0_OVF_vect) // Interrupts every 26.3us(38kHz)
{
    time_26micros++;
}

ISR(PCINT0_vect)
{
    cbi(PCMSK, PCINT0);
    sei(); // Multiple Interrupt Enable except Pin Change Interrupt
    readIR();
    initIRIn();
}

void initTimer(void)
{
    TCCR0A = _BV(COM0B1) | _BV(COM0B0) | _BV(WGM01) | _BV(WGM00); // Compare Match B Output, Fast PWM
	TCCR0B = _BV(WGM02) | _BV(CS01); // Prescale 1/8
	OCR0A = 29; // 38kHz
	OCR0B = 20; // Duty 1/3
	TIMSK0 = _BV(TOIE0); // Counter Overflow Interrupt Enable
}

void initIRIn(void)
{
    GIMSK = _BV(PCIE); // Pin Change Interrupt Enable
    PCMSK = _BV(PCINT0); // Pin 0 Change Enable
}

// void send(char c)
// {
//     TxByte(c);
// }

unsigned char my26Micros(void)
{
    return time_26micros;
}

void delay26nMicros(unsigned char duration)
{
    unsigned char time = my26Micros();
    while((unsigned char)(my26Micros() - time) < duration);
}

void delay4500us(void)
{
    delay26nMicros(173);
}

unsigned long readIR(void)
{
    //loop_until_bit_is_set(PINB, IRIN); // Wait until the code starts -- the output of IR receiver rodules is active low
    //loop_until_bit_is_clear(PINB, IRIN);
    unsigned char time;
    unsigned long code = 0;
    loop_until_bit_is_set(PINB, IRIN);
    time = my26Micros();
    loop_until_bit_is_clear(PINB, IRIN);
    time = my26Micros() - time;
    if(time < 156 || 190 < time){ // Elapsed time should be 4500us
        return READ_FAILED;
    }
    for(char i = 31; i >= 0; i--) {
        loop_until_bit_is_set(PINB, IRIN);
        time = my26Micros();
        loop_until_bit_is_clear(PINB, IRIN);
        time = my26Micros() - time;
        if(time > 40) {
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
    sendIR(0x4BB6C03F);
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
    unsigned char off_time;
    if(data) {
        off_time = 63; // 1686us
    } else {
        off_time = 21; // 562us
    }
    enablePWM();
    delay26nMicros(21);
    disablePWM();
    delay26nMicros(off_time);
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
    DDRB = _BV(LED) | _BV(IROUT);
    initTimer();
    initIRIn();
    // xdev_out(send);
    disablePWM();
    sei();
    while(1) {
        // readIR();
        ;
    }
}
