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

#define NUM_CODES 3

#define F_CPU 9600000UL

#define sbi(PORT, BIT) PORT |= _BV(BIT)
#define cbi(PORT, BIT) PORT &= ~_BV(BIT)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h> 
// #include "BasicSerial3.h"
// #include "xitoa.h"

//                                                       PLAY                      |<<                       >>|
PROGMEM const unsigned char ir_recieve[NUM_CODES][4] = {{0x4B, 0x98, 0x3A, 0xC5}, {0x4B, 0x40, 0x09, 0xF6}, {0x4B, 0x40, 0xF1, 0x0E}};
PROGMEM const unsigned char ir_send[NUM_CODES][4]    = {{0x9E, 0x61, 0x40, 0xBF}, {0x9E, 0x61, 0x20, 0xDF}, {0x9E, 0x61, 0xE0, 0x1F}};

volatile unsigned char time_26micros = 0;
volatile unsigned char code_vol[4] = {};

void initTimer(void);
void initIRIn(void);
unsigned char my26Micros(void);
void delay26nMicros(unsigned char);
void delay4500us(void);
unsigned char readIR(void);
void enablePWM(void);
void disablePWM(void);
void sendLeader(void);
void sendIR1Bit(unsigned char);
void sendIR(unsigned char *);

ISR(TIM0_OVF_vect) // Interrupts every 26.3us(38kHz)
{
    time_26micros++;
}

ISR(PCINT0_vect)
{
    cbi(PCMSK, PCINT0);
    sei(); // Multiple Interrupt Enable except Pin Change Interrupt
    sbi(PORTB, LED);
    // readIR();
    if(!readIR()) {
        // _delay_ms(250);
        for (char i = 0; i < 55; i++) {
            delay4500us();
        }
        for(char i = 0; i < NUM_CODES; i++) {
            for(char j = 0; j < 4; j++) {
                if(code_vol[j] != pgm_read_byte(&(ir_recieve[i][j]))) {
                    break;
                }
                if(j == 3) {
                    sendIR(ir_send[i]);
                }
            }
        }
    }
    initIRIn();
    cbi(PORTB, LED);
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

unsigned char readIR(void)
{
    //loop_until_bit_is_set(PINB, IRIN); // Wait until the code starts -- the output of IR receiver rodules is active low
    //loop_until_bit_is_clear(PINB, IRIN);
    loop_until_bit_is_set(PINB, IRIN);
    unsigned char time = my26Micros();
    loop_until_bit_is_clear(PINB, IRIN);
    time = my26Micros() - time;
    if(time < 156 || 190 < time){ // Elapsed time should be around 4500us
        return 1;
    }
    for(char i = 0; i < 4; i++) {
        code_vol[i] = 0;
        for(char j = 7; j >= 0; j--) {
            loop_until_bit_is_set(PINB, IRIN);
            time = my26Micros();
            loop_until_bit_is_clear(PINB, IRIN);
            time = my26Micros() - time;
            // if(time > 40) {
            if(56 < time && time < 72) { // "1" is around 3T(1686us)
                code_vol[i] |= _BV(j);
            } else if(time < 12 || 30 < time) { // "0" should be around 1T(562us)
                return 2;
            }
        }
    }
    return 0;
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

void sendIR(unsigned char *data)
{
    unsigned char code_byte = 0;
    sendLeader();
    for(char i = 0; i < 4; i++) {
        code_byte = pgm_read_byte(&(data[i]));
        for(char j = 7; j >= 0; j--) {
            sendIR1Bit((code_byte >> j) & 0x01);
        }
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
