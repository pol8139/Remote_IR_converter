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

#define NUM_CODES 12

#define sbi(PORT, BIT) PORT |= _BV(BIT)
#define cbi(PORT, BIT) PORT &= ~_BV(BIT)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

PROGMEM const unsigned char ir_recieve[NUM_CODES][4] = 
{
    {0x4B, 0xB6, 0x21, 0xDE}, // RC-934R LEFT
    {0x4B, 0xB6, 0xA1, 0x5E}, // RC-934R RIGHT
    {0x4B, 0xB6, 0xE9, 0x16}, // RC-934R ENTER
    {0x4B, 0x98, 0x3A, 0xC5}, // RC-934R PLAY/PAUSE
    {0x4B, 0x40, 0x09, 0xF6}, // RC-934R |<<
    {0x4B, 0x40, 0xF1, 0x0E}, // RC-934R >>|
    {0x4B, 0x98, 0xFA, 0x05}, // RC-934R REPEAT
    {0x4B, 0x40, 0xCB, 0x34}, // RC-934R MODE
    {0x02, 0xFD, 0x48, 0xB7}, // SE-R0473 TV電源
    {0x02, 0xFD, 0xF0, 0x0F}, // SE-R0473 TV/入力切替
    {0x02, 0xFD, 0x58, 0xA7}, // SE-R0473 TV音量↑
    {0x02, 0xFD, 0x78, 0x87}, // SE-R0473 TV音量↓
};

PROGMEM const unsigned char ir_send[NUM_CODES][4] = 
{
    {0xA2, 0xED, 0xF8, 0x07}, // SE-R0473 チャンネル↓
    {0xA2, 0xED, 0x78, 0x87}, // SE-R0473 チャンネル↑
    {0x9E, 0x61, 0x80, 0x7F}, // RS-CD6T OPEN/CLOSE
    {0x9E, 0x61, 0xAA, 0x55}, // RS-CD6T PAUSE
    {0x9E, 0x61, 0x20, 0xDF}, // RS-CD6T |<<
    {0x9E, 0x61, 0xE0, 0x1F}, // RS-CD6T >>|
    {0x9E, 0x61, 0x10, 0xEF}, // RS-CD6T S/F/OFF
    {0x9E, 0x61, 0x50, 0xAF}, // RS-CD6T TIME DISPLAY
    {0x4B, 0x36, 0xD3, 0x2C}, // RC-934R POWER
    {0x4B, 0xB6, 0x70, 0x8F}, // RC-934R CBL/SAT
    {0x4B, 0xB6, 0x40, 0xBF}, // RC-934R VOL +
    {0x4B, 0xB6, 0xC0, 0x3F}, // RC-934R VOL -
};

volatile unsigned char time_26micros = 0;
volatile unsigned char code_vol[4] = {};

void initTimer(void);
void initIRIn(void);
void disableUnusedFunctions(void);
unsigned char my26Micros(void);
void delay26nMicros(unsigned char);
void delay4500us(void);
unsigned char readIR(void);
void enablePWM(void);
void disablePWM(void);
void sendLeader(void);
void sendIR1Bit(unsigned char);
void sendIR(const unsigned char *);

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
    unsigned char result = readIR();
    // _delay_ms(150);
    for(char i = 0; i < 33; i++) {
        delay4500us();
    }
    if(!result) {
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

void disableUnusedFunctions(void)
{
    // ADC is disabled by default
    ACSR |= _BV(ACD); // Analog Comparator Disable
    MCUSR &= ~_BV(WDRF);
    WDTCR |= _BV(WDCE) | _BV(WDE);
    WDTCR = 0x00; // WDT Disable
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

void delay4500us(void)
{
    delay26nMicros(173);
}

unsigned char readIR(void)
{
    loop_until_bit_is_set(PINB, IRIN); // Note that the output of IR receiver rodules is active low
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

void sendIR(const unsigned char *data)
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
    DDRB = _BV(LED) | _BV(IROUT) | _BV(DDB4) | _BV(DDB2);
    initTimer();
    initIRIn();
    disableUnusedFunctions();
    // xdev_out(send);
    disablePWM();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sei();
    while(1) {
        sleep_bod_disable();
        sleep_mode();
    }
}
