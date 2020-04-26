#ifndef IR_H
#define IR_H

#ifndef __AVR_ATtiny13A__
#define __AVR_ATtiny13A__
#endif // not needed when compile but needed when auto-complete of VSCode

#define NUM_CODES 12

#define IRIN 0
#define IROUT 1
#define LED 3

#define sbi(PORT, BIT) PORT |= _BV(BIT)
#define cbi(PORT, BIT) PORT &= ~_BV(BIT)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

void initTimer(void);
void initIRIn(void);
unsigned char readIR(void);
void enablePWM(void);
void disablePWM(void);
void sendLeader(void);
void sendIR1Bit(unsigned char);
void sendIR(const unsigned char *);

#endif
