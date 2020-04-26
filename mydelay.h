#ifndef MYDELAY_H
#define MYDELAY_H

#ifndef __AVR_ATtiny13A__
#define __AVR_ATtiny13A__
#endif // not needed when compile but needed when auto-complete of VSCode

unsigned char my26Micros(void);
void delay26nMicros(unsigned char);
void delay4500Micros(void);

#endif
