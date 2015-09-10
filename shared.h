#ifndef __SHARED_H
#define __SHARED_H

extern volatile unsigned long _pw0_us, _pw1_us;
extern volatile unsigned long _pw0_last_t, _pw1_last_t;
extern volatile unsigned int _speed_2, _speed_3;

void ISR0();
void ISR1();
void ISR2();
void ISR3();

#endif

