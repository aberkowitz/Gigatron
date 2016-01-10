/**
 * isr.h
 * Gigatron motor control Arduino code for interrupts.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 *
 * @date    2016-01-10    syler   renamed from shared.h to isr.h
 *
 **/

#ifndef __ISR_H
#define __ISR_H

extern volatile unsigned long _pw0_us, _pw1_us, _pw2_us;
extern volatile unsigned long _pw0_last_t, _pw1_last_t, _pw2_last_t;
extern volatile int _speed_2, _speed_3;

void ISR0();
void ISR1();
void ISR2();
void ISR4();
void ISR5();

#endif

