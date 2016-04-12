/**
 * isr.h
 * Gigatron motor control Arduino code for interrupts.
 * 
 * @author  Bayley Wang       <bayleyw@mit.edu>
 * @author  Syler Wagner      <syler@mit.edu>
 * @author  Chris Desnoyers   <cjdesno@mit.edu>
 *
 * @date    2016-01-10    syler   renamed from shared.h to isr.h
 * @date    2016-03-27    syler   define encoder interrupts and pins, fixed ISRs for left and right encoders
 *
 **/

#ifndef __ISR_H
#define __ISR_H

//$ left encoder
#define R_ENCODER_INTERRUPT 4
#define R_ENCODER_PIN_A 19
#define R_ENCODER_PIN_B 17

//$ right encoder
#define L_ENCODER_INTERRUPT 5
#define L_ENCODER_PIN_A 18
#define L_ENCODER_PIN_B 16
//#define L_ENCODER_REVERSED //$ define that the left encoder is reversed to handle directionality

extern volatile unsigned long _pw0_us, _pw1_us, _pw2_us;
extern volatile unsigned long _pw0_last_t, _pw1_last_t, _pw2_last_t;
extern volatile long _ticks_left, _ticks_right;		//$ number of ticks for each encoder

void ISR0();
void ISR1();
void ISR2();
void LeftISR();   //$ left encoder interrupt service routine
void RightISR();  //$ right encoder interrupt service routine

#endif


