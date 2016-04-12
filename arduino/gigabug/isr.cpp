  /**
 * isr.cpp
 * Gigatron motor control Arduino code for interrupt service routines.
 * 
 * @author  Bayley Wang       <bayleyw@mit.edu>
 * @author  Syler Wagner      <syler@mit.edu>
 * @author  Chris Desnoyers   <cjdesno@mit.edu>
 *
 * @date    2016-03-27    syler   define encoder interrupts and pins, fixed ISRs for left and right encoders
 *
 **/

#include <Arduino.h>
#include "isr.h"

volatile unsigned long _pw0_us, _pw1_us, _pw2_us;
volatile unsigned long _pw0_last_t, _pw1_last_t, _pw2_last_t;

volatile long _ticks_left, _ticks_right;  //$ number of ticks for each encoder
volatile bool _read_left, _read_right;    //$ state of digitalRead(encoder pin B) for each encoder

void ISR0() {
  int state = digitalRead(2);
  if (state) {
    _pw0_last_t = micros();
  } else {
    _pw0_us = micros() - _pw0_last_t;
  }
}

void ISR1() {
  int state = digitalRead(3);
  if (state) {
    _pw1_last_t = micros();
  } else {
    _pw1_us = micros() - _pw1_last_t;
  }
}

void ISR2() {
  int state = digitalRead(21);
  if (state) {
    _pw2_last_t = micros();
  } else {
    _pw2_us = micros() - _pw2_last_t;
  }
}

//Adapted for quadrature encoder, direction inferred from pulse alignment (11 forward, 10 backward)
//$ left encoder interrupt service routine
void LeftISR() { 
  //$ read encoder input pin B
  _read_left = bitRead(PORTH, 1); //$ equivalent to digitalRead(L_ENCODER_PIN_B) but faster 

  #ifdef L_ENCODER_REVERSED //$ if left encoder is reversed
    //$ increment counter if B leads A
    _ticks_left += _read_left ? +1 : -1; //$ if (_read_left) {_ticks_left++;} else {_ticks_left--;}
  #else
    //$ increment counter if A leads B
    _ticks_left += _read_left ? -1 : +1; //$ if (_read_left) {_ticks_left--;} else {_ticks_left++;}
  #endif

}

//$ right encoder interrupt service routine
void RightISR() { 
  //$ read encoder input pin B
  _read_right = bitRead(PORTH, 0); //$ equivalent to digitalRead(R_ENCODER_PIN_B) but faster

  #ifdef R_ENCODER_REVERSED //$ if right encoder is reversed
    //$ increment counter if B leads A
    _ticks_right += _read_right ? +1 : -1; //$ if (_read_right) {_ticks_right++;} else {_ticks_right--;}
  #else
    //$ increment counter if A leads B
    _ticks_right += _read_right ? -1 : +1; //$ if (_read_right) {_ticks_right--;} else {_ticks_right++;}
  #endif

}





