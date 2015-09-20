/**
 * isr.cpp
 * Gigatron motor control Arduino code.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 **/

#include <Arduino.h>
#include "shared.h"

volatile unsigned long _pw0_us, _pw1_us;
volatile unsigned long _pw0_last_t, _pw1_last_t;
volatile unsigned int _speed_2, _speed_3;

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

void ISR4() {
  //Serial.println("AYY");
  _speed_2++;
}

void ISR5() {
  //Serial.println("AYY");
  _speed_3++;
}



