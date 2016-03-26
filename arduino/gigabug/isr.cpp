/**
 * isr.cpp
 * Gigatron motor control Arduino code for interrupts.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 *
 **/

#include <Arduino.h>
#include "isr.h"

volatile unsigned long _pw0_us, _pw1_us, _pw2_us;
volatile unsigned long _pw0_last_t, _pw1_last_t, _pw2_last_t;
volatile int _speed_2, _speed_3;
//if speeds need to be unsigned, we can figure out a different way to convey direction

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
  //Serial.println(_pw1_us);
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
void ISR4() {
  //Serial.println("AYY");
  if (digitalRead(16)) {
    //Serial.println("ISR4 digitalRead(16)");
    _speed_2++;
  }
  else{
    //Serial.println("ISR4");
    _speed_2--;
  }
}

void ISR5() {
  //Serial.println("AYY");
  if (digitalRead(17)) {
    //Serial.println("ISR5 digitalRead(17)");
    _speed_3++;
  }
  else{
    //Serial.println("ISR5");
    _speed_3--;
  }
}




