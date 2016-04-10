/**
 * rcdecoder.cpp
 * Gigatron motor control Arduino code for RC decoder.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Chris Desnoyers   <cjdesno@mit.edu>
 *
 * @date    2016-01-10    syler   moved to separate .cpp file, header is in classes.h
 *
 **/

#include <Arduino.h>
#include "classes.h"
#include "isr.h"

RCDecoder::RCDecoder(int interrupt, int minV, int maxV) {
  _interrupt = interrupt;
  _minV = minV;
  _maxV = maxV;
  
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(21, INPUT);
  
  if (_interrupt == 0) {
    attachInterrupt(0, ISR0, CHANGE);
  } else if (_interrupt == 1) {
    attachInterrupt(1, ISR1, CHANGE);
  } else if (_interrupt == 2) {
    attachInterrupt(2, ISR2, CHANGE);
  }
}

unsigned char RCDecoder::GetVal() {
  long pw;
  if (_interrupt == 0) {
    pw = _pw0_us;
  } else if (_interrupt == 1) {
    pw = _pw1_us;
  }if (_interrupt == 2) {
    pw = _pw2_us;
  }
  
  //Serial.println(pw); //RC decoder vals
  //dp(pw);
  pw = (pw - _minV) << 8;
  pw /= (_maxV - _minV);
  if (pw < 0) pw = 0;
  if (pw > 255) pw = 255;
  return (unsigned char) pw;
}

