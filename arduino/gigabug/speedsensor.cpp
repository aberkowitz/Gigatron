/**
 * speedsensor.cpp
 * Gigatron motor control Arduino code for Hall Effect sensors.
 * 
 * @author  Bayley Wang       <bayleyw@mit.edu>
 * @author  Chris Desnoyers   <cjdesno@mit.edu>
 * @author  Daniel Gonzalez   <dgonz@mit.edu>
 * @author  Syler Wagner      <syler@mit.edu>
 *
 * @date    2016-01-10    syler   moved to separate .cpp file, header is in classes.h
 * @date    2016-03-27    syler   fixed RPM calculation for new quadrature encoders and cleaned up encoder interrupt pin setup
 *
 **/

#include <Arduino.h>
#include "classes.h"
#include "isr.h"

#define PULSES_PER_REV 600 //$ number of encoder pulses per full motor revolution

SpeedSensor::SpeedSensor(int interrupt, int poles, int interval) {
  _interrupt = interrupt;
  _poles = poles;
  _interval = interval;
  //interval is set in gigatron.ino; it is the interval at which the context loop runs, 
  //and not related to any property of the encoders

  //see https://www.arduino.cc/en/Reference/AttachInterrupt for interrupt documentation
  //in summary, attachInterrupt(4) likely attaches an interrupt to pin 18, and does not attach one to pin 4

  /*$ actually, the link above says the Mega2560 has the following mapping:
      Interrupt     0   1   2   3   4   5
      Mega2560 pin  2   3   21  20  19  18
  */

  pinMode(L_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(L_ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(R_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(R_ENCODER_PIN_B, INPUT_PULLUP);

  if (_interrupt == R_ENCODER_INTERRUPT) {
    attachInterrupt(R_ENCODER_INTERRUPT, RightISR, FALLING);
  } else {
    attachInterrupt(L_ENCODER_INTERRUPT, LeftISR, FALLING);
  }
  
  _ticks_left = _ticks_right = 0;
}

//$ returns number of ticks per S_LOOP_INTERVAL
long SpeedSensor::GetTicks() {
  long ticks;

  if (_interrupt == L_ENCODER_INTERRUPT) { // If we are the left sensor
    ticks = _ticks_left;
    _ticks_left = 0;
  } else if (_interrupt == R_ENCODER_INTERRUPT) { // right sensor
    ticks = _ticks_right;
   _ticks_right = 0;
  }

  return ticks;
}

/*
//$ TODO: this is unsigned, need to fix!
unsigned int SpeedSensor::GetSpeed() {
  long ticks;

  if (_interrupt == L_ENCODER_INTERRUPT) {// If we are the left sensor
    ticks = _ticks_left;
    // dp(sp); //$ do not uncomment this print statement if you want your Hall sensors to work
    _ticks_left = 0;
  } else if (_interrupt == R_ENCODER_INTERRUPT) { // right sensor
    ticks = _ticks_right;
    // dp(sp); //$ see above - do not uncomment this print statement unless you add delay somehow  
    _ticks_right = 0;
  }
  
  long motor_revs = ticks / PULSES_PER_REV;
  double wheel_revs = motor_revs * gearRatio;

  double rpm = wheel_revs * (60.0 * 1000) / _interval;

  return rpm; 

  /*$ old code that does not apply to quadrature encoders
      plus DGonz's filter
      //Serial.println(sp);
      //Ticks *60000 millis/minute*(1/interval)*revolutions/ 7 ticks = RPM
      //long hz = sp * 1000 / _interval;
      //long rpm = 120 * hz / _poles;

      //TODO how does 600 pulses per motor revolution affect this code?

      double rpm = sp*60.0*1000.0/_interval/7.0;
      rpmSmooth = (rpm * (1 - filterVal)) + (rpmSmooth  *  filterVal);
      //Serial.println(rpmSmooth);
      if (rpmSmooth < 0) rpmSmooth = 0;
    //  dp(rpmSmooth);
      return (unsigned int) rpmSmooth;

  
}
*/

