/**
 * speedsensor.cpp
 * Gigatron motor control Arduino code for Hall Effect sensors.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Chris Desnoyers   <cjdesno@mit.edu>
 * @author  Daniel Gonzalez   <dgonz@mit.edu>
 *
 * @date    2016-01-10    syler   moved to separate .cpp file, header is in classes.h
 *
 **/

#include <Arduino.h>
#include "classes.h"
#include "isr.h"

SpeedSensor::SpeedSensor(int interrupt, int poles, int interval) {
  _interrupt = interrupt;
  _poles = poles;
  _interval = interval;
  //interval is set in gigatron.ino; it is the interval at which the context loop runs, 
  //and not related to any property of the encoders

  //see https://www.arduino.cc/en/Reference/AttachInterrupt for interrupt documentation
  //in summary, attachInterrupt(4) likely attaches an interrupt to pin 18, and does not attach one to pin 4

  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  digitalWrite(16, HIGH);
  digitalWrite(17, HIGH);
  digitalWrite(18, HIGH);
  digitalWrite(19, HIGH);
  
  if (_interrupt == 4) {
    attachInterrupt(4, ISR4, RISING);
  } else {
    attachInterrupt(5, ISR5, RISING);
  }
  
  _speed_2 = _speed_3 = 0;
}

//$ TODO: this is unsigned, need to fix!
unsigned int SpeedSensor::GetSpeed() {
  long sp;
  if (_interrupt == 4) {// If we are the left sensor
    sp = _speed_2;
    //dp(sp); //$ do not uncomment this print statement if you want your Hall sensors to work
    _speed_2 = 0;
  } else if (_interrupt == 5) { // right sensor
    sp = _speed_3;
    //dp(sp); //$ see above - do not uncomment this print statement unless you add delay somehow  
    _speed_3 = 0;
  }
  
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

