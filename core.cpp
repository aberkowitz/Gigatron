#include <Arduino.h>
#include "classes.h"
#include "shared.h"

DCServo::DCServo(int pwmPin, int dirPin, int posPin) {
  _pwmPin = pwmPin;
  _dirPin = dirPin;
  _posPin = posPin;
  _minV = 0;
  _maxV = 1023;
  
  pinMode(_pwmPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  pinMode(_posPin, INPUT);
}

void DCServo::ConfigSensor(int minV, int maxV) {
  _minV = minV;
  _maxV = maxV;
}

void DCServo::SetVelocity(int vel) {
  if (vel > 0) {
    analogWrite(_pwmPin, vel);
    digitalWrite(_dirPin, LOW);
  } else {
    analogWrite(_pwmPin, -vel);
    digitalWrite(_dirPin, HIGH);
  }
}

unsigned char DCServo::GetPos() {
  long adu = analogRead(_posPin);
  //Serial.println(adu); //$ uncomment for pot calibration
  long tmp = (adu - _minV) << 8 ;;
  tmp /= (_maxV - _minV);
  if (tmp < 0) tmp = 0;
  if (tmp > 255) tmp = 255;
  return (unsigned char) tmp;
}

RCDecoder::RCDecoder(int interrupt, int minV, int maxV) {
  _interrupt = interrupt;
  _minV = minV;
  _maxV = maxV;
  
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  
  if (_interrupt == 0) {
    attachInterrupt(0, ISR0, CHANGE);
  } else {
    attachInterrupt(1, ISR1, CHANGE);
  }
}

unsigned char RCDecoder::GetVal() {
  long pw;
  if (_interrupt == 0) {
    pw = _pw0_us;
  } else {
    pw = _pw1_us;
  }
  //Serial.println(pw); //RC decoder vals
  //dp(pw);
  pw = (pw - _minV) << 8;
  pw /= (_maxV - _minV);
  if (pw < 0) pw = 0;
  if (pw > 255) pw = 255;
  return (unsigned char) pw;
}

SpeedSensor::SpeedSensor(int interrupt, int poles, int interval) {
  _interrupt = interrupt;
  _poles = poles;
  _interval = interval;
  
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  
  if (_interrupt == 4) {
    attachInterrupt(4, ISR4, RISING);
  } else {
    attachInterrupt(5, ISR5, RISING);
  }
  
  _speed_2 = _speed_3 = 0;
}

unsigned int SpeedSensor::GetSpeed() {
  long sp;
  if (_interrupt == 4) {// If we are the left sensor
    sp = _speed_2;
    _speed_2 = 0;
  } else { // right sensor
    sp = _speed_3;
    _speed_3 = 0;
  }
  
  //Serial.println(sp);
  //Ticks *60000 millis/minute*(1/interval)*revolutions/ 7 ticks = RPM
  //long hz = sp * 1000 / _interval;
  //long rpm = 120 * hz / _poles;

  double rpm = sp*60.0*1000.0/_interval/7.0;
  rpmSmooth = (rpm * (1 - filterVal)) + (rpmSmooth  *  filterVal);
  if (rpm < 0) rpm = 0;
  return (unsigned int) rpmSmooth;
}

