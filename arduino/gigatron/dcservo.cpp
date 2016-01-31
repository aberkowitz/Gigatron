/**
 * dcservo.cpp
 * Gigatron motor control Arduino code for steering servo.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Syler Wagner      <syler@mit.edu>
 *
 * @date    2016-01-10    syler   moved to separate .cpp file, header is in classes.h
 *
 **/

#include <Arduino.h>
#include "classes.h"
#include "isr.h"

DCServo::DCServo(int pwmPin1, int pwmPin2, int posPin) {
  _pwmPin1 = pwmPin1;
  _pwmPin2 = pwmPin2;
  _posPin = posPin;
  _minV = 0;
  _midV = 511;
  _maxV = 1023;
  
  pinMode(_pwmPin1, OUTPUT);
  pinMode(_pwmPin2, OUTPUT);
  pinMode(_posPin, INPUT);
}

void DCServo::ConfigPot(int minV, int midV, int maxV) {
  _minV = minV;
  _midV = midV;
  _maxV = maxV;
}

void DCServo::SetVelocity(int vel) {
  //if (vel>200) vel = 200; //200 is my cap
  
  if (vel > 0) {
    analogWrite(_pwmPin1, vel);
    digitalWrite(_pwmPin2, LOW);
  } else {
    analogWrite(_pwmPin2, -vel);
    digitalWrite(_pwmPin1, LOW);
  }
}

unsigned char DCServo::GetPos() {
  long adu = analogRead(_posPin);
  //dp(adu); //$ uncomment for pot calibration
  long tmp = (adu - _minV) << 8 ;;
  tmp /= (_maxV - _minV);
  if (tmp < 0) tmp = 0;
  if (tmp > 255) tmp = 255;
  //dp(tmp);
  return (unsigned char) tmp;
}

//$ takes pot limits and middle value and linearizes the output
/* DGonz's measurements as of 9:49pm 9/24/2015
 * 439 in, 0 out
 * 549 in, 127 out
 * 622 in, 255 out
 */
unsigned char DCServo::GetPosLinearized() {
  long adu = analogRead(_posPin);
  //dp(adu); //$ uncomment for pot calibration
  long tmp;
  if (adu < _midV) {
    tmp = (adu - _minV) << 7 ;;
    tmp /= (_midV - _minV);
  }
  else {
    tmp = (adu - _midV) << 7 ;
    tmp /= (_maxV - _midV);
    tmp += 127;
  }
  if (tmp < 0) tmp = 0;
  if (tmp > 255) tmp = 255;
  //dp(tmp);
  unsigned char res = (unsigned char) tmp;
//  dp (res);
  return res;
}