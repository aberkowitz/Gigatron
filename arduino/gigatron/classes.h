/**
 * pidcontroller.cpp
 * Gigatron motor control Arduino code for class definitions.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 *
 * @date    2016-01-10    syler   refactored header files
 *
 **/

#ifndef __CLASSES_H
#define __CLASSES_H

#define dp(var) Serial.print(#var);Serial.print(": ");Serial.println(var)


class DCServo {
public:
  DCServo(int pwmPin, int dirPin, int posPin);
  void ConfigPot(int minV, int midV, int maxV);
  void SetVelocity(int vel);
  unsigned char GetPos();
  unsigned char GetPosLinearized();
private:
  int _pwmPin, _dirPin, _posPin;
  int _minV, _midV, _maxV;
};

class RCDecoder {
public:
  RCDecoder(int interrupt, int minV, int maxV); // edges of pulse widths in microseconds
  unsigned char GetVal();
private:
  int _interrupt;
  int _minV, _maxV;
};

class SpeedSensor {
public:
  SpeedSensor(int interrupt, int poles, int interval);
  unsigned int GetSpeed();
  double rpmSmooth = 0.0;
  double filterVal = 0.25;
private:
  int _interrupt;
  int _poles, _interval;
};

class PidController {
public:
  PidController(long kp, long ki, long kd, long out_max, long out_min);
  int Update(int ref, int in);
  int ZeroIntegrator();
  void ResetGains(long kp, long ki, long kd);
private:
  long _kp, _ki, _kd;
  long _out_max, _out_min;
  
  int _last_in;
  long _integral;
};
  

#endif

