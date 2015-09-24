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
  double rpmSmooth;
  double filterVal = 0.25;
private:
  int _interrupt;
  int _poles, _interval;
};
  

#endif

