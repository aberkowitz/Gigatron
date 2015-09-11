#ifndef __CLASSES_H
#define __CLASSES_H

#define dp(var) Serial.print(#var);Serial.print(": ");Serial.println(var)
class DCServo {
public:
  DCServo(int pwmPin, int dirPin, int posPin);
  void ConfigSensor(int minV, int maxV);
  void SetVelocity(int vel);
  unsigned char GetPos();
private:
  int _pwmPin, _dirPin, _posPin;
  int _minV, _maxV;
};

class RCDecoder {
public:
  RCDecoder(int interrupt, int minV, int maxV);
  unsigned char GetVal();
private:
  int _interrupt;
  int _minV, _maxV;
};

class SpeedSensor {
public:
  SpeedSensor(int interrupt, int poles, int interval);
  unsigned int GetSpeed();
private:
  int _interrupt;
  int _poles, _interval;
};
  

#endif

