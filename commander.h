#ifndef __COMMANDER_H
#define __COMMANDER_H

#include <Arduino.h>
#include "classes.h"
#include "shared.h"

class Commander {
public:
  virtual unsigned char GetLeftSpeedCmd(){return 0;}
  virtual unsigned char GetRightSpeedCmd(){return 0;}
  virtual unsigned char GetPositionCmd(){return 0;}
};

class RCCommander: public Commander {
public:
  RCCommander(RCDecoder *sp, RCDecoder *pos);
  virtual unsigned char GetLeftSpeedCmd();
  virtual unsigned char GetRightSpeedCmd();
  virtual unsigned char GetPositionCmd();
private:
  RCDecoder *_sp, *_pos;
};
  
#endif

