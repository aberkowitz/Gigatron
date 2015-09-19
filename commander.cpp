/**
 * commander.cpp
 * Gigatron motor control Arduino code.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Syler Wagner  <syler@mit.edu>
 *
 * @date    2015-09-16    syler   fixed odometry message sending
 **/

#include <Arduino.h>
#include "classes.h"
#include "shared.h"
#include "commander.h"

RCCommander::RCCommander(RCDecoder *sp, RCDecoder *pos) {
  _sp = sp;
  _pos = pos;
}

int RCCommander::GetLeftDirectionCmd() {
  unsigned int spd = _sp->GetVal();
  if (spd > 124) {
    return 1;
  }
  else if (spd > 116) {
    return 0;
  }
  else {
    return -1;
  }
}

//460 reverse, 510 forward
//~120 should be reverse cutoff (120.9)
//return +1 for forward, -1 for reverse, 0 for standstill
int RCCommander::GetRightDirectionCmd() {
  unsigned int spd = _sp->GetVal();
  if (spd > 124) {
    return 1;
  }
  else if (spd > 116) {
    return 0;
  }
  else {
    return -1;
  }
}

unsigned char RCCommander::GetLeftSpeedCmd() {
  return _sp->GetVal();
}

unsigned char RCCommander::GetRightSpeedCmd() {
  return _sp->GetVal();
}

unsigned char RCCommander::GetPositionCmd() {
  return _pos->GetVal();
}

//$
JetsonCommander::JetsonCommander(ros::NodeHandle *nh) {

  _nh = nh;

  _jetsonMode = false;

  _leftRPMCmd = 0; 
  _rightRPMCmd = 0; 
  _posCmd = 128;
}

unsigned int JetsonCommander::GetLeftRPMCmd() {
  return _leftRPMCmd;
}

unsigned int JetsonCommander::GetRightRPMCmd() {
  return _rightRPMCmd;
}

unsigned char JetsonCommander::GetPositionCmd() {
  return _posCmd;
}

