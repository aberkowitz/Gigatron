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

  _jetsonMode = true;

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

