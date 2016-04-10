
/**
 * commander.cpp
 * Gigatron motor control Arduino code Jetson and RC commanders.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Syler Wagner  <syler@mit.edu>
 * @author  Daniel Gonzalez   <dgonz@mit.edu>
 *
 * @date    2015-09-16    syler   fixed odometry message sending
 **/

#include <Arduino.h>
#include "classes.h"
#include "isr.h"
#include "commander.h"

RCCommander::RCCommander(RCDecoder *sp, RCDecoder *pos, RCDecoder *kill) {
  _sp = sp;
  _pos = pos;
  _kill = kill;
}

int RCCommander::GetLeftSpeedCmd() {
  int left_command = 2 * (int) _sp->GetVal() - 255;

  //$ check edge cases
  if (left_command > 250) left_command = 250;
  if (left_command < -250) left_command = -250;
  
  return left_command;
}

int RCCommander::GetRightSpeedCmd() {
  int right_command = 2 * (int) _sp->GetVal() - 255;

  //$ check edge cases
  if (right_command > 250) right_command = 250;
  if (right_command < -250) right_command = -250;
  
  return right_command;
}

unsigned char RCCommander::GetPositionCmd() {
  return _pos->GetVal();
}

unsigned char RCCommander::GetKillCmd() {
  return _kill->GetVal();
}


//$
JetsonCommander::JetsonCommander(ros::NodeHandle *nh) {

  _nh = nh;

/*
  _jetsonMode = true;
  _semiautomaticMode = false;
*/

  _autonomous = 2;

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


