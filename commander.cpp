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
//  _sub = _nh.subscribe("cmd_vel", 1000, &JetsonCommander::CmdCallback, this);

  _leftRPMCmd = 0; //$ TODO: fix
  _rightRPMCmd = 0; //$ TODO: fix
  _posCmd = 128;
}

/*

void JetsonCommander::CmdCallback(const std_msgs::Int16MultiArray::ConstPtr& cmd) {
	Serial.println("Steering angle: " << cmd.data[0]);
    Serial.println(" Left wheel velocity: " << cmd.data[1]);
    Serial.println(" Right wheel velocity: " << cmd.data[2] << "\n");
	_pos = (char) cmd.data[0];
	_lSp = (char) cmd.data[1];
	_rSp = (char) cmd.data[2];	
}
*/


unsigned int JetsonCommander::GetLeftRPMCmd() {
  return _leftRPMCmd;
}

unsigned int JetsonCommander::GetRightRPMCmd() {
  return _rightRPMCmd;
}

unsigned char JetsonCommander::GetPositionCmd() {
  return _posCmd;
}

