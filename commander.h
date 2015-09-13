#ifndef __COMMANDER_H
#define __COMMANDER_H

#include <Arduino.h>
#include "classes.h"
#include "shared.h"
#include <ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>

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

class JetsonCommander: public Commander { //$ wooo
public:
  JetsonCommander(ros::NodeHandle *nh);
//  virtual void CmdCallback(const std_msgs::Int16MultiArray::ConstPtr& cmd);
  virtual unsigned char GetLeftSpeedCmd();
  virtual unsigned char GetRightSpeedCmd();
  virtual unsigned char GetPositionCmd();
  unsigned char _leftCmd, _rightCmd, _posCmd;
private:
  ros::NodeHandle *_nh;
 // ros::Subscriber _sub;
};


  
#endif

