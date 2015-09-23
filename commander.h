/**
 * commander.h
 * Gigatron motor control Arduino code.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Syler Wagner  <syler@mit.edu>
 *
 * @date    2015-09-16    syler   fixed odometry message sending
 **/

 #ifndef __COMMANDER_H
#define __COMMANDER_H

#include <Arduino.h>
#include "classes.h"
#include "shared.h"
#include <ros.h>

class Commander {
public:
  virtual unsigned char GetLeftSpeedCmd(){return 0;}
  virtual unsigned char GetRightSpeedCmd(){return 0;}
  virtual unsigned char GetPositionCmd(){return 0;}
  virtual int GetLeftDirectionCmd(){return 1;}
  virtual int GetRightDirectionCmd(){return 1;}
};

class RCCommander: public Commander {
public:
  RCCommander(RCDecoder *sp, RCDecoder *pos);
  virtual unsigned char GetLeftSpeedCmd();
  virtual unsigned char GetRightSpeedCmd();
  virtual unsigned char GetPositionCmd();
  virtual int GetLeftDirectionCmd();
  virtual int GetRightDirectionCmd();
private:
  RCDecoder *_sp, *_pos;
};

class JetsonCommander: public Commander { //$ wooo
public:
  JetsonCommander(ros::NodeHandle *nh);
  unsigned int GetLeftRPMCmd();
  unsigned int GetRightRPMCmd();
  virtual unsigned char GetPositionCmd();
  unsigned char _posCmd;
  unsigned int _leftRPMCmd, _rightRPMCmd;
  boolean _jetsonMode; //$ true for Jetson control, false for RC
  boolean _semiautomaticMode; //$ true for Jetson control, false for RCprivate:
  ros::NodeHandle *_nh;
};


  
#endif

