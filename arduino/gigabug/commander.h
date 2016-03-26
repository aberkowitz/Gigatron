/**
 * commander.h
 * Gigatron motor control Arduino code Jetson and RC commanders.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Syler Wagner  <syler@mit.edu>
 * @author  Daniel Gonzalez   <dgonz@mit.edu>
 *
 * @date    2015-09-16    syler   fixed odometry message sending
 **/

#ifndef __COMMANDER_H
#define __COMMANDER_H

#include <Arduino.h>
#include "classes.h"
#include "isr.h"
#include <ros.h>

class Commander {
public:
  virtual unsigned char GetLeftSpeedCmd(){return 0;}
  virtual unsigned char GetRightSpeedCmd(){return 0;}
  virtual unsigned char GetPositionCmd(){return 0;}
  virtual unsigned char GetKillCmd(){return 0;}
  virtual int GetLeftDirectionCmd(){return 1;}
  virtual int GetRightDirectionCmd(){return 1;}
};

class RCCommander: public Commander {
public:
  RCCommander(RCDecoder *sp, RCDecoder *pos,  RCDecoder *kill);
  virtual unsigned char GetLeftSpeedCmd();
  virtual unsigned char GetRightSpeedCmd();
  virtual unsigned char GetPositionCmd();
  virtual unsigned char GetKillCmd();
  virtual int GetLeftDirectionCmd();
  virtual int GetRightDirectionCmd();
private:
  RCDecoder *_sp, *_pos, *_kill;
};

class JetsonCommander: public Commander { //$ wooo
public:
  JetsonCommander(ros::NodeHandle *nh);
  unsigned int GetLeftRPMCmd();
  unsigned int GetRightRPMCmd();
  virtual unsigned char GetPositionCmd();
  unsigned char _posCmd;
  unsigned int _leftRPMCmd, _rightRPMCmd;
  
  unsigned int _autonomous;
/*
  boolean _jetsonMode; //$ true for Jetson control, false for RC
  boolean _semiautomaticMode; //$ true for Jetson control, false for RCprivate:
*/
  ros::NodeHandle *_nh;
};
  
#endif


