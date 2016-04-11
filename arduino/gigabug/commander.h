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
  virtual int GetLeftVelCmd() {return 0;}
  virtual int GetRightVelCmd() {return 0;}
  virtual unsigned char GetPositionCmd() {return 0;}
  virtual unsigned char GetKillCmd() {return 0;}
};

class RCCommander: public Commander {
public:
  RCCommander(RCDecoder *sp, RCDecoder *pos,  RCDecoder *kill);
  int GetLeftVelCmd();
  int GetRightVelCmd();
  unsigned char GetPositionCmd();
  unsigned char GetKillCmd();

private:
  RCDecoder *_sp, *_pos, *_kill;
};

class JetsonCommander: public Commander { //$ wooo
public:
  JetsonCommander(ros::NodeHandle *nh);
  int GetLeftVelCmd();
  int GetRightVelCmd();
  unsigned char GetPositionCmd();
  double _angle;
  double _left_vel, _right_vel;
  
  unsigned int _autonomous;
/*
  boolean _jetsonMode; //$ true for Jetson control, false for RC
  boolean _semiautomaticMode; //$ true for Jetson control, false for RCprivate:
*/
  ros::NodeHandle *_nh;
};
  
#endif


