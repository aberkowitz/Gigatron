#ifndef __CONTEXT_H
#define __CONTEXT_H

#include <Arduino.h>
#include "shared.h"
#include "classes.h"
#include "commander.h"
#include <ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt16MultiArray.h>

class PidController {
public:
  PidController(long kp, long ki, long kd, long out_max, long out_min);
  int Update(int ref, int in);
  int ZeroIntegrator();
private:
  long _kp, _ki, _kd;
  long _out_max, _out_min;
  
  int _last_in;
  long _integral;
};

class Context {
public:
  Context(Commander *commander, DCServo *servo,
          SpeedSensor *left, SpeedSensor *right,
          int lPwm, int rPwm,
          PidController *lSp, PidController *rSp,
          PidController *pos,
          ros::NodeHandle *nh,
          JetsonCommander *jcommander,
          std_msgs::UInt16MultiArray *odomsg,
          ros::Publisher *pub);
  void ConfigureLoop(int sInterval, int pInterval);
  void Start();
private:
  Commander *_commander;
  DCServo *_servo;
  SpeedSensor *_left, *_right;
  int _lPwm, _rPwm;
  PidController *_lSp, *_rSp, *_pos;
  int _sInterval, _pInterval;
  
  unsigned long _last_st, _last_pt;

  //$
  ros::NodeHandle *_nh;
  JetsonCommander *_jcommander;
  std_msgs::UInt16MultiArray *_odomsg;
  ros::Publisher *_pub;
  boolean _jetsonMode; 

};

#endif

