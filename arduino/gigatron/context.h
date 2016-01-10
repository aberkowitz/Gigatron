/**
 * context.h
 * Gigatron motor control Arduino code.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Syler Wagner  <syler@mit.edu>
 *
 * @date    2015-09-16    syler   fixed odometry message sending
 **/

#ifndef __CONTEXT_H
#define __CONTEXT_H

#include <Arduino.h>
#include "shared.h"
#include "classes.h"
#include "commander.h"
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>

class PidController {
public:
  PidController(long kp, long ki, long kd, long out_max, long out_min);
  int Update(int ref, int in);
  int ZeroIntegrator();
  void ResetGains(long kp, long ki, long kd);
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
          int lRev, int rRev,
          PidController *lSp, PidController *rSp,
          PidController *pos,
          ros::NodeHandle *nh,
          JetsonCommander *jcommander,
          geometry_msgs::Vector3 *odomsg,
          ros::Publisher *pub,
          geometry_msgs::Vector3 *commsg,
          ros::Publisher *compub,
          std_msgs::Float32 *angmsg,
          ros::Publisher *angpub,
          std_msgs::Float32 *angcommsg,
          ros::Publisher *angcompub);
  void ConfigureLoop(int sInterval, int pInterval);
  void Start();

private:
  Commander *_commander;
  DCServo *_servo;
  SpeedSensor *_left, *_right;
  int _lPwm, _rPwm, _lRev, _rRev;
  PidController *_lSp, *_rSp, *_pos;
  int _sInterval, _pInterval;
  
  unsigned long _last_st, _last_pt;

  //$
  ros::NodeHandle *_nh;
  JetsonCommander *_jcommander;

  geometry_msgs::Vector3 *_odomsg;
  ros::Publisher *_pub;

  geometry_msgs::Vector3 *_commsg;
  ros::Publisher *_compub;

  std_msgs::Float32 *_angmsg;
  ros::Publisher *_angpub;

  std_msgs::Float32 *_angcommsg;
  ros::Publisher *_angcompub;

};

#endif

