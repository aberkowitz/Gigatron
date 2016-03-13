/**
 * context.h
 * Gigatron motor control Arduino code.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Syler Wagner  <syler@mit.edu>
 *
 * @date    2015-09-16    syler   fixed odometry message sending
 * @date    2016-01-10    syler   moved PID controller to separate class
 *
 **/

#ifndef __CONTEXT_H
#define __CONTEXT_H

#include <Arduino.h>
#include "isr.h"
#include "classes.h"
#include "commander.h"
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>

#include <gigatron/Radio.h>
#include <gigatron/Steering.h>
#include <gigatron/Motors.h>


class Context {
public:
  Context(Commander *commander, DCServo *servo,
          SpeedSensor *left, SpeedSensor *right,
          int lPwm, int rPwm,
          int lRev, int rRev,
          PIDController *lSp, PIDController *rSp,
          PIDController *pos,
          ros::NodeHandle *nh,
          JetsonCommander *jcommander,
          gigatron::Radio *radio_msg,
          ros::Publisher *radio_pub,
          gigatron::Steering *steer_msg,
          ros::Publisher *steer_pub,
          gigatron::Motors *mot_msg,
          ros::Publisher *mot_pub
          );
  void ConfigureLoop(int sInterval, int pInterval);
  void Start();

private:
  Commander *_commander;
  DCServo *_servo;
  SpeedSensor *_left, *_right;
  int _lPwm, _rPwm, _lRev, _rRev;
  PIDController *_lSp, *_rSp, *_pos;
  int _sInterval, _pInterval;
  
  unsigned long _last_st, _last_pt;

  //$
  ros::NodeHandle *_nh;
  JetsonCommander *_jcommander;

  gigatron::Radio *_radio_msg;
  ros::Publisher *_radio_pub;

  gigatron::Steering *_steer_msg;
  ros::Publisher *_steer_pub;

  gigatron::Motors *_mot_msg;
  ros::Publisher *_mot_pub;
};

#endif

