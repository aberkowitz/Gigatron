/**
 * context.cpp
 * Gigatron motor control Arduino code.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Syler Wagner  <syler@mit.edu>
 *
 * @date    2015-09-16    syler   fixed odometry message sending
 * @date    2016-01-10    syler   moved PID controller to separate class
 * 
 **/

#include <Arduino.h>
#include "Servo.h"
#include "isr.h"
#include "classes.h"
#include "commander.h"
#include "context.h"
 
Servo leftMotor;
Servo rightMotor;

Context::Context(Commander *commander, DCServo *servo,
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
  ) {
  _commander = commander;
  _servo = servo;
  _left = left;
  _right = right;
  _lPwm = lPwm;
  _rPwm = rPwm;
  _lRev = lRev;
  _rRev = rRev;  
  _lSp = lSp;
  _rSp = rSp;
  _pos = pos;

  _nh = nh; //$ ROS node handle
  _jcommander = jcommander; //$ Jetson commander

  //$ ROS publishers and messages
  _radio_msg = radio_msg; 
  _radio_pub = radio_pub;
  _steer_msg = steer_msg; 
  _steer_pub = steer_pub;
  _mot_msg = mot_msg; 
  _mot_pub = mot_pub;
  
  pinMode(_lPwm, OUTPUT);
  pinMode(_rPwm, OUTPUT);
  //pinMode(_lRev, OUTPUT);
  //pinMode(_rRev, OUTPUT);
  //digitalWrite(_lRev, HIGH);
  //digitalWrite(_rRev, HIGH);

  //ROBOCLAW
  leftMotor.attach(_lPwm);
  rightMotor.attach(_rPwm);
}

/*$ Configure time intervals for speed (drive motor) and 
  position (steering servo) loops.
  @param  sInterval  [ms] speed loop interval 
  @param  pInterval  [ms] position loop interval 
  */
  void Context::ConfigureLoop(int sInterval, int pInterval) {
    _sInterval = sInterval;
    _pInterval = pInterval;
  }


  void Context::Start() {

    //$ clear messages
    _radio_msg->speed_left = 0;
    _radio_msg->speed_right = 0;
    _radio_msg->dir_left = 0;
    _radio_msg->dir_right = 0;
    _radio_msg->angle = 128;
    _radio_msg->kill = 0;

    _steer_msg->angle = 128;
    _steer_msg->angle_command = 128;

    _mot_msg->rpm_left = 0;
    _mot_msg->rpm_right = 0;
    _mot_msg->usec_left = 1500;
    _mot_msg->usec_right = 1500;

    _last_st = _last_pt = millis();

  //unsigned int oldMode = _jcommander->_autonomous;
    unsigned int oldMode = 2;

    for (;;) {
    _nh->spinOnce(); //$ spin node handle
    
    unsigned long t = millis();
    unsigned long d_st = t - _last_st;
    unsigned long d_pt = t - _last_pt;

        // KILLSWITCH ENGAGE \m/
    if (_commander->GetKillCmd() > 75) {
      if (_jcommander->_autonomous == 0) {
        _jcommander->_autonomous = oldMode;
        //$ HALP IT'S GOING IN REVERSE
//        digitalWrite(_lRev, LOW);
        //digitalWrite(_rRev, HIGH); 
        //digitalWrite(_lRev, HIGH); 

      }
    }
    else {
      if (_jcommander->_autonomous > 0) {
        oldMode = _jcommander->_autonomous;
      }
      _jcommander->_autonomous = 0;
    }

    //$ left and right speed commands
    int lSpC;
    int rSpC;

      //left and right microsecond write values for ROBOCLAW
    unsigned int luSec;
    unsigned int ruSec;
    
    if (d_st > _sInterval) {  //$ speed (drive motor) loop
      //$ left and right speed commands

      int l_ticks = _left->GetTicks();
      int r_ticks = _right->GetTicks();
      //$ get values from RC commander or Jetson commander
      if (_jcommander->_autonomous > 1) { //$ fully autonomous mode

        //$ commanded values
        int lRPMC = _jcommander->GetLeftVelCmd();
        int rRPMC = _jcommander->GetRightVelCmd();
        

        //$ update PID controllers
//        lSpC = - _lSp->Update(lRPMC, l_ticks);
//        rSpC = - _rSp->Update(rRPMC, r_ticks);

        

        // //$ sending open loop commands between -250 (max forward) and 250 (max backward)
         lSpC = lRPMC;
         rSpC = rRPMC;
      }
      else { //$ RC mode and semiautomatic mode
        lSpC = _commander->GetLeftVelCmd();
        rSpC = _commander->GetRightVelCmd();
      }

      //$ convert to RoboClaw format of
      //$ servo-style timed pulses (1250-1750)
      luSec = (unsigned int) 1500 + lSpC;
      ruSec = (unsigned int) 1500 + rSpC;

      //$ write to RoboClaw motor controller
      leftMotor.writeMicroseconds(luSec);
      rightMotor.writeMicroseconds(ruSec);

      _last_st = t;

      // double leftWheelRPM = (double) _left->GetSpeed();
      // double rightWheelRPM = (double) _right->GetSpeed();
      
      // //$ write wheel velocities
      // _mot_msg->rpm_left = leftWheelRPM; * RPM_TO_M_S;
      // _mot_msg->rpm_right = rightWheelRPM * RPM_TO_M_S;
      
      //$ write wheel velocities
      _mot_msg->rpm_left = l_ticks;
      _mot_msg->rpm_right = r_ticks;
      _mot_msg->usec_left = ruSec;
      _mot_msg->usec_right = luSec;

      //$ publish message
      _mot_pub->publish(_mot_msg);

      //$ write 
      _radio_msg->speed_left = _commander->GetLeftVelCmd();
      _radio_msg->speed_right = _commander->GetRightVelCmd();
//      _radio_msg->dir_left = _commander->GetLeftDirectionCmd(); //$ TODO: remove from messages
//      _radio_msg->dir_right = _commander->GetRightDirectionCmd();
      _radio_msg->angle = _commander->GetPositionCmd();
      _radio_msg->kill = _commander->GetKillCmd();

      _radio_pub->publish(_radio_msg);

    }

    if (d_pt > _pInterval) { //$ position (steering servo) loop
      unsigned char pC;
      if (_jcommander->_autonomous == 0) { //$ RC mode
        pC = _commander->GetPositionCmd();
      }
      else  { //$ mixed mode and fully autonomous mode
        pC = _jcommander->GetPositionCmd();
      }  
      unsigned char pS = _servo->GetPosLinearized();

      int vel = _pos->Update(pC, pS); //$ update PID controller

      //$ command analogWrite/digitalWrite
      _servo->SetVelocity(vel);

      _last_pt = t;

      //$ steering servo position and Hall effect readings
      double servoPWM = (double) pS;
      double steeringAngle = STEERING_ANGLE_RANGE * (servoPWM / STEERING_PWM_RANGE) - ABS_MAX_STEERING_ANGLE;

      //$ write steering angle and servo PWM command to message
      _steer_msg->angle = pS;
      _steer_msg->angle_command = pC;
      
      //$ publish message
      _steer_pub->publish(_steer_msg);

    }
  }
}



