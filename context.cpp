/**
 * context.cpp
 * Gigatron motor control Arduino code.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Syler Wagner  <syler@mit.edu>
 *
 * @date    2015-09-16    syler   fixed odometry message sending
 **/

#include <Arduino.h>
#include "shared.h"
#include "classes.h"
#include "commander.h"
#include "context.h"
#define PI 3.1415926535897932384626433832795

const static double INCHES_TO_M = 0.0254; //$ conversion from inches to meters
//const static double PI = 3.141592653589793238463;

const static double wheelBaseWidth = 23.0 * INCHES_TO_M;  //$ [m]
const static double wheelRadius = 4.90 * INCHES_TO_M;     //$ [m]
const static double gearRatio = 11.0 / 60.0;  //$ gear ratio between motor and wheels

const static double RPM_TO_M_S = (2 * PI * wheelRadius) / 60.0;   //$ conversion from RPM to meters per second

const static double STEERING_PWM_RANGE = 255.0;
const static double STEERING_ANGLE_RANGE = 50 * (PI / 180); //$ [radians] this is the correct steering range
const static double ABS_MAX_STEERING_ANGLE = 25 * (PI / 180); //$ [radians]

Context::Context(Commander *commander, DCServo *servo,
          SpeedSensor *left, SpeedSensor *right,
          int lPwm, int rPwm,
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
          ros::Publisher *angcompub) {
  _commander = commander;
  _servo = servo;
  _left = left;
  _right = right;
  _lPwm = lPwm;
  _rPwm = rPwm;  
  _lSp = lSp;
  _rSp = rSp;
  _pos = pos;

  _nh = nh; //$ ROS node handle
  _jcommander = jcommander; //$ Jetson commander
  
  //$ ROS publishers and messages
  _odomsg = odomsg; 
  _pub = pub;
  _commsg = commsg; 
  _compub = compub;
  _angmsg = angmsg; 
  _angpub = angpub;
  _angcommsg = angcommsg; 
  _angcompub = angcompub;
  
  pinMode(_lPwm, OUTPUT);
  pinMode(_rPwm, OUTPUT);
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
  _odomsg->x = -1;
  _odomsg->y = 0;
  _odomsg->z = 0;
  _commsg->x = -1;
  _commsg->y = 0;
  _commsg->z = 0;
  _angmsg->data = 0;
  _angcommsg->data = 128;
  
  _last_st = _last_pt = millis();

  for (;;) {
    _nh->spinOnce(); //$ spin node handle
    
    unsigned long t = millis();
    unsigned long d_st = t - _last_st;
    unsigned long d_pt = t - _last_pt;

    if (d_st > _sInterval) {  //$ speed (drive motor) loop
      //$ left and right speed commands
      unsigned int lSpC;
      unsigned int rSpC;

      //$ get values from RC commander or Jetson commander
      if (_jcommander->_jetsonMode) { //$ Jetson mode
        //$ sensed RPM values
        unsigned int lRPMS = _left->GetSpeed();
        unsigned int rRPMS = _right->GetSpeed();
        //$ commanded values
        unsigned int lRPMC = _jcommander->GetLeftRPMCmd();
        unsigned int rRPMC = _jcommander->GetRightRPMCmd();
        //$ update PID controllers
        lSpC = _lSp->Update(lRPMC, lRPMS);
        rSpC = _rSp->Update(rRPMC, rRPMS);
      }
      else { //$ RC mode
        lSpC = _commander->GetLeftSpeedCmd();
        rSpC = _commander->GetRightSpeedCmd();
      }
      //$ write commands
      analogWrite(_lPwm, lSpC);
      analogWrite(_rPwm, rSpC);

      _last_st = t;

      //$ write PWM commands to command message
      _commsg->y = lSpC;
      _commsg->z = rSpC;

      double leftWheelRPM = (double) _left->GetSpeed();
      double rightWheelRPM = (double) _right->GetSpeed();

      //$ write wheel velocities
      _odomsg->y = leftWheelRPM * RPM_TO_M_S;
      _odomsg->z = rightWheelRPM * RPM_TO_M_S;

      //$ publish messages
      _pub->publish(_odomsg);
      _compub->publish(_commsg);
    }
    if (d_pt > _pInterval) { //$ position (steering servo) loop
      unsigned char pC;
      if (_jcommander->_jetsonMode) { //$ Jetson mode
        pC = _jcommander->GetPositionCmd();
      }
      else { //$ RC mode
        pC = _commander->GetPositionCmd();
      }  
      unsigned char pS = _servo->GetPos();
      
      //dp(pC);
      //dp(pS);
      int vel = _pos->Update(pC, pS); //$ update PID controller
      //dp(vel);

      //$ command analogWrite/digitalWrite
      _servo->SetVelocity(vel);

      _last_pt = t;

      //$ steering servo position and Hall effect readings
      double servoPWM = (double) pS;
      double steeringAngle = STEERING_ANGLE_RANGE * (servoPWM / STEERING_PWM_RANGE) - ABS_MAX_STEERING_ANGLE;

      //$ write steering angle and servo PWM command to message
      _angmsg->data = steeringAngle;
      _angcommsg->data = pC;

      //$ publish messages
      _angpub->publish(_angmsg);
      _angcompub->publish(_angcommsg);

    }
  }
}


PidController::PidController(long kp, long ki, long kd, long out_max, long out_min) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _last_in = 0;
  _integral = 0;
  _out_max = out_max;
  _out_min = out_min;
}

void PidController::ResetGains(long kp, long ki, long kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _last_in = 0;
  _integral = 0;
}

int PidController::Update(int ref, int in) {
  int error = ref - in;

  _integral += error;
  
  long tmp = _integral * _ki >> 8;
  if (tmp > _out_max) _integral = (_out_max << 8) / _ki;
  if (tmp < _out_min) _integral = (_out_min << 8) / _ki;
  
  int deriv = _last_in - in;
  _last_in = in;
  tmp = _ki * _integral + _kp * error + _kd * deriv;
  tmp = tmp >> 8;
  if (tmp > _out_max) tmp = _out_max;
  if (tmp < _out_min) tmp = _out_min;
  return tmp;
}

