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
          ros::Publisher *compub) {
  _commander = commander;
  _servo = servo;
  _left = left;
  _right = right;
  _lPwm = lPwm;
  _rPwm = rPwm;  
  _lSp = lSp;
  _rSp = rSp;
  _pos = pos;

  _nh = nh; //$
  _jcommander = jcommander; //$
  _odomsg = odomsg; //$
  _pub = pub;
  _commsg = commsg; //$
  _compub = compub;

  _jetsonMode = true; //$ TODO: implement switching
  
  pinMode(_lPwm, OUTPUT);
  pinMode(_rPwm, OUTPUT);
}

void Context::ConfigureLoop(int sInterval, int pInterval) {
  _sInterval = sInterval;
  _pInterval = pInterval;
}

void Context::Start() {
  _last_st = _last_pt = millis();
  for (;;) {
    _nh->spinOnce(); //$ spin node handle
    
    unsigned long t = millis();
    unsigned long d_st = t - _last_st;
    unsigned long d_pt = t - _last_pt;

    if (d_st > _sInterval || d_pt > _pInterval) {
      //$ clear messages
      _odomsg->x = 128;
      _odomsg->y = 0;
      _odomsg->z = 0;
      _commsg->x = 128;
      _commsg->y = 0;
      _commsg->z = 0;
      if (d_st > _sInterval) {
        //$ left and right speed commands
        unsigned int lSpC;
        unsigned int rSpC;

        //$ get values from RC commander or Jetson commander
        if (_jetsonMode) { //$ Jetson mode
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

        //$ write PWM commands to message
        _commsg->y = lSpC;
        _commsg->z = rSpC;

        _last_st = t;
      }
      if (d_pt > _pInterval) {
        unsigned char pC;
        if (_jetsonMode) { //$ Jetson mode
          pC = _jcommander->GetPositionCmd();
        }
        else { //$ RC mode
          pC = _commander->GetPositionCmd();
        }  
        unsigned char pS = _servo->GetPos();
        
        //dp(pC);
        //dp(pS);

        int vel = _pos->Update(pC, pS);
        
        //dp(vel);
        // command analogWrite/digitalWrite
        _servo->SetVelocity(vel);

        //$ publish steering servo position and Hall effect readings
        
        double servoPWM = (double) pS;
        double leftWheelRPM = (double) _left->GetSpeed();
        double rightWheelRPM = (double) _right->GetSpeed();

        double steeringAngle = STEERING_ANGLE_RANGE * (servoPWM / STEERING_PWM_RANGE) - ABS_MAX_STEERING_ANGLE;

        _odomsg->x = steeringAngle;
        _odomsg->y = leftWheelRPM * RPM_TO_M_S;
        _odomsg->z = rightWheelRPM * RPM_TO_M_S;

        //$ publish servo PWM command
        _commsg->x = pC;

        _last_pt = t;
      }
      //$ publish messages
      _pub->publish(_odomsg);
      _compub->publish(_commsg);
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

