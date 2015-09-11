#include <Arduino.h>
#include "shared.h"
#include "classes.h"
#include "commander.h"
#include "context.h"

Context::Context(Commander *commander, DCServo *servo,
          SpeedSensor *left, SpeedSensor *right,
          int lPwm, int rPwm,
          PidController *lSp, PidController *rSp,
          PidController *pos,
          ros::NodeHandle *nh,
          JetsonCommander *jcommander) {
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
    _nh.spinOnce();
    unsigned long t = millis();
    unsigned long d_st = t - _last_st;
    unsigned long d_pt = t - _last_pt;
    if (d_st > _sInterval) {
      unsigned char lSpC;
      unsigned char rSpC;
      if (_jetsonMode) {
        lSpC = _jcommander->GetLeftSpeedCmd();
        rSpC = _jcommander->GetRightSpeedCmd();
      }
      else { // RC mode
        lSpC = _commander->GetLeftSpeedCmd();
        rSpC = _commander->GetRightSpeedCmd();
      }
      analogWrite(_lPwm, lSpC);
      analogWrite(_rPwm, rSpC);
      _last_st = t;
    }
    if (d_pt > _pInterval) {
      unsigned char pC;
      if (_jetsonMode) {
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
      _servo->SetVelocity(vel);
      _last_pt = t;
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

