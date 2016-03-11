#include <Servo.h>


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
#include "isr.h"
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
  _lRev = lRev;
  _rRev = rRev;  
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

gigatron::RadioInput *radio_msg;  
gigatron::Steering *steer_msg;  
gigatron::Motors *mot_msg;  

    //$ set up publishers
  ros::Publisher radio_pub("arduino/radio", radio_msg);
  _nh->advertise(radio_pub);
  ros::Publisher mot_pub("arduino/motors", mot_msg);
  _nh->advertise(mot_pub);
  ros::Publisher steer_pub("arduino/steering", steer_msg);
  _nh->advertise(steer_pub);

  radio_msg->speed_left = -1;
  radio_msg->speed_right = -1;
  radio_msg->dir_left = -1;
  radio_msg->dir_right = -1;
  radio_msg->angle = -1;
  radio_msg->kill = -1;

  steer_msg->angle = -1;
  steer_msg->angle_command = -1;

  mot_msg->rpm_left = -1;
  mot_msg->rpm_right = -1;
  mot_msg->usec_left = -1;
  mot_msg->usec_right = -1;


  //$ clear messages
  // _odomsg->x = -1;
  // _odomsg->y = 0;
  // _odomsg->z = 0;
  // _commsg->x = -1;
  // _commsg->y = 0;
  // _commsg->z = 0;
  // _angmsg->data = 0;
  // _angcommsg->data = 128;
  
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

    //dp(_jcommander->_autonomous);
      unsigned int lSpC;
      unsigned int rSpC;
      //left and right microsecond write values for ROBOCLAW
      unsigned int luSec;
      unsigned int ruSec;
      int lDir;
      int rDir;

    if (d_st > _sInterval) {  //$ speed (drive motor) loop
      //$ left and right speed commands


      //$ get values from RC commander or Jetson commander
      if (_jcommander->_autonomous > 1) { //$ fully autonomous mode
        //Direction is forward by default
        //Autonomous Gigatron doesn't know how to go backwards yet
        lDir = 0;
        rDir = 0;
        //$ sensed RPM values
        unsigned int lRPMS = _left->GetSpeed();
        unsigned int rRPMS = _right->GetSpeed();
        //Serial.println(rRPMS); //Controller's perceived RPM 
        //$ commanded values
        unsigned int lRPMC = _jcommander->GetLeftRPMCmd();
        unsigned int rRPMC = _jcommander->GetRightRPMCmd();
        //$ update PID controllers
        lSpC = _lSp->Update(lRPMC, lRPMS);
        rSpC = _rSp->Update(rRPMC, rRPMS);
      }
      else { //$ RC mode and semiautomatic mode
        lSpC = _commander->GetLeftSpeedCmd();
        rSpC = _commander->GetRightSpeedCmd();
        lDir = _commander->GetLeftDirectionCmd();
        rDir = _commander->GetRightDirectionCmd();
        
        if (lDir == -1) {
          lSpC = 0;
        }
        else if (lDir == 1) {
          lSpC = (lSpC - 122) * (255 / (255 - 124));
          //digitalWrite(_lRev, HIGH);
        }
        else {
          lSpC = (255 -lSpC) * (255 / (255 - (255 - 116)));
          //digitalWrite(_lRev, LOW);
        }
        
        if (rDir == -1) {
          rSpC = 0;
        }
        else if (rDir == 1) {
          rSpC = (rSpC - 122) * (255 / (255 - 124));
          //digitalWrite(_rRev, HIGH);
        }
        else {
          rSpC = (255 - rSpC) * (255 / (255 - (255 - 116)));
          //digitalWrite(_rRev, LOW);
        }
      }
      //$ write commands
      //analogWrite(_lPwm, lSpC);
      //analogWrite(_rPwm, rSpC);

      //ROBOCLAW
      //This is a bit uglier than it maybe needs to be because everything is unsigned...
      if (lSpC > 250) {
        lSpC = 250;
      }
      if (rSpC > 250) {
        rSpC = 250;
      }
      if (lDir) {
        luSec = 1500 + lSpC;
      } else {
        luSec = 1500 - lSpC;
      }
      if (rDir) {
        ruSec = 1500 + rSpC;
      } else {
        ruSec = 1500 - rSpC;
      }
      leftMotor.writeMicroseconds(luSec);
      rightMotor.writeMicroseconds(ruSec);

      _last_st = t;

      //$ write PWM commands to command message
      //note that max is now 250, not 255
      // _commsg->y = lSpC;
      // _commsg->z = rSpC;

      double leftWheelRPM = (double) _left->GetSpeed();
      double rightWheelRPM = (double) _right->GetSpeed();
      
      //$ write wheel velocities
      // _odomsg->y = leftWheelRPM * RPM_TO_M_S;
      // _odomsg->z = rightWheelRPM * RPM_TO_M_S;

      // //$ publish messages
      // _pub->publish(_odomsg);
      // _compub->publish(_commsg);


  mot_msg->rpm_left = leftWheelRPM * RPM_TO_M_S;
  mot_msg->rpm_right = rightWheelRPM * RPM_TO_M_S;
  mot_msg->usec_left = ruSec;
  mot_msg->usec_right = luSec;

  mot_pub.publish(mot_msg);

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
      // _angmsg->data = steeringAngle;
      // _angcommsg->data = pC;

      // //$ publish messages
      // _angpub->publish(_angmsg);
      // _angcompub->publish(_angcommsg);


      /*$ NEW MESSAGES FOR DEBUGGING
       */

         steer_msg->angle = steeringAngle;
  steer_msg->angle_command = pC;

  steer_pub.publish(steer_msg);



      radio_msg->speed_left = _commander->GetLeftSpeedCmd();
  radio_msg->speed_right = _commander->GetRightSpeedCmd();
  radio_msg->dir_left = _commander->GetLeftDirectionCmd();
  radio_msg->dir_right = _commander->GetRightDirectionCmd();
  radio_msg->angle = _commander->GetPositionCmd();
  radio_msg->kill = _commander->GetKillCmd();

  radio_pub.publish(radio_msg);

    }
  }
}


