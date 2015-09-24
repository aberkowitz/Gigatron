
/**
 * gigatron.ino
 * Gigatron motor control Arduino code.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Syler Wagner  <syler@mit.edu>
 *
 * @date    2015-09-16    syler   fixed odometry message sending
 * @date    2015-09-16    syler   added PID gain tuning via ROS messages
 **/

#include "classes.h"
#include "commander.h"
#include "context.h"
#include "shared.h"
#include <ros.h>
#include <geometry_msgs/Vector3.h> 
#include <std_msgs/Float32.h> //$ for steering odometry stuff and mode
#include <std_msgs/UInt16.h> //$ for mode switching

#define PI 3.1415926535897932384626433832795
 
#define LOOP_INTERVAL 10
#define S_LOOP_INTERVAL 85

//$ steering pot calibration
int minADU = 449;
int maxADU = 631; //$ 

const static double INCHES_TO_M = 0.0254; //$ conversion from inches to meters

//$ car dimensions
const static double wheelBaseWidth = 23.0 * INCHES_TO_M;  //$ [m]
const static double wheelRadius = 4.90 * INCHES_TO_M;     //$ [m]
const static double gearRatio = 11.0 / 60.0;  //$ gear ratio between motor and wheels

//pin 30 left reverse
//pin 31 right reverse


//$ constants
const static double RPM_TO_M_S = (2 * PI * wheelRadius) / 60.0;   //$ conversion from RPM to meters per second

const static double STEERING_PWM_RANGE = 255.0;
const static double STEERING_ANGLE_RANGE = 50 * (PI / 180); //$ [radians] this is the correct steering range
const static double ABS_MAX_STEERING_ANGLE = 25 * (PI / 180); //$ [radians]

ros::NodeHandle nh;       //$ node handle

// JetsonCommander(ros::NodeHandle *nh);
JetsonCommander jc(&nh);  //$ Jetson commander

PidController lSp(2, 1, 1, 255, 0);
PidController rSp(2, 1, 1, 255, 0);

PidController pPos(150, 0, 0, 255, -255);

geometry_msgs::Vector3 odomsg;  //$ odometry message
std_msgs::Float32 angmsg;       //$ measured steering angle message
geometry_msgs::Vector3 commsg;  //$ command message
std_msgs::Float32 angcommsg;    //$ command message
//$ TODO: fix sending back odometry values on separate channels


void CmdCallback(const geometry_msgs::Vector3& cmd) {
  
  double desiredSteeringAngle = cmd.x;
  unsigned char servoPWM = (desiredSteeringAngle + ABS_MAX_STEERING_ANGLE) * (STEERING_PWM_RANGE / STEERING_ANGLE_RANGE);

  jc._posCmd = servoPWM;
  jc._leftRPMCmd = (unsigned int) (cmd.y / RPM_TO_M_S);
  jc._rightRPMCmd = (unsigned int) (cmd.z / RPM_TO_M_S);
  //jc._rightRPMCmd = 1200;
}

/*$ Swith between radio RC and autonomous/Jetson RC mode.
  */
void SwitchCallback(const std_msgs::UInt16& mode) {
  
  jc._autonomous = mode.data;
/*
 if (mode.data == 0) {
   jc._jetsonMode = false;
   jc._semiautomaticMode = false;
 }
 else if (mode.data == 2) {
   jc._jetsonMode = true;
   jc._semiautomaticMode = false;
 }
 else if (mode.data == 1) { //$ manual throttle, autonomous steering
   jc._jetsonMode = false;
   jc._semiautomaticMode = true;
 }*/
}

/*$ Set PID controller gains for both drive motors with a 
  Vector3 ROS message (kp, ki, kd) published on the /gains 
  topic.
  */
void GainsCallback(const geometry_msgs::Vector3& gain) {
  long kp = (long) gain.x;
  long ki = (long) gain.y;
  long kd = (long) gain.z;
  lSp.ResetGains(kp, ki, kd);
  rSp.ResetGains(kp, ki, kd);
}

void setup() {
  Serial.begin(38400);

  // RCDecoder(int interrupt, int minV, int maxV);
  RCDecoder pos(0, 984, 2004); 
  //Was 1480, expanded to add reverse
  RCDecoder sp(1, 1020, 1990);

  // SpeedSensor(int interrupt, int poles, int interval);
  SpeedSensor left(4, 14, S_LOOP_INTERVAL); 
  SpeedSensor right(5, 14, S_LOOP_INTERVAL);

  // DCServo(int pwmPin, int dirPin, int posPin);
  DCServo servo(5, 4, A0); 

  // DCServo::ConfigSensor(int minV, int maxV);
  servo.ConfigSensor(minADU, maxADU);
  RCCommander rc(&sp, &pos);

  /*$ The PID controller definitions got moved up top so that GainsCallback works */
    // PidController(long kp, long ki, long kd, long out_max, long out_min);
    //PidController lSp(0, 100, 0, 255, 0);
    //PidController rSp(0, 100, 0, 255, 0);

  //$ set up subscribers
  ros::Subscriber<geometry_msgs::Vector3> sub("control", CmdCallback);
  nh.subscribe(sub);
  ros::Subscriber<std_msgs::UInt16> switchsub("switch", SwitchCallback);
  nh.subscribe(switchsub);
  ros::Subscriber<geometry_msgs::Vector3> gainsub("gains", GainsCallback);
  nh.subscribe(gainsub);

  //$ set up publishers
  ros::Publisher pub("odo_val", &odomsg);
  nh.advertise(pub);
  ros::Publisher compub("command", &commsg);
  nh.advertise(compub);
  ros::Publisher angpub("ang_val", &angmsg);
  nh.advertise(angpub);
  ros::Publisher angcompub("ang_command", &angcommsg);
  nh.advertise(angcompub);

  /* Context(Commander *commander, DCServo *servo,
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
          ros::Publisher *angcompub) */
  Context context(&rc, &servo, &left, &right, 9, 10, 30, 31, &lSp, &rSp, &pPos, &nh, &jc, &odomsg, &pub, &commsg, &compub, &angmsg, &angpub, &angcommsg, &angcompub);

  // Context::ConfigureLoop(int sInterval, int pInterval);
  context.ConfigureLoop(S_LOOP_INTERVAL, LOOP_INTERVAL);

  context.Start(); // the actual looping happens here
}

void loop() {
}

