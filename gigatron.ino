#include "classes.h"
#include "commander.h"
#include "context.h"
#include "shared.h"
#include <ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>

#define LOOP_INTERVAL 10

void setup() {
  Serial.begin(38400);

  // RCDecoder(int interrupt, int minV, int maxV);
  RCDecoder pos(0, 984, 2004); 
  RCDecoder sp(1, 1480, 1990);

  // SpeedSensor(int interrupt, int poles, int interval);
  SpeedSensor left(2, 14, LOOP_INTERVAL); 
  SpeedSensor right(3, 14, LOOP_INTERVAL);

  // DCServo(int pwmPin, int dirPin, int posPin);
  DCServo servo(5, 4, A0); 

  // DCServo::ConfigSensor(int minV, int maxV);
  servo.ConfigSensor(280, 430);
  RCCommander rc(&sp, &pos);

  // PidController(long kp, long ki, long kd, long out_max, long out_min);
  PidController lSp(0, 100, 0, 255, 0);
  PidController rSp(0, 100, 0, 255, 0);
  PidController pPos(500, 0, 100, 255, -255);

  //$
  ros::NodeHandle nh;
  JetsonCommander jc(&nh);

  /* Context(Commander *commander, DCServo *servo,
          SpeedSensor *left, SpeedSensor *right,
          int lPwm, int rPwm,
          PidController *lSp, PidController *rSp,
          PidController *pos); */
  Context context(&rc, &servo, &left, &right, 9, 10, &lSp, &rSp, &pPos, &nh, &jc);

  // Context::ConfigureLoop(int sInterval, int pInterval);
  context.ConfigureLoop(LOOP_INTERVAL, LOOP_INTERVAL);

  context.Start(); // the actual looping happens here
}

void loop() {
}

