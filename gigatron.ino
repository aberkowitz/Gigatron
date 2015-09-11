#include "classes.h"
#include "commander.h"
#include "context.h"
#include "shared.h"
#include <ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>

#define LOOP_INTERVAL 10

ros::NodeHandle nh; //$ node handle
JetsonCommander jc(&nh);  //$ Jetson commander
std_msgs::Int16MultiArray odomsg; //$ odometry message

void CmdCallback(const std_msgs::Int16MultiArray& cmd) {
/*Serial.println("Steering angle: " << cmd.data[0]);
  Serial.println(" Left wheel velocity: " << cmd.data[1]);
  Serial.println(" Right wheel velocity: " << cmd.data[2] << "\n");
*/
  jc._posCmd = (char) cmd.data[0];
  jc._leftCmd = (char) cmd.data[1];
  jc._rightCmd = (char) cmd.data[2];  
}

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
 // ros::NodeHandle nh;
 // JetsonCommander jc(&nh);
  ros::Subscriber<std_msgs::Int16MultiArray> sub("control", CmdCallback);
  nh.subscribe(sub);
  ros::Publisher pub("odo_val", &odomsg);
  nh.advertise(pub);

  /* Context(Commander *commander, DCServo *servo,
          SpeedSensor *left, SpeedSensor *right,
          int lPwm, int rPwm,
          PidController *lSp, PidController *rSp,
          PidController *pos,
          ros::NodeHandle *nh,
          JetsonCommander *jcommander,
          std_msgs::Int16MultiArray *odomsg,
          ros::Publisher *pub); */
  Context context(&rc, &servo, &left, &right, 9, 10, &lSp, &rSp, &pPos, &nh, &jc, &odomsg, &pub);

  // Context::ConfigureLoop(int sInterval, int pInterval);
  context.ConfigureLoop(LOOP_INTERVAL, LOOP_INTERVAL);

  context.Start(); // the actual looping happens here
}

void loop() {
}

