#include "classes.h"
#include "commander.h"
#include "context.h"
#include "shared.h"
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#define PI 3.1415926535897932384626433832795
 
#define LOOP_INTERVAL 10
#define S_LOOP_INTERVAL 100

const static double INCHES_TO_M = 0.0254; //$ conversion from inches to meters
//const static double PI = 3.141592653589793238463;

const static double wheelBaseWidth = 23.0 * INCHES_TO_M;  //$ [m]
const static double wheelRadius = 4.90 * INCHES_TO_M;     //$ [m]
const static double gearRatio = 11.0 / 60.0;  //$ gear ratio between motor and wheels

const static double RPM_TO_M_S = (2 * PI * wheelRadius) / 60.0;   //$ conversion from RPM to meters per second

const static double STEERING_PWM_RANGE = 255.0;
const static double STEERING_ANGLE_RANGE = 50 * (PI / 180); //$ [radians] this is the correct steering range
const static double ABS_MAX_STEERING_ANGLE = 25 * (PI / 180); //$ [radians]

ros::NodeHandle nh; //$ node handle
JetsonCommander jc(&nh);  //$ Jetson commander
geometry_msgs::Vector3 odomsg; //$ odometry message
geometry_msgs::Vector3 commsg; //$ command message


void CmdCallback(const geometry_msgs::Vector3& cmd) {
  
  double desiredSteeringAngle = cmd.x;

  unsigned char servoPWM = (desiredSteeringAngle + ABS_MAX_STEERING_ANGLE) * (STEERING_PWM_RANGE / STEERING_ANGLE_RANGE);

  jc._posCmd = servoPWM;
  jc._leftRPMCmd = (unsigned int) (cmd.y / RPM_TO_M_S);
  jc._rightRPMCmd = (unsigned int) (cmd.z / RPM_TO_M_S);
}

void setup() {
  Serial.begin(38400);

  // RCDecoder(int interrupt, int minV, int maxV);
  RCDecoder pos(0, 984, 2004); 
  RCDecoder sp(1, 1480, 1990);

  // SpeedSensor(int interrupt, int poles, int interval);
  SpeedSensor left(4, 14, S_LOOP_INTERVAL); 
  SpeedSensor right(5, 14, S_LOOP_INTERVAL);

  // DCServo(int pwmPin, int dirPin, int posPin);
  DCServo servo(5, 4, A0); 

  // DCServo::ConfigSensor(int minV, int maxV);
  servo.ConfigSensor(232, 463);
  RCCommander rc(&sp, &pos);

  // PidController(long kp, long ki, long kd, long out_max, long out_min);
  //PidController lSp(0, 100, 0, 255, 0);
  //PidController rSp(0, 100, 0, 255, 0);

  PidController lSp(0, 5, 0, 255, 0);
  PidController rSp(0, 5, 0, 255, 0);

  PidController pPos(500, 0, 100, 255, -255);

  //$
 // ros::NodeHandle nh;
 // JetsonCommander jc(&nh);
  ros::Subscriber<geometry_msgs::Vector3> sub("control", CmdCallback);
  nh.subscribe(sub);
  ros::Publisher pub("odo_val", &odomsg);
  nh.advertise(pub);
  ros::Publisher compub("command", &commsg);
  nh.advertise(compub);

  /* Context(Commander *commander, DCServo *servo,
          SpeedSensor *left, SpeedSensor *right,
          int lPwm, int rPwm,
          PidController *lSp, PidController *rSp,
          PidController *pos,
          ros::NodeHandle *nh,
          JetsonCommander *jcommander,
          std_msgs::Int16MultiArray *odomsg,
          ros::Publisher *pub); */
  Context context(&rc, &servo, &left, &right, 9, 10, &lSp, &rSp, &pPos, &nh, &jc, &odomsg, &pub, &commsg, &compub);

  // Context::ConfigureLoop(int sInterval, int pInterval);
  context.ConfigureLoop(S_LOOP_INTERVAL, LOOP_INTERVAL);

  context.Start(); // the actual looping happens here
}

void loop() {
}

