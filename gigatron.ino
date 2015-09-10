#include "classes.h"
#include "commander.h"
#include "context.h"
#include "shared.h"

#define LOOP_INTERVAL 10

void setup() {
  Serial.begin(38400);
  RCDecoder pos(0, 984, 2004);
  RCDecoder sp(1, 1480, 1990);
  SpeedSensor left(2, 14, LOOP_INTERVAL);
  SpeedSensor right(3, 14, LOOP_INTERVAL);
  DCServo servo(5, 4, A0);
  servo.ConfigSensor(280, 430);
  RCCommander rc(&sp, &pos);
  PidController lSp(0, 100, 0, 255, 0);
  PidController rSp(0, 100, 0, 255, 0);
  PidController pPos(500, 0, 100, 255, -255);
  Context context(&rc, &servo, &left, &right, 9, 10, &lSp, &rSp, &pPos);
  context.ConfigureLoop(LOOP_INTERVAL, LOOP_INTERVAL);
  context.Start();
}

void loop() {
}

