#include <Arduino.h>
#include "classes.h"
#include "shared.h"
#include "commander.h"

RCCommander::RCCommander(RCDecoder *sp, RCDecoder *pos) {
  _sp = sp;
  _pos = pos;
}

unsigned char RCCommander::GetLeftSpeedCmd() {
  return _sp->GetVal();
}

unsigned char RCCommander::GetRightSpeedCmd() {
  return _sp->GetVal();
}

unsigned char RCCommander::GetPositionCmd() {
  return _pos->GetVal();
}

