#include "MotorControl.h"


MotorControl::MotorControl(int pin_enable, int pin_dir){
    g_pin_enable = pin_enable;
    g_pin_dir = pin_dir;
    pinMode(pin_enable, OUTPUT);
    pinMode(pin_dir, OUTPUT);
}

int MotorControl::run_moto(int direction, int value){
  // direction should be given by 0 or 1, value should be given between -255 and 255,
  digitalWrite(g_pin_dir, direction);
  analogWrite(g_pin_enable, abs(value));
  return abs(value);
}
