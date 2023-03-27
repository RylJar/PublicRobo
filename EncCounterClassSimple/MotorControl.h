#ifndef MotorControl_h
#define MotorControl_h
#include <Arduino.h>


class MotorControl{
  private:
    int g_pin_enable;
    int g_pin_dir;

  public:
    MotorControl(int pin_enable, int pin_dir);
    int run_moto(int direction, int value);
};

#endif