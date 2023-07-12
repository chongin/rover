#pragma once

#include <ros.h>
#include <std_msgs/String.h>
#include "motor_control.h"
#include "servo_control.h"

class GlobalObjects
{
public:
  GlobalObjects()
  {
    nh = new ros::NodeHandle();
    nh->getHardware()->setBaud(115200);
    nh->initNode();

    servo_control = new ServoControl();
    motor_control = new MotorControl();
  }

  
public:
  ros::NodeHandle* nh;
  MotorControl* motor_control;
  ServoControl* servo_control;
};
