#pragma once
#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

class CmdVelControl
{
public:
  static CmdVelControl* GetInstance()
  {
      if (_instance != nullptr)
      {
        _instance = new CmdVelControl();
      }

      return _instance;
  }

  
private:
  CmdVelControl()
  {
    subCmdVel = new ros::Subscriber<geometry_msgs::Twist>("cmd_vel", &calc_pwm_values );
  }

  
private:
  static CmdVelControl* _instance;
  ros::Subscriber<geometry_msgs::Twist>* subCmdVel; 
};


CmdVelControl* CmdVelControl::_instance = nullptr;
