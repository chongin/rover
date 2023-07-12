#pragma once
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

class CmdVelControl
{
public:
  static CmdVelControl* GetInstance()
  {
    if (_instance == nullptr)
    {
       _instance = new CmdVelControl();
    }
    return _instance;
  }

  static void HandleCalcPwmValues(const geometry_msgs::Twist& cmdVel)
  {
    GetInstance()->CalcPwmValues(cmdVel);
  }
  
  void SetData(ros::NodeHandle* nh, MotorControl* moto_control, ServoControl* servo_control)
  {
    _nh = nh;
    _motor = moto_control;
    _servo = servo_control;


    _nh->subscribe(*subCmdVel);
  }

   void DriveStraight()
  {
//    int offset = 5;
//    int leftCount = Encoder::GetInstance()->GetLeftWheelTick()->data;
//    int rightCount = Encoder::GetInstance()->GetRightWheelTick()->data;
//
//    long leftDiff = leftCount - prevLeftCount;
//    long rightDiff = rightCount - prevRightCount;
//
//    prevLeftCount = leftCount;
//    prevRightCount = rightCount;
//
//    int rightPower = _motor->GetRightSpeed();
//    if (leftDiff > rightDiff)
//    {
//      rightPower += offset;
//    }
//    else if (leftDiff < rightDiff)
//    {
//      rightPower -= offset;
//    }
//
//    _motor->SetRightSpeed(rightPower);

    int offset = 2;
    int leftCount = Encoder::GetInstance()->GetLeftWheelTick()->data;
    int rightCount = Encoder::GetInstance()->GetRightWheelTick()->data;
    int rightPower = _motor->GetRightSpeed();
    if (leftCount > rightCount)
    {
      rightPower += offset;
    }
    else if (leftCount < rightCount)
    {
      rightPower -= offset;
    }

    _motor->SetRightSpeed(rightPower);
  }

  // Take the velocity command as input and calculate the PWM values.
  void CalcPwmValues(const geometry_msgs::Twist& cmdVel) 
  {
    _ros_speed = CalcSpeedByLinear(cmdVel.linear.x);
    _ros_angle = CaclAngleByLinear(cmdVel.angular.z);

    Serial.print("ros_speed:");
    Serial.print(_ros_speed);
    Serial.print("ros_angle:");
    Serial.println(_ros_angle);
    if (cmdVel.angular.z != 0.0) {
      _go_straight = false;
    }
    // Go straight
    else {
      _go_straight = true;
    }
  }
  
  void SetPwmValues() 
  {
    _servo->SetServoAngle(_ros_angle);
    if (_ros_speed == 0)
    {
      _motor->SetSpeed(0);
      _motor->TurnOffAll();
    }
    else {
      _motor->SetLeftSpeed(_ros_speed);
      if (_motor->GetRightSpeed() == 0)
      {
        _motor->SetRightSpeed(_ros_speed);
      }

      if (_forward)
      {
        _motor->Forward();
      }
      else {
        _motor->Backward();
      }
      
      if (_go_straight)
      {
        DriveStraight();
      }
    }
  }

  int CalcSpeedByLinear(float linear)
  {
    //float step = 0.01;
    float max_linear_value = 0.22;
    int current_speed = 0;
    if (abs(linear) > 0) //linear >0 forward, <0 backward
    {
      _forward = linear > 0;
      float percentage = abs(linear) / max_linear_value;
      current_speed = _motor->GetMaxSpeed() * percentage;
    }
    else {
      current_speed = 0;
    }
    return current_speed;
  }

  int CaclAngleByLinear(float linear)
  {
    //float step = 0.1;
    float max_linear_value = 2.28;
    float percentage = abs(linear) / max_linear_value;

    int current_angle = 0;
    int center_angle = 90;
    if (linear > 0) //turn left
    {
      current_angle = _servo->GetMinAngle() + (center_angle - _servo->GetMinAngle()) * percentage;
    }
    else if (linear < 0)//turn right
    {
      current_angle = _servo->GetMaxAngle() - (_servo->GetMaxAngle() - center_angle) * percentage;
    }
    else {
      current_angle = center_angle;
    }
    return current_angle;
  }
private:
  CmdVelControl()
  {
    _nh = nullptr;
    _motor = nullptr;
    _servo = nullptr;

    subCmdVel = new ros::Subscriber<geometry_msgs::Twist>("cmd_vel", CmdVelControl::HandleCalcPwmValues);
  }

private:
  static CmdVelControl* _instance;

  ros::NodeHandle* _nh;
  MotorControl* _motor;
  ServoControl* _servo;

  ros::Subscriber<geometry_msgs::Twist>* subCmdVel;

  int prevLeftCount = 0;
  int prevRightCount = 0;

  int _ros_speed = 0;
  int _ros_angle = 0;
  bool _go_straight = false;
  bool _forward = true;
};

CmdVelControl* CmdVelControl::_instance = nullptr;
