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

    _nh->advertise(*_car_info_pub);
    _nh->subscribe(*_sub_cmd_vel);
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
    int left_count = Encoder::GetInstance()->GetLeftWheelTick()->data;
    int right_count = Encoder::GetInstance()->GetRightWheelTick()->data;
    int left_speed = _motor->GetLeftSpeed();
    if (right_count > left_count)
    {
      if (left_speed - _motor->GetRightSpeed() < 30)
      {
        left_speed += offset;
      }
      
    }
    else if (right_count < left_count)
    {
      if (_motor->GetRightSpeed() - left_speed < 30)
      {
        left_speed -= offset;
      }
    }
    else {
      left_speed = _motor->GetRightSpeed();
    }


    _motor->SetLeftSpeed(left_speed);
  }

  // Take the velocity command as input and calculate the PWM values.
  void CalcPwmValues(const geometry_msgs::Twist& cmdVel) 
  {
    _ros_speed = CalcSpeedByLinear(cmdVel.linear.x);
    _ros_angle = CaclAngleByLinear(cmdVel.angular.z);

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
      _motor->SetRightSpeed(_ros_speed); //right side is slower than left side
      if (_motor->GetLeftSpeed() == 0)
      {
        _motor->SetLeftSpeed(_ros_speed);
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

  void PublishCarInfo()
  {
     // Record the time
    currentMillis = millis();
   
    // If 100ms have passed, print the number of ticks
    if (currentMillis - previousMillis > interval) {
       
      previousMillis = currentMillis;
       
       char myString[50];
       snprintf(myString, sizeof(myString), "%d,%d,%d,%d,%d", 
        Encoder::GetInstance()->GetLeftWheelTick()->data,
        Encoder::GetInstance()->GetRightWheelTick()->data,
        _motor->GetLeftSpeed(), _motor->GetRightSpeed(), _servo->GetAngle()
       );
        
       _car_info_msg->data = myString;
       _car_info_pub->publish(_car_info_msg);
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
      current_angle = center_angle - (center_angle - _servo->GetMinAngle()) * percentage;
    }
    else if (linear < 0)//turn right
    {
      current_angle = center_angle + (_servo->GetMaxAngle() - center_angle) * percentage;
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

    _car_info_msg = new std_msgs::String();
    _sub_cmd_vel = new ros::Subscriber<geometry_msgs::Twist>("cmd_vel", CmdVelControl::HandleCalcPwmValues);
    _car_info_pub = new ros::Publisher("car_info", _car_info_msg);
  }

private:
  static CmdVelControl* _instance;

  ros::NodeHandle* _nh;
  MotorControl* _motor;
  ServoControl* _servo;

  ros::Subscriber<geometry_msgs::Twist>* _sub_cmd_vel;
   std_msgs::String* _car_info_msg;
  ros::Publisher* _car_info_pub;
  
  int prevLeftCount = 0;
  int prevRightCount = 0;

  int _ros_speed = 0;
  int _ros_angle = 0;
  bool _go_straight = false;
  bool _forward = true;

  // 100ms interval for measurements
  const int interval = 100;
  long previousMillis = 0;
  long currentMillis = 0;
};

CmdVelControl* CmdVelControl::_instance = nullptr;
