#pragma once
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

class ManualDriveHandler
{
public:
  static ManualDriveHandler* GetInstance()
  {
    if (_instance == nullptr)
    {
       _instance = new ManualDriveHandler();
    }
    return _instance;
  }
  
  static void HandleControlMessageCb( const std_msgs::String& ctrlMsg)
  {
    ManualDriveHandler::GetInstance()->ControlMessageCb(ctrlMsg);
  }
 
  void ControlMessageCb( const std_msgs::String& ctrlMsg)
  {
    const String& data = ctrlMsg.data;
    if (data[0] == 'W') {
      _moto_control->Forward();
    } 
    else if (data[0] == 'S') {
      _moto_control->Backward();
    } 
    else if (data[0] == 'A') {
      _servo_control->DecreaseAngle();
    } 
    else if (data[0] == 'D') {
      _servo_control->IncreaseAngle();
    }
    else if (data[0] == 'T') {
      _moto_control->TurnOffAll();
    }
    else if (data[0] == 'Q')
    {
      _moto_control->IncreaseSpeed();
    }
    else if (data[0] == 'E')
    {
      _moto_control->DecreaseSpeed();
    }
    else if (data[0] == 'R')
    {
      _moto_control->ResetSpeed();
      _servo_control->ResetAngle();
    }
    else{
      Serial.print("Didn't recognize:");
      Serial.println(data);
      return;
    }
  
    PublishMessage(data, _moto_control->GetMotoInfo(), 
      _servo_control->GetAngle());
  }
  
  void SetData(ros::NodeHandle* nh, MotorControl* moto_control, ServoControl* servo_control)
  {
    _nh = nh;
    _moto_control = moto_control;
    _servo_control = servo_control;

    _nh->advertise(*_drive_info_pub);
    _nh->subscribe(*_drive_car_sub);
  }

  void PublishMessage(String command, MotoInfo motoInfo, int angle)
  {
     char myString[50];
     snprintf(myString, sizeof(myString), "%s,%d,%d,%d", command.c_str(), motoInfo.left_speed, motoInfo.right_speed, angle);
     _drive_info_msg->data = myString;
     _drive_info_pub->publish(_drive_info_msg);
  }
  
private:
  ManualDriveHandler()
  {
    _nh = nullptr;
    _moto_control = nullptr;
    _servo_control = nullptr;

    _drive_info_msg = new std_msgs::String();
    _drive_info_pub = new ros::Publisher("drive_info", _drive_info_msg);
    _drive_car_sub = new ros::Subscriber<std_msgs::String>("drive_car", ManualDriveHandler::HandleControlMessageCb);
  }

private:
  static ManualDriveHandler* _instance;

  ros::NodeHandle* _nh;
  MotorControl* _moto_control;
  ServoControl* _servo_control;

  std_msgs::String* _drive_info_msg;
  ros::Publisher* _drive_info_pub;

  ros::Subscriber<std_msgs::String>* _drive_car_sub;
};

ManualDriveHandler* ManualDriveHandler::_instance = nullptr;
