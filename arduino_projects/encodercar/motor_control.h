#pragma once
#include <Arduino.h>

struct MotoInfo
{
  int left_speed = 0;
  int right_speed = 0;
};

class MotorControl
{
public:
  MotorControl();

  void Forward();
 
  void Backward();
  
  void TurnOffAll();

  void SetSpeed(int speed);

  void IncreaseSpeed();
  
  void DecreaseSpeed();
  
  void SetLeftSpeed(int speed);
  void SetRightSpeed(int speed);
  
  void ForwardLeft();
  void ForwardRight();

  void BackwardLeft();
  void BackwardRight();

  void TurnOffLeft();
  void TurnOffRight();
  
  void IncreaseLeftSpeed();
  void IncreaseRightSpeed();
  
  void DecreaseLeftSpeed();
  void DecreaseRightSpeed();
 
  MotoInfo GetMotoInfo();
private:
  void InitMotor();

  int _left_speed;
  int _right_speed;
    // Motor A connections
  int enA = 6;
  int in1 = 12;
  int in2 = 13;
  // Motor B connections
  int enB = 5;
  int in3 = 8;
  int in4 = 7;
};
