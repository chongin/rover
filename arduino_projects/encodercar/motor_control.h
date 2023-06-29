#pragma once

class MotorControl
{
public:
  MotorControl();

  void SetLeftSpeed(int speed);

  void SetRightSpeed(int speed);

  void Forward();
 
  void Backward();
  
  void TurnOffAll();
  
  void ForwardLeft();

  void ForwardRight();

  void BackwardLeft();

  void BackwardRight();

  void TurnOffLeft();

  void TurnOffRight();
  
  void IncreaseLeftSpeed();

  void DecreaseLeftSpeed();

  void IncreaseRightSpeed();
  
  void DecreaseRightSpeed();
 
  
private:
  void InitMotor();

  int _left_speed;
  int _right_speed;
};
