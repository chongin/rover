#include "motor_control.h"

//const int DEFAULT_SPEED = 120;
const int DEFAULT_SPEED = 0;
const int MIN_SPEED = 0;
const int MAX_SPEED = 255;
const int MOTO_INCREAMENT = 1;
MotorControl::MotorControl()
{
  _left_speed = 0;
  _right_speed = 0;
  InitMotor();
  ResetSpeed();
//  SetLeftSpeed(DEFAULT_SPEED);
//  SetRightSpeed(DEFAULT_SPEED + 20);
}

void MotorControl::ResetSpeed()
{
  SetSpeed(DEFAULT_SPEED);
}

MotoInfo MotorControl::GetMotoInfo()
{
  MotoInfo info;
  info.left_speed = _left_speed;
  info.right_speed = _right_speed;
  return info;
}

void MotorControl::InitMotor()
{
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

int MotorControl::GetMinSpeed()
{
  return MIN_SPEED;
}

int MotorControl::GetMaxSpeed()
{
  return MAX_SPEED;
}

void MotorControl::SetSpeed(int speed)
{
  SetLeftSpeed(speed);
  SetRightSpeed(speed);
}

void MotorControl::IncreaseSpeed()
{
  IncreaseLeftSpeed();
  IncreaseRightSpeed();
}
  
void MotorControl::DecreaseSpeed()
{
  DecreaseLeftSpeed();
  DecreaseRightSpeed();
}
  
void MotorControl::SetLeftSpeed(int speed)
{
  if (speed >= MIN_SPEED && speed <= MAX_SPEED)
  {
    _left_speed = speed;
    analogWrite(enB, speed);
  }
}

void MotorControl::SetRightSpeed(int speed)
{
  if (speed >= MIN_SPEED && speed <= MAX_SPEED)
  {
    _right_speed = speed;
    analogWrite(enA, speed);
  }
}

void MotorControl::Forward()
{
  ForwardLeft();
  ForwardRight();
}

void MotorControl::Backward()
{
  BackwardLeft();
  BackwardRight();
}

void MotorControl::TurnOffAll()
{
  TurnOffLeft();
  TurnOffRight();
}

void MotorControl::ForwardLeft()
{
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void MotorControl::BackwardLeft()
{
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void MotorControl::ForwardRight()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void MotorControl::BackwardRight()
{ 
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void MotorControl::TurnOffLeft()
{
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void MotorControl::TurnOffRight()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void MotorControl::IncreaseLeftSpeed()
{
  if (_left_speed < MAX_SPEED)
  {
    _left_speed += MOTO_INCREAMENT;
    SetLeftSpeed(_left_speed);
  } 
}

void MotorControl::DecreaseLeftSpeed()
{
  if (_left_speed > MIN_SPEED)
  {
     _left_speed -= MOTO_INCREAMENT;
     SetLeftSpeed(_left_speed);
  }
}

void MotorControl::IncreaseRightSpeed()
{
  if (_right_speed < MAX_SPEED)
  {
    _right_speed += MOTO_INCREAMENT;
    SetRightSpeed(_right_speed);
  }
}

void MotorControl::DecreaseRightSpeed()
{
  if (_right_speed > MIN_SPEED)
  {
    _right_speed -= MOTO_INCREAMENT;
    SetRightSpeed(_right_speed);
  }
}
