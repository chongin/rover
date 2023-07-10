#include "motor_control.h"

const int DEFAULT_SPEED = 150;
const int MIN_SPEED = 50;
MotorControl::MotorControl()
{
  InitMotor();
  //ResetSpeed();
  SetLeftSpeed(DEFAULT_SPEED);
  SetRightSpeed(DEFAULT_SPEED + 20);
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
  if (speed > MIN_SPEED && speed < 256)
  {
    _left_speed = speed;
    analogWrite(enB, speed);

//    Serial.print("Set Left speed:");
//    Serial.println(_left_speed);
  }
    
}

void MotorControl::SetRightSpeed(int speed)
{
  if (speed > MIN_SPEED && speed < 256)
  {
    _right_speed = speed;
    analogWrite(enA, speed);

//    Serial.print("Set Right speed:");
//    Serial.println(_right_speed);
  }
}

void MotorControl::Forward()
{
  ForwardLeft();
  ForwardRight();

//  Serial.println("Forward All");
}

void MotorControl::Backward()
{
  BackwardLeft();
  BackwardRight();

//  Serial.println("Backward all");
}

void MotorControl::TurnOffAll()
{
  TurnOffLeft();
  TurnOffRight();
//  Serial.println("Turn Off All");
}

void MotorControl::ForwardLeft()
{
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  
//  Serial.println("Forward Left");
}

void MotorControl::BackwardLeft()
{
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
  Serial.println("Backward Left");
}

void MotorControl::ForwardRight()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
 
//  Serial.println("Forward Right");
}

void MotorControl::BackwardRight()
{ 
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  Serial.println("Backward Right");
}

void MotorControl::TurnOffLeft()
{
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

//  Serial.println("Turn Off Left");
}

void MotorControl::TurnOffRight()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

//  Serial.println("Turn Off Right");
}

void MotorControl::IncreaseLeftSpeed()
{
  if (_left_speed < 255)
  {
    _left_speed += 1;
    SetLeftSpeed(_left_speed);
  } 
}

void MotorControl::DecreaseLeftSpeed()
{
  if (_left_speed > 0)
  {
     _left_speed -= 1;
     SetLeftSpeed(_left_speed);
  }
}

void MotorControl::IncreaseRightSpeed()
{
  if (_right_speed < 255)
  {
    _right_speed += 1;
    SetRightSpeed(_right_speed);
  }
}

void MotorControl::DecreaseRightSpeed()
{
  if (_right_speed > 0)
  {
    _right_speed -= 1;
    SetRightSpeed(_right_speed);
  }
}
