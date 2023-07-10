#pragma once
#include <Servo.h>
#include <Arduino.h>

#define servoPin 10

const int DEFAULT_ANGLE = 90;
const int ANGLE_STEP = 2;
const int MIN_ANGLE = 56;
const int MAX_ANGLE = 124;
class ServoControl
{
public:
  ServoControl()
  {
     myservo.attach(servoPin);
     _angle = DEFAULT_ANGLE;
     myservo.write(_angle);
  }

  void SetServoAngle(int angle)
  {
    if (angle >= MIN_ANGLE && angle <= MAX_ANGLE)
    {
      _angle = angle;
      myservo.write(angle);
//
//      Serial.print("Angle:");
//      Serial.println(_angle);
    }
  }

  void ResetAngle()
  {
     _angle = DEFAULT_ANGLE;
     myservo.write(_angle);
  }
  
  int GetAngle()
  {
    return _angle;
  }

  void TurnLeft()
  {
    SetServoAngle(MIN_ANGLE);
  }

  void TurnRight()
  {
    SetServoAngle(MAX_ANGLE);
  }
  
  void IncreaseAngle()
  {
    if (_angle < MAX_ANGLE)
    {
      _angle += ANGLE_STEP;
      SetServoAngle(_angle);
    }
  }

  void DecreaseAngle()
  {
    if (_angle > MIN_ANGLE) 
    {
       _angle -= ANGLE_STEP;
       SetServoAngle(_angle);
    }
  }
  
private:
  Servo myservo;
  int _angle = 0;
};
