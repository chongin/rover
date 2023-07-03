#pragma once
#include <Servo.h>
#include <Arduino.h>

#define servoPin 10

const int DEFAULT_ANGLE = 100;
const int ANGLE_STEP = 2;
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
    if (angle >= 0 && angle <= 180)
    {
      _angle = angle;
      myservo.write(angle);

      Serial.print("Angle:");
      Serial.println(_angle);
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
    SetServoAngle(0);
  }

  void TurnRight()
  {
    SetServoAngle(180);
  }
  
  void IncreaseAngle()
  {
    if (_angle < 180)
    {
      _angle += ANGLE_STEP;
      SetServoAngle(_angle);
    }
  }

  void DecreaseAngle()
  {
    if (_angle > 0) 
    {
       _angle -= ANGLE_STEP;
       SetServoAngle(_angle);
    }
  }
  
private:
  Servo myservo;
  int _angle = 0;
};
