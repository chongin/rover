#pragma once
#include <Servo.h>

#define servoPin 5

class ServoControl
{
public:
  ServoControl()
  {
     myservo.attach(servoPin);
     _angle = 0;
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
      _angle += 1;
      SetServoAngle(_angle);
    }
  }

  void DecreaseAngle()
  {
    if (_angle > 0) 
    {
       _angle -= 1;
       SetServoAngle(_angle);
    }
  }
  
private:
  Servo myservo;
  int _angle = 0;
};
