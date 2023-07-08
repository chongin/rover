#pragma once

void test_car(MotorControl* moto_control)
{
  for (int i = 0; i < 256; i++ )
  {
    moto_control->SetLeftSpeed(i);
    moto_control->ForwardLeft();
    delay(30);
  }
  
  moto_control->TurnOffLeft();
  delay(2000);

  for (int i = 0; i < 256; i++ )
  {
    moto_control->SetLeftSpeed(i);
    moto_control->ForwardLeft();
    Serial.println(moto_control->GetMotoInfo().left_speed);
    delay(30);
  }

  delay(2000);
  moto_control->TurnOffLeft();
  delay(2000);

  for (int i = 0; i < 256; i++ )
  {
    moto_control->SetRightSpeed(i);
    moto_control->ForwardRight();
    Serial.println(moto_control->GetMotoInfo().right_speed);
    delay(30);
  }

   delay(2000);
   moto_control->TurnOffRight();
   delay(2000);
   
  //moto_control->SetRightSpeed(255);
  //moto_control->BackwardRight();
  //delay(2000);
  moto_control->TurnOffRight();
  delay(1000);

  moto_control->Forward();
  delay(2000);
  moto_control->TurnOffAll();
  delay(1000);
  moto_control->Backward();
  delay(2000);
  moto_control->TurnOffAll();
  delay(1000);
}

void test_servo(ServoControl* servo_control)
{
  //servo_control->SetServoAngle(0);
  
  for (int i = 0; i < 180; ++i)
  {
    servo_control->IncreaseAngle();
    delay(30);
  }
  Serial.println("Increase completed");
  delay(1000);
  for (int i = 0; i < 180; ++i)
  {
    servo_control->DecreaseAngle();
    delay(30);
  }
  Serial.println("Decrease completed.");
  delay(1000);
}
