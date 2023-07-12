#include "globle_objects.h"
#include "motor_control.h"
#include "servo_control.h"
#include "encoder.h"
#include "test_car.h"
#include "manual_drive_handler.h"
#include "cmd_vel_control.h"


GlobalObjects* globle_objects = nullptr;
 
void setup() 
{
  Serial.begin(115200);
  globle_objects = new GlobalObjects();
  //ManualDriveHandler::GetInstance()->SetData(globle_objects->nh, globle_objects->motor_control, globle_objects->servo_control);
  Encoder::GetInstance()->setup_encoder(globle_objects->nh);
  
  CmdVelControl::GetInstance()->SetData(globle_objects->nh, globle_objects->motor_control, globle_objects->servo_control);
}

void loop() 
{
  Encoder::GetInstance()->publish_encoder_ticker();
  CmdVelControl::GetInstance()->SetPwmValues();
  globle_objects->nh->spinOnce();
  
  
  
 // test_car(globle_objects->moto_control);
// test_servo(globle_objects->servo_control);
  delay(50);
}
