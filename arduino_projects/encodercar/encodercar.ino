#include "globle_objects.h"
#include "motor_control.h"
#include "servo_control.h"
#include "encoder.h"
#include "test_car.h"
#include "keyboard_handler.h"


GlobalObjects* globle_objects = nullptr;

void setup() 
{
  Serial.begin(115200);
  globle_objects = new GlobalObjects();
  KeyboardHandler::GetInstance()->SetData(globle_objects->nh, globle_objects->moto_control, globle_objects->servo_control);
  Encoder::GetInstance()->setup_encoder(globle_objects->nh);
}

void loop() 
{
  Encoder::GetInstance()->publish_encoder_ticker();
  globle_objects->nh->spinOnce();
  
  
// test_car(globle_objects->moto_control);
// test_servo(globle_objects->servo_control);
  delay(50);
}
