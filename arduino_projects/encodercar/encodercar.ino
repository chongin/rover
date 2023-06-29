#include <ros.h>
#include <std_msgs/String.h>
#include "motor_control.h"
#include "servo_control.h"


std_msgs::String carStatusMsg;
ros::NodeHandle* nh;
MotorControl* moto_control;
ServoControl* servo_control;

void controlMessageCb( const std_msgs::String& ctrlMsg){
  String data = ctrlMsg.data;

  if (data == "up") {
    moto_control->Forward();
  } 
  else if (data == "down") {
    moto_control->Backward();
  } 
  else if (data == "left") {
    servo_control->DecreaseAngle();
  } 
  else if (data == "right") {
    servo_control->IncreaseAngle();
  }
  else if (data == "stop") {
    moto_control->TurnOffAll();
  }
  else if (data == "speed+")
  {
    moto_control->IncreaseLeftSpeed();
    moto_control->IncreaseRightSpeed();
  }
  else if (data == "speed-")
  {
    moto_control->DecreaseLeftSpeed();
    moto_control->DecreaseRightSpeed();
  }
  else{
    Serial.print("Didn't recognize this command:");
    Serial.println(data);
  }
}

ros::Subscriber<std_msgs::String> sub("drive_car", controlMessageCb);


void setup() 
{
  Serial.begin(115200);
  nh = new ros::NodeHandle();
  nh->getHardware()->setBaud(115200);
  nh->initNode();
  nh->subscribe(sub);

  servo_control = new ServoControl();
  moto_control = new MotorControl();
}

void loop() 
{
  //nh->spinOnce();
  test_card();
  //test_servo();
  delay(500);
}

void test_card()
{
  //moto_control->ForwardLeft();
  //delay(2000);
  //moto_control->TurnOffLeft();
  //delay(1000);
  moto_control->ForwardRight();
  delay(2000);
  moto_control->TurnOffRight();
  delay(1000);
//
//  moto_control->Forward();
//  delay(2000);
//  moto_control->TurnOffAll();
//  delay(1000);
//  moto_control->Backward();
//  delay(2000);
//  moto_control->TurnOffAll();
//  delay(1000);
}

void test_servo()
{
  servo_control->SetServoAngle(0);
  delay(1000);
  for (int i = 0; i < 180; ++i)
  {
    servo_control->IncreaseAngle();
    delay(30);
  }
}
