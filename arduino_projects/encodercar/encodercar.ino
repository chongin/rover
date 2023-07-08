#include <ros.h>
#include <std_msgs/String.h>
#include "motor_control.h"
#include "servo_control.h"
//#include "encoder.h"
#include "test_car.h"


ros::NodeHandle* nh;
MotorControl* moto_control;
ServoControl* servo_control;

std_msgs::String carInfoMsg;
ros::Publisher pub("car_info", &carInfoMsg);


void PublishMessage(String command, MotoInfo motoInfo, int angle)
{
   char myString[50];
   snprintf(myString, sizeof(myString), "%s,%d,%d,%d", command.c_str(), motoInfo.left_speed, motoInfo.right_speed, servo_control->GetAngle());
   carInfoMsg.data = myString;
   pub.publish(&carInfoMsg);
}

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
    moto_control->IncreaseSpeed();
  }
  else if (data == "speed-")
  {
    moto_control->DecreaseSpeed();
  }
  else if (data == "reset")
  {
    moto_control->ResetSpeed();
    servo_control->ResetAngle();
    
  }
  else{
    Serial.print("Didn't recognize this command:");
    Serial.println(data);
  }

  PublishMessage(data, moto_control->GetMotoInfo(), servo_control->GetAngle());
}

ros::Subscriber<std_msgs::String> sub("drive_car", controlMessageCb);

void setup() 
{
  Serial.begin(115200);
  nh = new ros::NodeHandle();
  nh->getHardware()->setBaud(115200);
  nh->initNode();
  nh->advertise(pub);
  nh->subscribe(sub);

  
  servo_control = new ServoControl();
  moto_control = new MotorControl();

 // setup_encoder(nh);
}

void loop() 
{
 // publish_encoder_ticker();
  nh->spinOnce();
  
  
// test_car(moto_control);
// test_servo(servo_control);
  delay(50);
}
