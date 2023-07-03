#include <ros.h>
#include <std_msgs/String.h>
#include "motor_control.h"
#include "servo_control.h"



ros::NodeHandle* nh;
MotorControl* moto_control;
ServoControl* servo_control;

bool sendflag = false;
std_msgs::String carInfoMsg;

ros::Publisher pub("car_info", &carInfoMsg);

const int MAX_STRINGS = 2; // Maximum number of strings to store
String strings[MAX_STRINGS]; // Array to store strings
int numStrings = 0; // Number of strings currently stored

void AddMessage(String* command, MotoInfo motoInfo, int angle)
{
   String dd = *command + "," + String(motoInfo.left_speed) + "," + String(motoInfo.right_speed) + "," + String(servo_control->GetAngle());
   carInfoMsg.data = dd.c_str();
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
  else{
    Serial.print("Didn't recognize this command:");
    Serial.println(data);
  }

  AddMessage(&data, moto_control->GetMotoInfo(), servo_control->GetAngle());
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
}

void loop() 
{
  nh->spinOnce();
 
//  std_msgs::String ctrlMsg;
//  ctrlMsg.data = "up";
//  controlMessageCb(ctrlMsg);
//  delay(2000);
//  ctrlMsg.data = "down";
//  controlMessageCb(ctrlMsg);
//  delay(2000);
  //test_car();
  //test_servo();
  delay(50);
}

void test_car()
{
//  for (int i = 0; i < 256; i++ )
//  {
//    moto_control->SetLeftSpeed(i);
//    moto_control->ForwardLeft();
//    delay(30);
//  }
//  
//  moto_control->TurnOffLeft();
//  delay(2000);
//
//  for (int i = 0; i < 256; i++ )
//  {
//    moto_control->SetLeftSpeed(i);
//    moto_control->BackwardLeft();
//    delay(30);
//  }
//  
//  moto_control->TurnOffLeft();
//  delay(2000);

//  for (int i = 0; i < 256; i++ )
//  {
//    moto_control->SetRightSpeed(i);
//    moto_control->BackwardRight();
//    delay(30);
//  }
  
  //moto_control->SetRightSpeed(255);
  //moto_control->BackwardRight();
  //delay(2000);
//  moto_control->TurnOffRight();
//  delay(1000);
//
  moto_control->Forward();
  delay(2000);
  moto_control->TurnOffAll();
  delay(1000);
  moto_control->Backward();
  delay(2000);
  moto_control->TurnOffAll();
  delay(1000);
}

void test_servo()
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

void addString(String newString) {
  if (numStrings < MAX_STRINGS) {
    strings[numStrings] = newString;
    numStrings++;
  }
}
