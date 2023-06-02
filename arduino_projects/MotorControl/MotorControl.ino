
#include <ros.h>
#include <std_msgs/String.h>

#include "MotorManager.h"

String carStatus = "stop";
std_msgs::String carStatusMsg;
ros::NodeHandle  nh;

void controlMessageCb( const std_msgs::String& ctrlMsg){
  String data = ctrlMsg.data;

  if (data == "up") {
    forward();
    setCarStatus(data);
  } 
  else if (data == "down") {
    backward();
    setCarStatus(data);
  } 
  else if (data == "left") {
    turnLeft();
    setCarStatus(data);
  } 
  else if (data == "right") {
    turnRight();
    setCarStatus(data);
  } 
  else if (data == "stop") {
    stopMotors();
    setCarStatus(data);
  }
  else if (data == "speed+")
  {
    increaseMotorSpeed();
  }
  else if (data == "speed-")
  {
    decreaseMotorSpeed();
  }
  else{
    Serial.print("Didn't recognize this command:");
    Serial.println(data);
  }
}

void setCarStatus(const String& status)
{
  carStatus = status;
  carStatusMsg.data = carStatus.c_str();
  Serial.println("update status" + carStatus);
}

ros::Subscriber<std_msgs::String> sub("drive_car", controlMessageCb);
ros::Publisher carInfoPub("car_info", &carStatusMsg);

void setup()
{
  setMotorShield();
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(carInfoPub);
  nh.subscribe(sub);
}

void loop()
{
  carInfoPub.publish( &carStatusMsg );
  nh.spinOnce();
  delay(500);
}

void testCar()
{
  forward();
  delay(2000);
  backward();
  delay(2000);
  turnLeft();
  delay(2000);
  turnRight();
  delay(2000);
  stopMotors();
  delay(2000);
}
