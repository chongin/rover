#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motor_1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor_2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor_3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor_4 = AFMS.getMotor(4);

int currentSpeed = 40;


void setMotorsSpeed(int sp) {
  motor_1->setSpeed(sp);
  motor_2->setSpeed(sp);
  motor_3->setSpeed(sp);
  motor_4->setSpeed(sp);
}

void increaseMotorSpeed()
{
  currentSpeed += 10;
  if (currentSpeed > 250)
  {
    currentSpeed = 250;
  }
  setMotorsSpeed(currentSpeed);
}

void decreaseMotorSpeed()
{
  currentSpeed -= 10;
  if (currentSpeed < 0)
  {
    currentSpeed = 250;
  }
  
  setMotorsSpeed(currentSpeed);
}

void setMotorShield(){
   AFMS.begin();
  setMotorsSpeed(currentSpeed);
 
}

void forward() {
  motor_1->run(FORWARD);
  motor_2->run(FORWARD);
  motor_3->run(FORWARD);
  motor_4->run(FORWARD);
}

void backward() {
  
  motor_1->run(BACKWARD);
  motor_2->run(BACKWARD);
  motor_3->run(BACKWARD);
  motor_4->run(BACKWARD);
  
}

void stopMotors() {
  
  motor_1->run(RELEASE);
  motor_2->run(RELEASE);
  motor_3->run(RELEASE);
  motor_4->run(RELEASE);
  
  }

void turnLeft() {
  motor_1->run(FORWARD);
  motor_2->run(FORWARD);
  motor_3->run(BACKWARD);
  motor_4->run(BACKWARD);
}


void turnRight() {
  motor_1->run(BACKWARD);
  motor_2->run(BACKWARD);
  motor_3->run(FORWARD);
  motor_4->run(FORWARD);
  
}
