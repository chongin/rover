#pragma once
#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include "encoder.h"

class CmdVelControl
{
public:
  static CmdVelControl* GetInstance()
  {
      if (_instance != nullptr)
      {
        _instance = new CmdVelControl();
      }

      return _instance;
  }

  static void HandleCalcPwmValues(const geometry_msgs::Twist& cmdVel)
  {
    GetInstance()->CalcPwmValues(cmdVel);
  }

  void SetData(MotorControl* moto, ServoControl* servo)
  {
    _moto = moto;
    _servo = servo;
  }

  void CalcVelLefWheel()
  {
    // Previous timestamp
    static double prevTime = 0;
    
    // Variable gets created and initialized the first time a function is called.
    static int prevLeftCount = 0;
  
    // Manage rollover and rollunder when we get outside the 16-bit integer range 
    int numOfTicks = (65535 + Encoder::GetInstance()->GetLeftWheelTick()->data - prevLeftCount) % 65535;
  
    // If we have had a big jump, it means the tick count has rolled over.
    if (numOfTicks > 10000) {
          numOfTicks = 0 - (65535 - numOfTicks);
    }
  
    // Calculate wheel velocity in meters per second
    velLeftWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
  
    // Keep track of the previous tick count
    prevLeftCount = Encoder::GetInstance()->GetLeftWheelTick()->data;
  
    // Update the timestamp
    prevTime = (millis()/1000);
  
  }
  // Calculate the right wheel linear velocity in m/s every time a 
  // tick count message is published on the /right_ticks topic. 
  void CalcVelRightWheel(){
    
    // Previous timestamp
    static double prevTime = 0;
    
    // Variable gets created and initialized the first time a function is called.
    static int prevRightCount = 0;
  
    // Manage rollover and rollunder when we get outside the 16-bit integer range 
    int numOfTicks = (65535 + Encoder::GetInstance()->GetRightWheelTick()->data - prevRightCount) % 65535;
  
    if (numOfTicks > 10000) {
          numOfTicks = 0 - (65535 - numOfTicks);
    }
  
    // Calculate wheel velocity in meters per second
    velRightWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
  
    prevRightCount = Encoder::GetInstance()->GetRightWheelTick()->data;
    
    prevTime = (millis()/1000);
  
  }
  
  // Take the velocity command as input and calculate the PWM values.
  void CalcPwmValues(const geometry_msgs::Twist& cmdVel) {
    
    // Record timestamp of last velocity command received
    lastCmdVelReceived = (millis()/1000);
    
    // Calculate the PWM value given the desired velocity 
    pwmLeftReq = K_P * cmdVel.linear.x + b;
    pwmRightReq = K_P * cmdVel.linear.x + b;
  
    // Check if we need to turn 
    if (cmdVel.angular.z != 0.0) {
  
      // Turn left
      if (cmdVel.angular.z > 0.0) {
        pwmLeftReq = -PWM_TURN;
        pwmRightReq = PWM_TURN;
      }
      // Turn right    
      else {
        pwmLeftReq = PWM_TURN;
        pwmRightReq = -PWM_TURN;
      }
    }
    // Go straight
    else {
      
      // Remove any differences in wheel velocities 
      // to make sure the robot goes straight
      static double prevDiff = 0;
      static double prevPrevDiff = 0;
      double currDifference = velLeftWheel - velRightWheel; 
      double avgDifference = (prevDiff+prevPrevDiff+currDifference)/3;
      prevPrevDiff = prevDiff;
      prevDiff = currDifference;
  
      // Correct PWM values of both wheels to make the vehicle go straight
      pwmLeftReq -= (int)(avgDifference * DRIFT_MULTIPLIER);
      pwmRightReq += (int)(avgDifference * DRIFT_MULTIPLIER);
    }
  
    // Handle low PWM values
    if (abs(pwmLeftReq) < PWM_MIN) {
      pwmLeftReq = 0;
    }
    if (abs(pwmRightReq) < PWM_MIN) {
      pwmRightReq = 0;  
    }  
  }
  
  void set_pwm_values() 
  {
  }

private:
  CmdVelControl()
  {
    subCmdVel = new ros::Subscriber<geometry_msgs::Twist>("cmd_vel", &CmdVelControl::HandleCalcPwmValues);
  }

private:
  static CmdVelControl* _instance;
  MotorControl* _moto; 
  ServoControl* _servo;
  ros::Subscriber<geometry_msgs::Twist>* subCmdVel;


  // How much the PWM value can change each cycle
  const int PWM_INCREMENT = 1;
  
  // Number of ticks per wheel revolution. We won't use this in this code.
  const int TICKS_PER_REVOLUTION = 620;
  
  // Wheel radius in meters
  const double WHEEL_RADIUS = 0.033;
  
  // Distance from center of the left tire to the center of the right tire in m
  const double WHEEL_BASE = 0.17;
  
  // Number of ticks a wheel makes moving a linear distance of 1 meter
  // This value was measured manually.
  const double TICKS_PER_METER = 3100; // Originally 2880
  
  // Proportional constant, which was measured by measuring the 
  // PWM-Linear Velocity relationship for the robot.
  const int K_P = 278;
  
  // Y-intercept for the PWM-Linear Velocity relationship for the robot
  const int b = 52;
  
  // Correction multiplier for drift. Chosen through experimentation.
  const int DRIFT_MULTIPLIER = 120;
  
  // Turning PWM output (0 = min, 255 = max for PWM values)
  const int PWM_TURN = 80;
  
  // Set maximum and minimum limits for the PWM values
  const int PWM_MIN = 80; // about 0.1 m/s
  const int PWM_MAX = 100; // about 0.172 m/s
  
  // Set linear velocity and PWM variable values for each wheel
  double velLeftWheel = 0;
  double velRightWheel = 0;
  double pwmLeftReq = 0;
  double pwmRightReq = 0;
  
  // Record the time that the last velocity command was received
  double lastCmdVelReceived = 0;
};


CmdVelControl* CmdVelControl::_instance = nullptr;
