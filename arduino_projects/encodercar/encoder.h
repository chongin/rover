#pragma once
#include <ros.h>
#include <std_msgs/Int16.h>

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 4
#define ENC_IN_RIGHT_B 11


class Encoder
{
public:
  static Encoder* GetInstance()
  {
    if (_instance == nullptr)
    {
      _instance = new Encoder();
    }

    return _instance;
  }

  static void handle_right_wheel_tick()
  {
    GetInstance()->right_wheel_tick();
  }
  
  static void handle_left_wheel_tick()
  {
    GetInstance()->left_wheel_tick();
  }
  
  void right_wheel_tick() {
   
    // Read the value for the encoder for the right wheel
    int val = digitalRead(ENC_IN_RIGHT_B);
   
    if(val == LOW) {
      Direction_right = false; // Reverse
    }
    else {
      Direction_right = true; // Forward
    }
     
    if (Direction_right) {
       
      if (right_wheel_tick_count->data == encoder_maximum) {
        right_wheel_tick_count->data = encoder_minimum;
      }
      else {
        right_wheel_tick_count->data++;  
      }    
    }
    else {
      if (right_wheel_tick_count->data == encoder_minimum) {
        right_wheel_tick_count->data = encoder_maximum;
      }
      else {
        right_wheel_tick_count->data--;  
      }   
    }
  }

  void left_wheel_tick() {
   
    // Read the value for the encoder for the left wheel
    int val = digitalRead(ENC_IN_LEFT_B);
   
    if(val == LOW) {
      Direction_left = true; // Reverse
    }
    else {
      Direction_left = false; // Forward
    }
     
    if (Direction_left) {
      if (left_wheel_tick_count->data == encoder_maximum) {
        left_wheel_tick_count->data = encoder_minimum;
      }
      else {
        left_wheel_tick_count->data++;  
      }  
    }
    else {
      if (left_wheel_tick_count->data == encoder_minimum) {
        left_wheel_tick_count->data = encoder_maximum;
      }
      else {
        left_wheel_tick_count->data--;  
      }   
    }
  }

  void setup_encoder(ros::NodeHandle* nh)
  {
    // Set pin states of the encoder
    pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
    pinMode(ENC_IN_LEFT_B , INPUT);
    pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
    pinMode(ENC_IN_RIGHT_B , INPUT);
   
    // Every time the pin goes high, this is a tick
    attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), Encoder::handle_left_wheel_tick, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), Encoder::handle_right_wheel_tick, RISING);
  
    nh->advertise(*rightPub);
    nh->advertise(*leftPub);
  }
  
  void publish_encoder_ticker()
  {
    // Record the time
    currentMillis = millis();
   
    // If 100ms have passed, print the number of ticks
    if (currentMillis - previousMillis > interval) {
       
      previousMillis = currentMillis;
       
      rightPub->publish( right_wheel_tick_count);
      leftPub->publish( left_wheel_tick_count);

//      Serial.print("Right:");
//      Serial.print(right_wheel_tick_count->data);
//      Serial.print(",Left:");
//      Serial.println(left_wheel_tick_count->data);
    }
  }

  std_msgs::Int16* GetLeftWheelTick()
  {
    return left_wheel_tick_count;
  }

  std_msgs::Int16* GetRightWheelTick()
  {
    return right_wheel_tick_count;
  }
private:
  Encoder()
  {
    right_wheel_tick_count = new std_msgs::Int16();
    rightPub = new ros::Publisher("right_ticks", right_wheel_tick_count);

    left_wheel_tick_count = new std_msgs::Int16();
    leftPub = new ros::Publisher("left_ticks", left_wheel_tick_count);
  }

private:
  static Encoder* _instance;

  // True = Forward; False = Reverse
  boolean Direction_left = true;
  boolean Direction_right = true;
   
  // Minumum and maximum values for 16-bit integers
  const int encoder_minimum = -32768;
  const int encoder_maximum = 32767;
   
  // Keep track of the number of wheel ticks
  std_msgs::Int16* right_wheel_tick_count;
  ros::Publisher* rightPub;
   
  std_msgs::Int16* left_wheel_tick_count;
  ros::Publisher* leftPub;
   
  // 100ms interval for measurements
  const int interval = 100;
  long previousMillis = 0;
  long currentMillis = 0;

};

Encoder* Encoder::_instance = nullptr;
