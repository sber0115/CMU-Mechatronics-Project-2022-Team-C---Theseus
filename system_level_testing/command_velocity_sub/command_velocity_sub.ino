#include "ros.h"
#include "geometry_msgs/Twist.h"
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

unsigned long currentMillis;
unsigned long previousMillis;

const uint8_t encoder1_pinA;
const uint8_t encoder1_pinB;

const uint8_t encoder2_pinA;
const uint8_t encoder2_pinB;

const uint8_t encoder3_pinA;
const uint8_t encoder3_pinB;

const uint8_t encoder4_pinA;
const uint8_t encoder4_pinB;

const float R = 0.09;
const float L1 = 0.1778;
const float L2 = 0.1778;

volatile long encoder1_pos = 0;    // encoder 1
volatile long encoder2_Pos = 0;    // encoder 2
volatile long encoder3_pos = 0;    // encoder 3
volatile long encoder4_pos = 0;    // encoder 4

float desired_x;
float desired_y;
float desired_th;


volatile float M1_setpoint; // front left  (M1)
volatile float M2_setpoint; // rear left   (M2)
volatile float M3_setpoint; // front right (M3)
volatile float M4_setpoint; // rear right  (M4)


void velCallback(  const geometry_msgs::Twist& vel)
{
     desired_x = vel.linear.x;
     desired_y = vel.linear.y;
     desired_th = vel.angular.z;

     M1_setpoint = (desired_x - desired_y - (L1 + L2)*desired_th)/R;
     M2_setpoint = (desired_x + desired_y - (L1 + L2)*desired_th)/R;
     M3_setpoint = (desired_x + desired_y + (L1 + L2)*desired_th)/R;
     M4_setpoint = (desired_x - desired_y + (L1 + L2)*desired_th)/R;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);  

void setup() {
  nh.initNode();              // init ROS
  nh.subscribe(sub);          // subscribe to cmd_vel

  pinMode(encoder0PinA, INPUT_PULLUP);    // encoder pins
  pinMode(encoder0PinB, INPUT_PULLUP);

  attachInterrupt(18, doEncoderA, CHANGE);
  attachInterrupt(19, doEncoderB, CHANGE);

}

void loop() {
  nh.spinOnce();        // make sure we listen for ROS messages and activate the callback if there is one

}


void doEncoder1(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder1_pinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1_pinB) == LOW) {  
      encoder1_pos = encoder1_pos + 1;         // CW
    } 
    else {
      encoder1_pos = encoder1_pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0_pinB) == HIGH) {   
      encoder0_pos = encoder0_pos + 1;          // CW
    } 
    else {
      encoder0_pos = encoder_pos - 1;          // CCW
    }
  }
}

void doEncoder2(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder2_pinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder2_pinA) == HIGH) {  
      encoder2_pos = encoder2_pos + 1;         // CW
    } 
    else {
      encoder2_pos = encoder2_pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder2_pinA) == LOW) {   
      encoder2_pos = encoder2_pos + 1;          // CW
    } 
    else {
      encoder2_pos = encoder2_pos - 1;          // CCW
    }
  }
}

void doEncoder3(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder3_pinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder3_pinB) == LOW) {  
      encoder3_pos = encoder3_pos + 1;         // CW
    } 
    else {
      encoder3_pos = encoder3_pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder3_pinB) == HIGH) {   
      encoder3_pos = encoder3_pos + 1;          // CW
    } 
    else {
      encoder3_pos = encoder3_pos - 1;          // CCW
    }
  }
}

void doEncoder4(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder4_pinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder4_pinB) == LOW) {  
      encoder4_pos = encoder4_pos + 1;         // CW
    } 
    else {
      encoder4_pos = encoder4_pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder4_pinB) == HIGH) {   
      encoder4_pos = encoder4_pos + 1;          // CW
    } 
    else {
      encoder4_pos = encoder4_pos - 1;          // CCW
    }
  }
}
