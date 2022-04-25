#include "theseus.h"
#include <Servo.h>
/* #include "ros.h"
#include "geometry_msgs/Twist.h"
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/UInt16.h>
*/
/*
ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
*/

/*
 * motor driver pins
 *
 *    (front)
 *  M1       M3 <- reversed
 *
 *
 *  M2       M4 <- reversed
 *    (back )
 *              ~=PWM, *=INTERRUPT
 */           // EN~, IN1, IN2, ENCA*, ENCB*              
motor_t M1 = {2,22,23,18,A9};
motor_t M2 = {3,24,25,19,A11}; 
motor_t M3 = {4,26,29,20,A13}; 
motor_t M4 = {5,27,28,21,A15}; 

//body velocity translation parameters
const float R = 0.09;
const float L1 = 0.1778;
const float L2 = 0.1778;

float desired_x;
float desired_y;
float desired_th;


const uint32_t TEN_TO_THE_6 = 1000000;
////

Servo lac_v;
Servo lac_h;

int pos_v = 0;
int pos_h = 0;

// motor constants
const int32_t TICKS_PER_ROTATION = 16*50;

volatile int32_t current_time = 0;
volatile int32_t elapsed_time = 0; 

//ignore
int32_t current_ticks = 0;

//total encoder count for each motor
volatile int32_t current_pos1 = 0;
volatile int32_t current_pos2 = 0;
volatile int32_t current_pos3 = 0;
volatile int32_t current_pos4 = 0;

uint8_t STEPPER_1 = 10;
uint8_t STEPPER_2 = 11; 

/*
Encoder M1_enc(M1.ENCA, M1.ENCB); 
Encoder M2_enc(M2.ENCA, M2.ENCB);
Encoder M3_enc(M3.ENCA, M3.ENCB);
Encoder M4_enc(M4.ENCA, M4.ENCB);
*/


                  //  KP KI KD BIAS UMIN UMAX

const int32_t KP = 800;
const int32_t KI = 20;
const int32_t KD = 2000;
pid_params_t M1_PID = {KP,KI,KD,0,0,255}; 
pid_params_t M2_PID = {KP,KI,KD,0,0,255};
pid_params_t M3_PID = {KP,KI+8,KD,0,0,255};
pid_params_t M4_PID = {KP,KI,KD,0,0,255};

directive_t M1_command;
directive_t M2_command;
directive_t M3_command;
directive_t M4_command;

volatile int32_t M1_velocity = 0;
volatile int32_t M2_velocity = 0;
volatile int32_t M3_velocity = 0;
volatile int32_t M4_velocity = 0;


//To convert between radians/s and revs/m, multiply rads/s by 9.55
volatile int32_t M1_sp = 40; // rev/min
volatile int32_t M2_sp = 40;
volatile int32_t M3_sp = 40;
volatile int32_t M4_sp = 40;

/*
void velCallback(  const geometry_msgs::Twist& vel)
{
     desired_x = vel.linear.x;
     desired_y = vel.linear.y;
     desired_th = vel.angular.z;

     M1_sp = 9.55*(desired_x - desired_y - (L1 + L2)*desired_th)/R;
     M2_sp = 9.55*(desired_x + desired_y - (L1 + L2)*desired_th)/R;
     M3_sp = 9.55*(desired_x + desired_y + (L1 + L2)*desired_th)/R;
     M4_sp = 9.55*(desired_x - desired_y + (L1 + L2)*desired_th)/R;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);  
*/

uint32_t M1_prev_error = 0;

char arg1 = 'g';
int arg2 = 0;

void setup() {
  Serial.begin(115200);

  motor_initialize(M1);
  motor_initialize(M2);
  motor_initialize(M3);
  motor_initialize(M4);  

  lac_v.attach(8);
  lac_h.attach(9);

  attachInterrupt(digitalPinToInterrupt(M1.ENCA), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2.ENCA), readEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(M3.ENCA), readEncoder3, RISING);
  attachInterrupt(digitalPinToInterrupt(M4.ENCA), readEncoder4, RISING);
  /*
  nh.initNode();              // init ROS
  nh.subscribe(sub);          // subscribe to cmd_vel
  */
  //Serial.println("Input a position");

}

void loop() {
  // nh.spinOnce();        // make sure we listen for ROS messages and activate the callback if there is one

  current_time = millis();
  static int32_t previous_time = 0;
  elapsed_time = current_time - previous_time;
  
  static int32_t previous_ticks1 = 0;
  int32_t elapsed_ticks1 = current_pos1 - previous_ticks1;
  M1_velocity = (elapsed_ticks1 * 60000L) / (elapsed_time * TICKS_PER_ROTATION);
  previous_ticks1 = current_pos1;

  static int32_t previous_ticks2 = 0;
  int32_t elapsed_ticks2 = current_pos2 - previous_ticks2;
  M2_velocity = (elapsed_ticks2 * 60000L) / (elapsed_time * TICKS_PER_ROTATION);
  previous_ticks2 = current_pos2;

  static int32_t previous_ticks3 = 0;
  int32_t elapsed_ticks3 = current_pos3 - previous_ticks3;
  M3_velocity = (elapsed_ticks3 * 60000L) / (elapsed_time * TICKS_PER_ROTATION);
  previous_ticks3 = current_pos3;

  static int32_t previous_ticks4 = 0;
  int32_t elapsed_ticks4 = current_pos4 - previous_ticks4;
  M4_velocity = (elapsed_ticks4 * 60000L) / (elapsed_time * TICKS_PER_ROTATION);
  previous_ticks4 = current_pos4;

  do_pid();
  //lSerial.println(M1_command.speed);
  
   move();

  previous_time = current_time;
  

  //Serial.print("CMD:");
  //Serial.print(M1_command.speed);
  Serial.print(",SP:");
  Serial.print(M1_sp);
  //Serial.print(",T:");
  //Serial.print(elapsed_time);
  //Serial.print(",POS:");
  //Serial.print(elapsed_ticks);
  Serial.print(",V1:");
  Serial.print(String(M1_velocity));
  Serial.print(",V2:");
  Serial.print(String(M2_velocity));
  Serial.print(",V3:");
  Serial.print(String(M3_velocity));
  Serial.print(",V4:");
  Serial.println(String(M4_velocity));
  Serial.println();
  delay(100);
  unsigned long CurrentTime = millis();
  if (CurrentTime > 6000){
    M1_sp = -80;
    M2_sp = -80;
    M3_sp = -80;
    M4_sp = -80;
  }
}

int32_t sum_error1 = 0;
int32_t sum_error2 = 0;
int32_t sum_error3 = 0;
int32_t sum_error4 = 0;
int32_t prev_error1 = 0;
int32_t prev_error2 = 0;
int32_t prev_error3 = 0;
int32_t prev_error4 = 0;

void do_pid() {
  // Motor 1 PID
  directive_t out1 = {0,BRAKE};
  int32_t error1 = M1_sp - M1_velocity;
  sum_error1 += error1;
  int32_t integral1 = sum_error1 * elapsed_time;
  int32_t derivative1 = M1_PID.KD * (error1 - prev_error1) / elapsed_time;
  prev_error1 = error1;
  int32_t speed1 = derivative1 + M1_PID.KI * integral1 + M1_PID.KP * error1 + M1_PID.BIAS;
  uint32_t abs_speed1 = abs(speed1/10000);
  out1.speed = constrain(abs_speed1, 0, 255);
  out1.direction = (speed1 < 0 ? CCW : CW);

  // Motor 2 PID
  directive_t out2 = {0,BRAKE};
  int32_t error2 = M2_sp - M2_velocity;
  sum_error2 += error2;
  int32_t integral2 = sum_error2 * elapsed_time;
  int32_t derivative2 = (error2 - prev_error2) / elapsed_time;
  prev_error2 = error2;
  int32_t speed2 = M2_PID.KD * derivative2 + M2_PID.KI * integral2 + M2_PID.KP * error2 + M2_PID.BIAS;
  uint32_t abs_speed2 = abs(speed2/10000);
  out2.speed = constrain(abs_speed2, 0, 255);
  out2.direction = (speed2 < 0 ? CCW : CW);

  // Motor 3 PID
  directive_t out3 = {0,BRAKE};
  int32_t error3 = M3_sp - M3_velocity;
  sum_error3 += error3;
  int32_t integral3 = sum_error3 * elapsed_time;
  int32_t derivative3 = (error3 - prev_error3) / elapsed_time;
  prev_error3 = error3;
  int32_t speed3 = M3_PID.KD * derivative3 + M3_PID.KI * integral3 + M3_PID.KP * error3 + M3_PID.BIAS;
  uint32_t abs_speed3 = abs(speed3/10000);
  out3.speed = constrain(abs_speed3, 0, 255);
  out3.direction = (speed3 < 0 ? CCW : CW);

  // Motor 4 PID
  directive_t out4 = {0,BRAKE};
  int32_t error4 = M4_sp - M4_velocity;
  sum_error4 += error4;
  int32_t integral4 = sum_error4 * elapsed_time;
  int32_t derivative4 = (error4 - prev_error4) / elapsed_time;
  prev_error4 = error4;
  int32_t speed4 = M4_PID.KD * derivative4 + M4_PID.KI * integral4 + M4_PID.KP * error4 + M4_PID.BIAS;
  uint32_t abs_speed4 = abs(speed4/10000);
  out4.speed = constrain(abs_speed4, 0, 255);
  out4.direction = (speed4 < 0 ? CCW : CW);

  M1_command = out1;
  M2_command = out2;
  M3_command = out3;
  M4_command = out4;

}


void move() {
  
  motor_drive(M1, M1_command.speed, M1_command.direction);
  motor_drive(M2, M2_command.speed, M2_command.direction);
  motor_drive(M3, M3_command.speed, M3_command.direction);
  motor_drive(M4, M4_command.speed, M4_command.direction);
  
}

/*
 * motor_initialize(): Initializes motor GPIO pins
 *   motor_t motor   - DC motor to initialize
 */
void motor_initialize(motor_t motor) {
  pinMode(motor.EN, OUTPUT);
  pinMode(motor.IN1, OUTPUT);
  pinMode(motor.IN2, OUTPUT);
  pinMode(motor.ENCA, INPUT);
  pinMode(motor.ENCB, INPUT); 
  /*
  m_enc.write(0);  
  */
  return;
}

/*
 * motor_drive(): Spins specified motors in direction/speed provided
 *   motor_t motor   - DC motor to turn
 *   uint8_t speed   - Motor speed (0-255)
 *   dir_t direction - Motor direction (CW, CCW, BRAKE)
 */
void motor_drive(motor_t motor, uint8_t speed, dir_t direction) {
  analogWrite(motor.EN, speed);
  switch(direction) {
    case CW: 
      digitalWrite(motor.IN1, LOW);
      digitalWrite(motor.IN2, HIGH);
      break;
    case CCW:
      digitalWrite(motor.IN1, HIGH);
      digitalWrite(motor.IN2, LOW);
      break;
    case BRAKE:
      digitalWrite(motor.IN1, LOW);
      digitalWrite(motor.IN2, LOW);   
      break; 
    default: 
      Serial.println("ERROR: motor_drive(). Braking...");
      digitalWrite(motor.IN1, LOW);
      digitalWrite(motor.IN2, LOW);   
      break;  
  }
  return;
}

void readEncoder1(){
  current_pos1 += (PINK & (1<<1) ? 1 : -1);
}

void readEncoder2(){
  current_pos2 += (PINK & (1<<3) ? 1 : -1);
}

void readEncoder3(){
  current_pos3 += (PINK & (1<<5) ? 1 : -1);
}

void readEncoder4(){
  current_pos4 += (PINK & (1<<7) ? 1 : -1);
}
