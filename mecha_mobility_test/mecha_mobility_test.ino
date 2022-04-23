#include "theseus.h"
#include <Servo.h>
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

//body velocity translation parameters
const float R = 0.09;
const float L1 = 0.1778;
const float L2 = 0.1778;

float desired_x;
float desired_y;
float desired_th;

////

Servo lac_v;
Servo lac_h;

int pos_v = 0;
int pos_h = 0;

// motor constants
const uint32_t TICKS_PER_ROTATION = 16*50;
const uint32_t PULSES_PER_CM = (TICKS_PER_ROTATION * 1000) / 3047;

volatile long current_time = 0;
volatile long elapsed_time = 0; 
volatile long previous_time = 0;

//ignore
uint32_t current_ticks = 0;

//total encoder count for each motor
volatile int current_pos1 = 0;
volatile int current_pos2 = 0;
volatile int current_pos3 = 0;
volatile int current_pos4 = 0;

uint32_t elapsed_ticks = 0; 
uint32_t previous_ticks = 0;

uint8_t STEPPER_1 = 10;
uint8_t STEPPER_2 = 11;

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

/*
Encoder M1_enc(M1.ENCA, M1.ENCB); 
Encoder M2_enc(M2.ENCA, M2.ENCB);
Encoder M3_enc(M3.ENCA, M3.ENCB);
Encoder M4_enc(M4.ENCA, M4.ENCB);
*/

                  //  KP KI KD UMIN UMAX
pid_params_t M1_PID = {1,0,0,0,255};
pid_params_t M2_PID = {1,0,0,0,255};
pid_params_t M3_PID = {1,0,0,0,255};
pid_params_t M4_PID = {1,0,0,0,255};

directive_t M1_command;

volatile float M1_velocity = 0;
volatile float M2_velocity = 0;
volatile float M3_velocity = 0;
volatile float M4_velocity = 0;


//To convert between radians/s and revs/m, multiply rads/s by 9.55
volatile float M1_sp = 200; // rev/min
volatile float M2_sp = 0;
volatile float M3_sp = 0;
volatile float M4_sp = 0;


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

  attachInterrupt(digitalPinToInterrupt(18), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(19), readEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(20), readEncoder3, RISING);
  attachInterrupt(digitalPinToInterrupt(21), readEncoder4, RISING);

  nh.initNode();              // init ROS
  nh.subscribe(sub);          // subscribe to cmd_vel
  
  //Serial.println("Input a position");

}

void loop() {
  nh.spinOnce();        // make sure we listen for ROS messages and activate the callback if there is one

  M1_command = do_pid(current_pos1);
  
  /*
  Serial.println("M1 ticks: " + String(current_pos1));
  Serial.println("M2 ticks: " + String(current_pos2));
  Serial.println("M3 ticks: " + String(current_pos3));
  Serial.println("M4 ticks: " + String(current_pos4));

  delay(1000);
  */

  move();

  Serial.print(M1_command.speed);
  Serial.print(" ");
  Serial.print(M1_sp);
  Serial.println();

  /*
  Serial.println("EN1 pos: " + String(M1_enc.read()));
  Serial.println("EN2 pos: " + String(M2_enc.read()));
  Serial.println("EN3 pos: " + String(M3_enc.read()));
  Serial.println("EN4 pos: " + String(M4_enc.read()));
  */

  /*
  while (Serial.available()==0){}
  arg1 = Serial.read();
  arg2 = Serial.parseInt();
  switch(arg1) {
    case 'w':
      move(FWD, speed);
      break;
    case 's':
      move(BACK, speed);
      break;
    case 'a':
      move(LEFT, speed);
      break;
    case 'd':
      move(RIGHT, speed);
      break;
    case 'q':
      move(ROT_CCW, speed);
      break;
    case 'e':
      move(ROT_CW, speed);
      break;
    case ' ':
      move(STOP, speed);
      break;
    default:
      Serial.println("Invalid Command.");  
  }
  */
}

directive_t do_pid(double current_vel) {
  directive_t out = {0,BRAKE};

  int32_t error = M1_sp - M1_velocity;
  int32_t integral = error * elapsed_time;
  int32_t derivative = (error - M1_prev_error) / elapsed_time;

  int32_t speed = M1_PID.KD * derivative + M1_PID.KI * integral + M1_PID.KP * error;
  uint32_t abs_speed = abs(speed);
  out.speed = constrain(abs_speed, 0, 255);
  out.direction = (speed < 0 ? CCW : CW);

  return out;

}


void move() {
  
  motor_drive(M1, M1_command.speed, M1_command.direction);
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
  int b = digitalRead(M1.ENCB);
  int increment = 0;
  if (b > 0){
    increment = 1;
  }
  else{
    increment = -1;
  }
  current_pos1 += increment;
  long current_time = micros();
  float elapsed_time = ((float)(current_time - previous_time)/1.0e6);
  //
  M1_velocity = (increment)/elapsed_time; //seconds / encoder ticker
  
  //converting secs/encoder count to rotations per minute
  M1_velocity = ((1/M1_velocity)*60)/TICKS_PER_ROTATION;
  
  previous_time = current_time;
}

void readEncoder2(){
  int b = digitalRead(M2.ENCB);
  if (b > 0){
    current_pos2++;
  }
  else{
    current_pos2--;
  }
}

void readEncoder3(){
  int b = digitalRead(M3.ENCB);
  if (b > 0){
    current_pos3++;
  }
  else{
    current_pos3--;
  }
}

void readEncoder4(){
  int b = digitalRead(M4.ENCB);
  if (b > 0){
    current_pos4++;
  }
  else{
    current_pos4--;
  }
}
