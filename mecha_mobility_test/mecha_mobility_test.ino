#include "theseus.h"
//#include "Mecanum.h"
#include <Servo.h>
#include <Stepper.h>
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32MultiArray.h>

const uint32_t STEPS = 1000;
uint32_t curr_step = 0;
uint32_t input_angle = 0;

Stepper stepper(STEPS,A0,A1);

std_msgs::Float32MultiArray enc_msg;

ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
ros::Publisher enc_pub("encoders", &enc_msg);


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
motor_t M3 = {4,28,29,20,A13}; 
motor_t M4 = {5,26,27,21,A15}; 

//body velocity translation parameters
const float R = 0.0485;
const float L1 = 0.127;
const float L2 = 0.138;

float desired_x;
float desired_y;
float desired_th;
////

//odom parameters
const int32_t odom_period = 100;
//

Servo lac_v;
Servo lac_h;

uint32_t pos_v = 30;
uint32_t pos_h = 30;

// motor constants
const int32_t TICKS_PER_ROTATION = 16*50;

volatile int32_t current_time = 0;
volatile int32_t elapsed_time = 0; 
volatile int32_t odom_timer = 0;

//ignore
int32_t current_ticks = 0;

//total encoder count for each motor
volatile int32_t current_pos1 = 0;
volatile int32_t current_pos2 = 0;
volatile int32_t current_pos3 = 0;
volatile int32_t current_pos4 = 0;

int32_t last_encoder_counts[4];

                  //  KP KI KD BIAS UMIN UMAX
const int32_t KP = 800; //800
const int32_t KI = 20; //20
const int32_t KD = 2500; //2500
const int32_t BIAS = 0L;

/*
pid_params_t M1_PID = {KP+1000,KI,KD-150,BIAS,0,255}; 
pid_params_t M2_PID = {KP,KI,KD,BIAS,0,255};
pid_params_t M3_PID = {KP,KI+8,KD,BIAS,0,255};
pid_params_t M4_PID = {KP,KI,KD,BIAS,0,255};
*/

pid_params_t M1_PID = {KP,KI,KD,BIAS,0,255}; 
pid_params_t M2_PID = {KP,KI,KD,BIAS,0,255};
pid_params_t M3_PID = {KP,KI,KD,BIAS+10000,0,255};
pid_params_t M4_PID = {KP,KI,KD,BIAS,0,255};

directive_t M1_command;
directive_t M2_command;
directive_t M3_command;
directive_t M4_command;

volatile int32_t M1_velocity = 0;
volatile int32_t M2_velocity = 0;
volatile int32_t M3_velocity = 0;
volatile int32_t M4_velocity = 0;


//To convert between radians/s and revs/m, multiply rads/s by 9.55

/*
volatile int32_t M1_sp = 0; // rev/min
volatile int32_t M2_sp = 0;
volatile int32_t M3_sp = 0;
volatile int32_t M4_sp = 0;
*/

volatile int32_t M1_sp = 40; // rev/min
volatile int32_t M2_sp = 40;
volatile int32_t M3_sp = 40;
volatile int32_t M4_sp = 40;


void velCallback(  const geometry_msgs::Twist& vel)
{
  velocity_transform(vel.linear.x, vel.linear.y, vel.angular.z);
}

void armCallback(  const geometry_msgs::Point& arm)
{
  arm_transform(arm.x, arm.y, arm.z);
}

//ros::Subscriber<geometry_msgs::Twist> sub1("cmd_vel" , velCallback);
//ros::Subscriber<geometry_msgs::Point> sub2("cmd_arm" , armCallback);

void setup() {
  Serial.begin(115200);

  motor_initialize(M1);
  motor_initialize(M2);
  motor_initialize(M3);
  motor_initialize(M4);  

  //lac_v.attach(8);
  //lac_h.attach(9);

  stepper.setSpeed(50);
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(M1.ENCA), read_enc1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2.ENCA), read_enc2, RISING);
  attachInterrupt(digitalPinToInterrupt(M3.ENCA), read_enc3, RISING);
  attachInterrupt(digitalPinToInterrupt(M4.ENCA), read_enc4, RISING);
  /*  
  nh.initNode();              // init ROS
  nh.subscribe(sub1);          // subscribe to cmd_vel
  nh.subscribe(sub2);
  nh.advertise(enc_pub);      //advertise the encoder info
  enc_msg.data = (float *)malloc(sizeof(float)*4);
  enc_msg.data_length = 4;
  */
  
  //Serial.println("Input a position");

}

void loop() {
  //nh.spinOnce();        // make sure we listen for ROS messages and activate the callback if there is one

  // angle per step
  int32_t input_step = map(input_angle,0,360,0,200);
  
  // move to the target angle
  digitalWrite(A2, HIGH);
  stepper.step((input_step - curr_step)*4);
  lac_v.write(pos_v); 
  lac_h.write(pos_h);
  // save the new current angle
  curr_step = input_step;

  current_time = millis();
  static int32_t previous_time = 0;
  elapsed_time = current_time - previous_time;


  // send odom
  if((current_time - odom_timer) > odom_period) {
    // grab data from encoders
    get_encoder_counts(enc_msg.data); // returned as x, y, theta
    enc_msg.data[3] = (float)(current_time - odom_timer) / 1000;
    
    // publish data
    enc_pub.publish(&enc_msg);
    if((current_time - odom_period) > (odom_timer + odom_period)) {
      odom_timer = current_time;
    }
    else {
      odom_timer = odom_timer + odom_period;
    }
  }
  //end odom
  
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
  Serial.print("SP1:");
  Serial.print(M1_sp);
  Serial.print(",SP2:");
  Serial.print(M2_sp);
  Serial.print(",SP3:");
  Serial.print(M3_sp);
  Serial.print(",SP4:");
  Serial.print(M4_sp);  
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
  /*
  Serial.print(",CMD1:");
  Serial.print(String(M1_command.speed));
  Serial.print(",CMD2:");
  Serial.print(String(M2_command.speed));
  Serial.print(",CMD3:");
  Serial.print(String(M3_command.speed));
  Serial.print(",CMD4:");
  Serial.println(String(M4_command.speed));*/
  delay(100);
  
}

void velocity_transform(float x, float y, float theta) {
  M1_sp = (int32_t) 9.55*(x - y - (L1 + L2)*theta)/R;
  M2_sp = (int32_t) 9.55*(x + y - (L1 + L2)*theta)/R;
  M3_sp = (int32_t) 9.55*(x + y + (L1 + L2)*theta)/R;
  M4_sp = (int32_t) 9.55*(x - y + (L1 + L2)*theta)/R;
}

void arm_transform(float h, float s, float v) {
  
  input_angle = (uint32_t) s;
  v *= 1000;
  h *= 1000;
  pos_v = map((uint32_t) v, 0, 80, 30, 100);
  
  pos_h = map((uint32_t) h, 0, 87, 30, 130);
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
  out1.direction = ((speed1 < 0) ? CCW : CW);

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
  out2.direction = ((speed2 < 0) ? CCW : CW);

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
  out3.direction = ((speed3 < 0) ? CCW : CW);

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
  out4.direction = ((speed4 < 0) ? CCW : CW);

  M1_command = out1;
  M2_command = out2;
  M3_command = out3;
  M4_command = out4;

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
  return;
}

void move() {
  motor_drive(M1, M1_command.speed, M1_command.direction);
  motor_drive(M2, M2_command.speed, M2_command.direction);
  motor_drive(M3, M3_command.speed, M3_command.direction);
  motor_drive(M4, M4_command.speed, M4_command.direction);
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
      digitalWrite(motor.IN1, HIGH);
      digitalWrite(motor.IN2, LOW);
      break;
    case CCW:
      digitalWrite(motor.IN1, LOW);
      digitalWrite(motor.IN2, HIGH);
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

void read_enc1(){ current_pos1 += (PINK & (1<<1) ? 1 : -1); }
void read_enc2(){ current_pos2 += (PINK & (1<<3) ? 1 : -1); }
void read_enc3(){ current_pos3 += (PINK & (1<<5) ? 1 : -1); }
void read_enc4(){ current_pos4 += (PINK & (1<<7) ? 1 : -1); }

void get_encoder_counts(float *xyt_counts){
  int32_t newEncoderCounts[4];
  newEncoderCounts[0] = current_pos1; //front-left (M1)
  newEncoderCounts[1] = current_pos3; //front-right (M3)
  newEncoderCounts[2] = current_pos2; //back-left (M2)
  newEncoderCounts[3] = current_pos4; //back-right (M4)
  
  // find deltas
  int deltaEncoderCounts[4];
  for(int i = 0; i < 4; i++) {
    // check for overflow
    if(abs(last_encoder_counts[i]) > COUNT_OVERFLOW && abs(newEncoderCounts[i]) > COUNT_OVERFLOW && sign(last_encoder_counts[i]) != sign(newEncoderCounts[i])) {
      if(sign(last_encoder_counts[i]) > 0)
        deltaEncoderCounts[i] = newEncoderCounts[i] - last_encoder_counts[i] + INT_MAX;
      else
        deltaEncoderCounts[i] = newEncoderCounts[i] - last_encoder_counts[i] - INT_MAX;
    }
    else deltaEncoderCounts[i] = newEncoderCounts[i] - last_encoder_counts[i];

    /*
    // check for gross change -> MD25 board power cycled
    if(abs(deltaEncoderCounts[i]) > COUNT_RESET) deltaEncoderCounts[i] = 0;
    // no idea what the real value is in this case
    */
    
    // save encoder counts
    last_encoder_counts[i] = newEncoderCounts[i];
  }
  
  // convert the motor counts into x, y, theta counts
  xyt_counts[0] = (deltaEncoderCounts[0] + deltaEncoderCounts[1] + deltaEncoderCounts[2] + deltaEncoderCounts[3]) / 4;
  xyt_counts[1] = (0 - deltaEncoderCounts[0] + deltaEncoderCounts[1] + deltaEncoderCounts[2] - deltaEncoderCounts[3]) / 4;
  xyt_counts[2] = (0 - deltaEncoderCounts[0] + deltaEncoderCounts[1] - deltaEncoderCounts[2] + deltaEncoderCounts[3]) / 4;
}
