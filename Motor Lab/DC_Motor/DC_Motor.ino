#include <PIDController.h>
#include <Encoder.h>


// Pin definitions
const uint16_t ENC_A = 2;
const uint16_t ENC_B = 3;
const uint16_t ENA = 4;
const uint16_t IN1 = 5;
const uint16_t IN2 = 6;

enum dir_t { CW, CCW, STOP };

PIDController pid;
Encoder motor(2,3);


// DC motor vars
const uint16_t TICKS_PER_ROTATION = 2400;

const double K_P = 0.5;
const double K_I = 2.5;
const double K_D = 60;

int32_t motor_pos = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(ENC_A, INPUT); // ENCODER_A as Input
  pinMode(ENC_B, INPUT); // ENCODER_B as Input
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT); // sets motor velocity
  pinMode(ENA, OUTPUT);

  pid.begin();     
  pid.setpoint(0);  
  pid.tune(K_P, K_I, K_D);  
  pid.limit(0, 255);

}

void loop() {
<<<<<<< HEAD

  digitalWrite(STEP_EN_PIN, step_en_state);
  digitalWrite(STEP_EN_PIN, LOW);

  uint16_t button_reading = digitalRead(BUTTON_PIN);
  //Serial.println("Reading: " + String(button_reading));
  //enable_stepper(button_reading);
  //stepper_rotate(step_en_state, target_angle);

  val_adjust_stepper_angle(90);
      
  //Serial.println("Reading: " + String(button_reading) + " Saved State: " + String(step_en_state)); 
  

  /*
  prev_angle = get_angle(encoder_count);
    
  // read the position in an atomic block
  // to avoid potential misreads
  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    velocity2 = velocity_i;
  }

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;
  prev_error = error;
  

  // Convert count/s to RPM
  float v1 = velocity1/320.0*60.0;
  float v2 = velocity2/320.0*60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

  // Set a target
  float vt = 200*(sin(currT/1e6)>0);

  // Compute the control signal u
  float kp = 2;
  float kd = 0;
  float ki = 0;
  
  error = vt-v1Filt;
  
  eintegral = eintegral + error*deltaT;
  ederiv = error - prev_error;
  
  float u = kp*error + kd*ederiv + ki*eintegral;

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }
  
  motor_drive(1,PWM_PIN,pwr,IN1,IN2);


  Serial.print("V1:");
  Serial.print(v1);
  Serial.print(",");
  Serial.print("V1_Filt:");
  Serial.println(v1Filt);

  */
  
  delay(1);

}


void motor_drive(int dir, int pwmPin, int pwmVal, int in1, int in2) {
  analogWrite(pwmPin,pwmVal); // Motor speed
  if (dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if (dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}


uint16_t get_angle(uint32_t encoder_count){
  return (encoder_count * 360)/TOTAL_COUNT;
}


void read_encoder(){
  encoder_count++;
  encoder_count %= TOTAL_COUNT;

  //Serial.println(encoder_count);
  
  // Read encoder B when ENCA rises
  int b = digitalRead(ENC_B);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
}


void stepper_full_rotation(){
  // Spin the stepper motor 1 revolution slowly:
  for (uint16_t i = 0; i < STEPS_PER_REV; i++) {
    // These four lines result in 1 step:
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000);
  }

/*
  motor_pos = motor.read();
  int32_t next_pos = pid.compute(motor_pos);
  motor_drive(100, ((motor_pos - next_pos) > 0) ? CCW : CW);
  Serial.println("[" + String(millis() % 10000) + "]" + " Encoder Loc: " + String(motor_pos) + "->" + String(next_pos));
  pid.setpoint(128);
  delay(10); 
}

*/

void motor_drive(uint16_t speed, dir_t direction) {
  analogWrite(IN2, speed);
  switch(direction) {
    case CW: 
      digitalWrite(ENA,HIGH);
      digitalWrite(IN1,HIGH);
      break;
    case CCW:
      digitalWrite(ENA,HIGH);
      digitalWrite(IN1,LOW);
      break;
    default:
      digitalWrite(IN1,LOW);
      digitalWrite(IN2,LOW);   
      break;  
  }
}

void val_adjust_stepper_angle(int target_angle){

  curr_step_angle = map(curr_step_count,0,STEPS_PER_REV,0,360);

  while (abs(curr_step_angle - target_angle) > 5){
    if ((curr_step_angle > target_angle && abs(curr_step_angle - target_angle) < 180) ||
        (curr_step_angle < target_angle && abs(curr_step_angle - target_angle) >= 180)){
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(500);
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(500);
      curr_step_count--;
    }else{
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(500);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(500);
      curr_step_count++;
    }
    curr_step_count %= STEPS_PER_REV;
    curr_step_angle = map(curr_step_count,0,STEPS_PER_REV,0,360);
  }
  return;
}
