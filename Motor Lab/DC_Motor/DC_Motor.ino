#include <util/atomic.h>

//push button (will toggle step_en_state)
const uint16_t BUTTON_PIN = 13;
uint16_t button_state = LOW;
uint16_t last_button_state = LOW;

uint32_t last_debounce_time = 0;
uint32_t debounce_delay = 50; //in ms
////

//DC Motor pins
const uint16_t ENC_A = 2;
const uint16_t ENC_B = 3;
const uint16_t PWM_PIN = 4;
const uint16_t IN1 = 5;
const uint16_t IN2 = 6;

//DC motor vars
const uint16_t TOTAL_COUNT = 600;
volatile uint32_t encoder_count = 0;
uint16_t input_angle = 0;
uint16_t prev_angle;

/****************************************/

//stepper motor pins
const uint16_t STEP_PIN = 8;
const uint16_t STEP_DIR_PIN = 7;
const uint16_t STEP_EN_PIN = 12;

//stepper motor vars
uint16_t step_en_state = 0;
const uint16_t STEPS_PER_REV = 200;
uint32_t curr_step_angle = 0;
int curr_step_count = 0;
uint16_t step_dir = 0; //0 for CW, 1 for CCW

int target_angle = 0;

/****************************************/

// PID globals
long prevT = 0;
int posPrev = 0;
float prev_error = 0.0;
float error = 0.0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

float eintegral = 0.0;
float ederiv = 0.0;
/****************************************/

void setup() {
  Serial.begin(115200); // Serial for Debugging

  //DC Motor setup
  pinMode(ENC_A, INPUT); // ENCODER_A as Input
  pinMode(ENC_B, INPUT); // ENCODER_B as Input
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT); 
  pinMode(PWM_PIN, OUTPUT); // sets motor velocity
  attachInterrupt(digitalPinToInterrupt(ENC_A),read_encoder, RISING);  


  //Stepper motor setup
  pinMode(STEP_DIR_PIN, OUTPUT);
  pinMode(STEP_EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  //Push button setup
  pinMode(BUTTON_PIN, INPUT);

}

void loop() {

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
}

void enable_stepper(uint16_t reading){
  // If the switch changed, due to noise or pressing:
  if (reading != last_button_state) {
    // reset the debouncing timer
    last_debounce_time = millis();
  }

  if ((millis() - last_debounce_time) > debounce_delay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != button_state) {
      button_state = reading;

      // only enable the tepper if the new button state is HIGH
      if (button_state == HIGH) {
        step_en_state = !step_en_state;
      }
    }
  }

  // set the stepper driver enable state
  digitalWrite(STEP_EN_PIN, step_en_state);
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  last_button_state = reading;
}

void stepper_rotate(uint16_t step_en_state, int target_angle){
  if (step_en_state == 0) {
    return;
  }

  if (curr_step_angle > target_angle){
    step_dir = 1;
  }else{
    step_dir = 0;
  }

  digitalWrite(STEP_DIR_PIN, step_dir);
  
  curr_step_angle = map(curr_step_count,0,200,0,360);

  while (curr_step_angle != abs(target_angle)){
    Serial.println("Step Count: " + String(curr_step_count) + " Step Angle: " + String(curr_step_angle) 
                   + " Target Angle: " + String(target_angle) + " Step State: " + String(step_en_state));
    
    digitalWrite(STEP_PIN, HIGH);
    
    curr_step_count = (step_dir) ? curr_step_count-- : curr_step_count++; 
    curr_step_angle = map(curr_step_count,0,200,0,360);
    
    delayMicroseconds(500);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(500);
  }
  return;
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
