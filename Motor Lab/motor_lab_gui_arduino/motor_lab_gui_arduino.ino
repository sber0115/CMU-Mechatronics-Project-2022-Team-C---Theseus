#include <Servo.h>
#include <util/atomic.h>


//Servo
Servo myservo;
const uint16_t SERVO_PIN = 9;
////

//potentiometer
const uint16_t POT_PIN = A2;
uint16_t pot_deg = 0;
////

//ping sensor
const uint16_t PING_ECHO_PIN = 10;
const uint16_t PING_TRIG_PIN = 11;
const uint16_t SPEED_SOUND = 34;
const uint16_t MAX_PING_DISTANCE = 184;
uint32_t ping_distance = 0;
////

//fsr
const uint16_t FSR_PIN = A3;
uint32_t fsr_reading;
////

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

int target_pos = map(90, 0, 360, 0, 800);

////

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

//PID for position
volatile int posi = 0; 
long prevT = 0;
float eprev = 0;
float eintegral = 0;
////

//force sensor
uint32_t force_val = 0;
////

int buf_len = 50;
uint16_t deg;
char pot_buf[50];
char ping_buf[50];
char force_buf[50];
char incoming_buf[50];

size_t read_len;
size_t write_len;
char *token;
bool manual = true;

int servo_speed_manual = 0;
int dc_speed_manual = 0;
int dc_pos_manual = 0;
int stepper_angle_manual = 0;

int servo_speed_sensor = 0;
int dc_speed_sensor = 0;
int stepper_angle_sensor = 0;


void setup() {
  Serial.begin(115200);
  
  //Servo setup
  myservo.attach(SERVO_PIN);

  //Ping sensor setup
  pinMode(PING_ECHO_PIN, INPUT);
  pinMode(PING_TRIG_PIN, OUTPUT);

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
  
  uint16_t button_reading = digitalRead(BUTTON_PIN);
  
  //Serial.println("Reading: " + String(button_reading));
  //Serial.println("Reading: " + String(button_reading) + " Saved State: " + String(step_en_state));  

  //digitalWrite(STEP_EN_PIN, LOW);
  enable_stepper(button_reading);

  while(Serial.available() > 0) {
    read_len = Serial.readBytesUntil('\n', incoming_buf, buf_len);
    if(read_len >= buf_len) continue;    
    incoming_buf[read_len] = '\0';
    token = strtok(incoming_buf, ":");

    if(token != NULL){
      if(strcmp(token, "manual") == 0){
        token = strtok(NULL, ":");
        if(token != NULL){
          if(strcmp(token, "true") == 0){
            manual = true;
          }else if(strcmp(token, "false") == 0){
            manual = false;
          }
        }
      }
      else if(strcmp(token, "servo") == 0){
        token = strtok(NULL, ":");
        if(token != NULL){
          servo_speed_manual = strtol(token, NULL, 10);
        }
      }else if(strcmp(token, "dc_speed") == 0){
        token = strtok(NULL, ":");
        if(token != NULL){
          dc_speed_manual = strtol(token, NULL, 10);
        }
       }else if(strcmp(token, "dc_pos") == 0){
        token = strtok(NULL, ":");
        if(token != NULL){
          dc_pos_manual = strtol(token, NULL, 10);
        }
      }else if(strcmp(token, "stepper") == 0){
        token = strtok(NULL, ":");
        if(token != NULL){
          stepper_angle_manual = strtol(token, NULL, 10);
        }
      } 
    }
  }
  

  pot_deg = read_potentiometer();
  ping_distance = read_ping_sensor();
  fsr_reading = read_fsr();

  Serial.println("FSR Reading: " + String(fsr_reading));
  
  servo_speed_sensor = map(pot_deg, 0, 180, -50, 50);
  stepper_angle_sensor = map(ping_distance, 0, MAX_PING_DISTANCE, 0, 360);
  
  if(manual){
    if (fsr_reading >= 2) {
      adjust_servo_speed(0);

    } else{
      adjust_servo_speed(servo_speed_manual);
    }
//    adjust_dc_pos(180);
    adjust_stepper_angle(stepper_angle_manual);



    // motor power
    int pwr = abs(dc_speed_manual);

    if(pwr > 255){
      pwr = 255;
    }
  
    // motor direction
    int dir = 1;
    if(dc_speed_manual < 0){
      dir = -1;
    }
  
    // signal the motor
    set_motor(dir,IN2,pwr,IN1,IN2);

  }else{
    if (fsr_reading >= 2){
      adjust_servo_speed(0);
    } else{
      adjust_servo_speed(servo_speed_sensor);
    }
    //adjust_dc_pos(dc_speed_manual);
    adjust_stepper_angle(stepper_angle_sensor);
  }


  
  write_len = sprintf(pot_buf, "potentiometer:%d\n", pot_deg);
  Serial.write(pot_buf, write_len);
  
  write_len = sprintf(ping_buf,"ultrasonic:%d\n", ping_distance);
  Serial.write(ping_buf, write_len);
 
  
  delay(200);
  
}

uint16_t read_potentiometer() {
  uint16_t pot_in = analogRead(POT_PIN);
  uint16_t pot_deg = map(pot_in, 0, 1023, 0, 180);
  return pot_deg;
}

uint32_t read_ping_sensor() {     
  uint32_t time_ms = 0;
  uint32_t distance_cm = 0;

  // generates 40kHz ultrasonic pulse for 10ms
  digitalWrite(PING_TRIG_PIN, HIGH);  
  delayMicroseconds(10);    
  digitalWrite(PING_TRIG_PIN, LOW);
  
  // reads sound wave travel time (ms/bit) from echoPin
  time_ms = pulseIn(PING_ECHO_PIN, HIGH); 
   
  // calculates the distance (div. by 2 for time of flight)
  distance_cm = time_ms * SPEED_SOUND / 2 / 1000;    
  return distance_cm;
}

uint32_t read_fsr() {
  uint16_t fsr_in = analogRead(FSR_PIN);
  uint32_t fsr_f = 0;
 
  // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
  uint32_t fsr_v = map(fsr_in, 0, 1023, 0, 5000); 
 
  if (fsr_v != 0) {
    // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
    // so FSR = ((Vcc - V) * R) / V        yay math!
    uint32_t fsr_r = 5000 - fsr_v;     // fsrVoltage is in millivolts so 5V = 5000mV
    fsr_r *= 10000;                // 10K resistor
    fsr_r /= fsr_v;
    uint32_t fsr_g = 1000000;           // we measure in micromhos so 
    fsr_g /= fsr_r;
 
    // Use the two FSR guide graphs to approximate the force
    if (fsr_g <= 1000) {
      fsr_f = fsr_g / 80;   
    } else {
      fsr_f = fsr_g - 1000;
      fsr_f /= 30;          
    }
  }
  return fsr_f;
}


void adjust_servo_speed(int servo_speed) {
  myservo.writeMicroseconds(map(servo_speed,-50 ,50, 1309, 1710));
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

void adjust_stepper_angle(int target_angle){
  if (curr_step_angle > target_angle){
    step_dir = 1;
  }else{
    step_dir = 0;
  }

  digitalWrite(STEP_DIR_PIN, step_dir);
  
  curr_step_angle = map(curr_step_count,0,200,0,360);

  while (abs(curr_step_angle - target_angle) > 2){
    if (step_dir) {
      curr_step_count--;
    }else{
      curr_step_count++;
    }
    curr_step_angle = map(curr_step_count,0,200,0,360);
    
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000);
  }
  return;
}

void val_adjust_stepper_angle(int target_angle){
  curr_step_angle = map(curr_step_count,0,STEPS_PER_REV,0,360);

  while (abs(curr_step_angle - target_angle) > 2){
    if ((curr_step_angle > target_angle && abs(curr_step_angle - target_angle) < 180) ||
        (curr_step_angle < target_angle && abs(curr_step_angle - target_angle) >= 180)){
      curr_step_count--;
      digitalWrite(STEP_DIR_PIN, HIGH);
    }else{
      curr_step_count++;
      digitalWrite(STEP_DIR_PIN, LOW);
    }
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000);
    curr_step_count %= STEPS_PER_REV;
    curr_step_angle = map(curr_step_count,0,STEPS_PER_REV,0,360);
  }
  return;
}


void adjust_dc_pos(uint16_t target){
  // set target position
  //int target = 1200;
  //int target = 250*sin(prevT/1e6);
  target_pos = map(target, 0, 360, 0, 800);

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  
  // error
  int e = pos - target_pos;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  set_motor(dir,pwr,PWM_PIN,IN1,IN2);

  // store previous error
  eprev = e;

  Serial.print(target_pos);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
  
}

void read_encoder(){
  int b = digitalRead(ENC_B);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}

void set_motor(int dir, int pwmPin, int pwmVal, int in1, int in2) {
  analogWrite(IN2,pwmVal); // Motor speed
  if (dir == 1){ 
    // Turn one way
    digitalWrite(4,HIGH);
    digitalWrite(IN1,HIGH);
  }
  else if (dir == -1){
    // Turn the other way
    digitalWrite(IN1,LOW);
    digitalWrite(4,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);    
  }
}
