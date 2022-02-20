#include <Servo.h>

//servo
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
const uint16_t MAX_PING_DISTANCE = 60;
uint32_t ping_distance = 0;
////

//force sensor
uint32_t force_val = 0;
////

//stepper motor
const uint16_t STEP_PIN = 8;
const uint16_t STEPS_PER_REV = 200;
int curr_step_angle = 0;
int curr_step_count = 0;
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
int stepper_angle_manual = 0;

int servo_speed_sensor = 0;
int dc_speed_sensor = 0;
int stepper_angle_sensor = 0;


void setup() {
  // put your setup code here, to run once:
  myservo.attach(SERVO_PIN);
  pinMode(PING_ECHO_PIN, INPUT);
  pinMode(PING_TRIG_PIN, OUTPUT);
  
  Serial.begin(115200);
}
// Continuous servo stops when writing 1510 microseconds to it.
void loop() {

  //Serial.println("Ping distance: " + String(ping_distance));

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
      }else if(strcmp(token, "dc") == 0){
        token = strtok(NULL, ":");
        if(token != NULL){
          dc_speed_manual = strtol(token, NULL, 10);
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
  servo_speed_sensor = map(pot_deg, 0, 180, -50, 50);
  stepper_angle_sensor = map(ping_distance, 0, MAX_PING_DISTANCE, 0, 360);
  
  if(manual){
    adjust_servo_speed(servo_speed_manual);
//    adjust_dc_speed(dc_speed_manual);
    //adjust_stepper_angle(stepper_angle_manual);
  }else{
    adjust_servo_speed(servo_speed_sensor);
//    adjust_dc_speed(dc_speed_manual);
    adjust_stepper_angle(stepper_angle_manual);
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


void adjust_servo_speed(int servo_speed) {
  myservo.writeMicroseconds(map(servo_speed,-50 ,50, 1309, 1710));
}

void adjust_stepper_angle(int target_angle){
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
