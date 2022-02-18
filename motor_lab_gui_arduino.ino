#include <Servo.h>

Servo myservo;
const uint16_t POT_PIN = A2;

int buf_len = 50;
uint16_t deg;
char buf[50];
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

uint16_t pot_deg = 0;

void setup() {
  // put your setup code here, to run once:
  myservo.attach(9);
  Serial.begin(9600);
}
// Continuous servo stops when writing 1510 microseconds to it.
void loop() {
  while(Serial.available() > 0) {
    read_len = Serial.readBytesUntil('\n', buf, buf_len);
    if(read_len >= buf_len) continue;
    
    buf[read_len] = '\0';
    token = strtok(buf, ":");

    if(token != NULL){
      if(strcmp(token, "manual") == 0){
        token = strtok(NULL, ":");
        if(token != NULL){
          if(strcmp(token, "true")){
            manual = true;
          }else if(strcmp(token, "false")){
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
  servo_speed_sensor = adjust_servo_speed(map(pot_deg, 0, 180, -50, 50));
  
  if(manual){
    adjust_servo_speed(servo_speed_manual);
//    adjust_dc_speed(dc_speed_manual);
//    adjust_stepper_angle(stepper_angle_manual);
  }else{
    adjust_servo_speed(servo_speed_sensor);
//    adjust_dc_speed(dc_speed_manual);
//    adjust_stepper_angle(stepper_angle_manual);
  }
  
  write_len = sprintf(buf, "potentiometer:%d\n", pot_deg);
  Serial.write(buf, write_len);
  delay(200);
}

uint16_t read_potentiometer() {
  uint16_t pot_in = analogRead(POT_PIN);
  uint16_t pot_deg = map(pot_in, 0, 1023, 0, 180);
  return pot_deg;
}

int adjust_servo_speed(int servo_speed) {
  myservo.writeMicroseconds(map(servo_speed,-50 ,50, 1309, 1710));
}
