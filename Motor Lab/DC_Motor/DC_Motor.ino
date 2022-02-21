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
  motor_pos = motor.read();
  int32_t next_pos = pid.compute(motor_pos);
  motor_drive(100, ((motor_pos - next_pos) > 0) ? CCW : CW);
  Serial.println("[" + String(millis() % 10000) + "]" + " Encoder Loc: " + String(motor_pos) + "->" + String(next_pos));
  pid.setpoint(128);
  delay(10); 
}

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
