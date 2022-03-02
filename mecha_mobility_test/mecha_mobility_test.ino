#include <Wire.h>
#include "theseus.h"

// mobility controller's I2C address
const uint8_t SELF_ADDRESS = 0x02;

// motor driver pins
motor_t M1 = {2,3,4,A0,A1};
motor_t M2 = {2,3,4,A0,A1}; // TDB
motor_t M3 = {2,3,4,A0,A1}; // TDB
motor_t M4 = {2,3,4,A0,A1}; // TDB

const uint8_t RX_BUF_SIZE = 4;
const uint8_t TX_BUF_SIZE = 4;
volatile uint8_t rx_buf[RX_BUF_SIZE];
volatile uint8_t tx_buf[TX_BUF_SIZE];

void setup() {
  Serial.begin(115200);

  motor_initialize(M1);

  Wire.begin(SELF_ADDRESS);
  Wire.onRequest(event_tx_msg);
  Wire.onReceive(event_rx_msg);
  

}

void loop() {
  motor_drive(M1, 200, CCW);
  delay(1000);
}


/*
 * move(): 
 *
 *
 *    (front)
 *  M1       M3
 *
 *
 *  M2       M4
 *    (back )
 */
void move(move_t directive, uint8_t speed) {
  switch(directive) {
    case FWD:
      motor_drive(M1, speed, CW);
      motor_drive(M2, speed, CW);
      motor_drive(M3, speed, CCW);
      motor_drive(M4, speed, CCW);
      break;
    case BACK:
      motor_drive(M1, speed, CCW);
      motor_drive(M2, speed, CCW);
      motor_drive(M3, speed, CW);
      motor_drive(M4, speed, CW);
      break;
    case LEFT:
      motor_drive(M1, speed, CCW);
      motor_drive(M2, speed, CW);
      motor_drive(M3, speed, CCW);
      motor_drive(M4, speed, CW);
      break;
    case RIGHT:
      motor_drive(M1, speed, CW);
      motor_drive(M2, speed, CCW);
      motor_drive(M3, speed, CW);
      motor_drive(M4, speed, CCW);
      break;
  }
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
/*
 * motor_drive(): Spins specified motors in direction/speed provided
 *   motor_t motor   - DC motor to turn
 *   uint8_t speed   - Motor speed (0-255)
 *   dir_t direction - Motor direction (CW, CCW, BRAKE)
 */
void motor_drive(motor_t motor, uint8_t speed, dir_t direction) {
  analogWrite(motor.IN1, speed);
  switch(direction) {
    case CW: 
      digitalWrite(motor.EN, HIGH);
      digitalWrite(motor.IN2, HIGH);
      break;
    case CCW:
      digitalWrite(motor.EN, HIGH);
      digitalWrite(motor.IN2, LOW);
      break;
    case BRAKE:
      digitalWrite(motor.IN1, LOW);
      digitalWrite(motor.IN2, LOW);   
      break; 
    default: 
      Serial.println("ERROR STATE: motor_drive(). Braking...");
      digitalWrite(motor.IN1, LOW);
      digitalWrite(motor.IN2, LOW);   
      break;  
  }
  return;
}

/*
 * event_tx_msg(): Event handler when I2C message requested by master
 */
void event_tx_msg() {
  Wire.write((uint8_t *) &tx_buf, TX_BUF_SIZE);
  return;
}

/*
 * event_rx_msg(): Event handler when I2C message received from master
 */
void event_rx_msg(int num_bytes) {
  uint8_t i = 0;
  while((Wire.available() != 0) && (i < RX_BUF_SIZE)) {
    rx_buf[i] = Wire.read();
    i++;
  }
  return;
}

