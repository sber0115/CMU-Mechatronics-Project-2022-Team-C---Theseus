#include <Wire.h>
#include "theseus.h"

// MCU I2C address
const uint8_t SELF_ADDRESS = 0x02;
// i2c message buffers 
const uint8_t RX_BUF_SIZE = 4;
const uint8_t TX_BUF_SIZE = 4;
volatile uint8_t rx_buf[RX_BUF_SIZE];
volatile uint8_t tx_buf[TX_BUF_SIZE];
// current velocity command from CCU
volatile int16_t x_cmd = 0;
volatile int16_t y_cmd = 0;
volatile int16_t r_cmd = 0;

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
 */           // IN1~, IN2, ENCA*, ENCB*              
motor_t M1 = {3,4,20,21};
motor_t M2 = {5,8,16,17}; 
motor_t M3 = {6,7,14,15}; 
motor_t M4 = {9,10,11,12};  

// current velocity setpoints 
int32_t x_vel_sp = 0; // +x -> FWD,   -x -> BACK
int32_t y_vel_sp = 0; // +y -> RIGHT, -y -> LEFT
int32_t r_vel_sp = 0; // +r -> CW,    -r -> CCW

void setup() {
  Serial.begin(115200);

  motor_initialize(M1);
  motor_initialize(M2);
  motor_initialize(M3);
  motor_initialize(M4);  

  Wire.begin(SELF_ADDRESS);
  Wire.onRequest(event_tx_msg);
  Wire.onReceive(event_rx_msg);
}

void loop() {
  Serial.println("FWD");
  move(FWD, 150);
  delay(1000);
  Serial.println("BACK");
  move(BACK, 150);
  delay(1000);
  Serial.println("LEFT");
  move(LEFT, 150);
  delay(1000);
  Serial.println("RIGHT");
  move(RIGHT, 150);
  delay(1000);
  Serial.println("ROTATE CW");
  move(ROT_CW, 150);
  delay(1000);
  Serial.println("ROTATE CCW");
  move(ROT_CCW, 150);
  delay(1000);
  Serial.println("STOP");
  move(STOP, 0);
  delay(5000);  
}

void read_motor_command(uint8_t *rx_buf) {
  if(rx_buf == NULL) {
    Serial.println("ERROR: Null pointer rx_buf(). No dir ");
  } else {
    bool r_sgn = bitRead(rx_buf[0],0);
    bool y_sgn = bitRead(rx_buf[0],1);
    bool x_sgn = bitRead(rx_buf[0],2);
  
    r_vel_sp = rx_buf[1] * (r_sgn ? -1 : 1);
    y_vel_sp = rx_buf[2] * (y_sgn ? -1 : 1);
    x_vel_sp = rx_buf[3] * (x_sgn ? -1 : 1);
  }
  return;
}

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
    case ROT_CW:
      motor_drive(M1, speed, CW);
      motor_drive(M2, speed, CW);
      motor_drive(M3, speed, CW);
      motor_drive(M4, speed, CW);
      break;
    case ROT_CCW:
      motor_drive(M1, speed, CCW);
      motor_drive(M2, speed, CCW);
      motor_drive(M3, speed, CCW);
      motor_drive(M4, speed, CCW);
      break;
    case STOP: 
      motor_drive(M1, 0, BRAKE);
      motor_drive(M2, 0, BRAKE);
      motor_drive(M3, 0, BRAKE);
      motor_drive(M4, 0, BRAKE);
      break;
  }
}

/*
 * motor_initialize(): Initializes motor GPIO pins
 *   motor_t motor   - DC motor to initialize
 */
void motor_initialize(motor_t motor) {
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
  switch(direction) {
    case CW: 
      analogWrite(motor.IN1, speed);
      digitalWrite(motor.IN2, HIGH);
      break;
    case CCW:
      analogWrite(motor.IN1, speed);
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
  uint8_t bytes_read = 0;
  while((Wire.available() != 0) && (bytes_read < RX_BUF_SIZE)) {
    rx_buf[bytes_read] = Wire.read();
    bytes_read++;
  }
  return;
}

