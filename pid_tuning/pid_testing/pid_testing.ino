#include "theseus.h"

// MCU I2C address
const uint8_t SELF_ADDRESS = 0x08;

// motor pins
const uint8_t NUM_MOTORS = 4;
motor_t M1 = {56, 57, 55, 54, 11};
motor_t M2 = {61, 60, 58, 59, 10};
motor_t M3 = {24, 25, 22, 23, 0};
motor_t M4 = {28, 29, 26, 27, 1}; 

Encoder M1_encoder(M1.ENCA, M1.ENCB); 
Encoder M2_encoder(M2.ENCA, M2.ENCB); 
Encoder M3_encoder(M3.ENCA, M3.ENCB); 
Encoder M4_encoder(M4.ENCA, M4.ENCB); 

// pid
double M1_in, M1_out, M1_sp;
double M2_in, M2_out, M2_sp;
double M3_in, M3_out, M3_sp;
double M4_in, M4_out, M4_sp;
PID M1_pid(&M1_in, &M1_out, &M1_sp, 1, 0, 0, DIRECT);
PID M2_pid(&M2_in, &M2_out, &M2_sp, 1, 0, 0, DIRECT);
PID M3_pid(&M3_in, &M3_out, &M3_sp, 1, 0, 0, DIRECT);
PID M4_pid(&M4_in, &M4_out, &M4_sp, 1, 0, 0, DIRECT);


// motor constants
const uint32_t TICKS_PER_ROTATION = 16*50;
const uint32_t PULSES_PER_CM = TICKS_PER_ROTATION * 3280;
const uint32_t MOTOR_VELOCITY_MAX = 25; // cm/s

// i2c message buffers 
const uint8_t RX_BUF_SIZE = 5;
const uint8_t TX_BUF_SIZE = 5;
static volatile uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint8_t tx_buf[TX_BUF_SIZE];

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
 */           // IN1~, IN2, ENCA*, ENCB*, EN*   

void setup() {
  Serial.begin(115200);

  motor_initialize(M1);
  motor_initialize(M2);
  motor_initialize(M3);
  motor_initialize(M4);

  M1_encoder.write(0);
  M2_encoder.write(0);
  M3_encoder.write(0);
  M4_encoder.write(0);

  Wire.begin(SELF_ADDRESS);
  Wire.onRequest(event_tx_msg);
  Wire.onReceive(event_rx_msg);
}

void loop() {
  

  
}

void do_pid() {

  M1_in = M1_encoder.read() % TICKS_PER_ROTATION;
  M2_in = M2_encoder.read() % TICKS_PER_ROTATION;
  M3_in = M3_encoder.read() % TICKS_PER_ROTATION;
  M4_in = M4_encoder.read() % TICKS_PER_ROTATION;

  M1_pid.Compute();
  M2_pid.Compute();
  M3_pid.Compute();
  M4_pid.Compute();

  motor_drive(M1, constrain(abs(M1_out), 0, 255), (M1_out < 0 ? CCW : CW));
  motor_drive(M2, constrain(abs(M2_out), 0, 255), (M2_out < 0 ? CCW : CW));
  motor_drive(M3, constrain(abs(M3_out), 0, 255), (M3_out < 0 ? CCW : CW));
  motor_drive(M4, constrain(abs(M4_out), 0, 255), (M4_out < 0 ? CCW : CW));

  return;
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
  pinMode(motor.EN, OUTPUT);  
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
 * read_motor_command() - Parses I2C message into command for shipbot
 * 
 *
 */
void read_motor_command(uint8_t *rx_buf) {
  if(rx_buf == NULL) {
    Serial.println("ERROR: Null pointer rx_buf(). No dir ");
  } else {
    // tbd
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

  Serial.println("RECEIVED I2C message\n");
  
  uint8_t bytes_read = 0;
  while((Wire.available() != 0) && (bytes_read < RX_BUF_SIZE)) {
    rx_buf[bytes_read] = Wire.read();
    bytes_read++;
  }
  
  // memcpy(tx_buf, rx_buf, sizeof(rx_buf[0]*RX_BUF_SIZE));
  // Wire.write((uint8_t *) &tx_buf, TX_BUF_SIZE);
  return;
}
