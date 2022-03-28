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

const pid_constants_t M1_PID = {1,0,0,255};
const pid_constants_t M2_PID = {1,0,0,255};
const pid_constants_t M3_PID = {1,0,0,255};
const pid_constants_t M4_PID = {1,0,0,255};

// motor constants
const uint32_t ENCODER_PULSES_PER_ROTATION = 16*50;
const uint32_t PULSES_PER_CM = ENCODER_PULSES_PER_ROTATION * 3280;
const uint32_t MOTOR_VELOCITY_MAX = 25; // cm/s

// i2c message buffers 
const uint8_t RX_BUF_SIZE = 5;
const uint8_t TX_BUF_SIZE = 5;
static volatile uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint8_t tx_buf[TX_BUF_SIZE];

// current velocity command from CCU
static volatile int16_t x_cmd = 0;
static volatile int16_t y_cmd = 0;
static volatile int16_t r_cmd = 0;


// Globals
static int32_t target_pos[] = {0, 0, 0, 0};
static volatile int32_t global_pos[] = {0, 0, 0, 0};

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

void pid_evaluate(int16_t curr_pos, int16_t target_pos, uint16_t dt, int32_t *prev_error, int32_t *error_integral, pid_constants_t constants, uint8_t *motor_power, dir_t *motor_dir) {
  
  int32_t error = target_pos - curr_pos;
  int32_t dedt = (error - *prev_error)/dt;
  *error_integral += + error*dt;
  
  int32_t control_signal = constants.KP*error + constants.KD*dedt + constants.KI * (*error_integral);
  
  uint32_t control_magnitude = abs(control_signal);
  
  *motor_power = constrain(control_magnitude, 0, constants.UMAX);
  *motor_dir = (control_signal >= 0) ? CW : CCW;

  *prev_error = error;
}

void do_pid(motor_t motor, pid_constants_t constants) {
  // time difference calculations for PID
  static uint32_t prev_time = 0;
  uint32_t curr_time = micros();
  uint32_t dt = curr_time - prev_time;
  prev_time = curr_time;

  set_target(curr_time, dt);

  int32_t pos[NUM_MOTORS];
  // saving local copy in atomic block to prevent changes
  noInterrupts();
    pos[0] = global_pos[0];
    pos[1] = global_pos[1];
    pos[2] = global_pos[2];
    pos[3] = global_pos[3];
  interrupts();

  uint8_t motor_power = 0;
  dir_t motor_dir = BRAKE;

  static int32_t prev_error[NUM_MOTORS];
  static int32_t error_integral[NUM_MOTORS];

  pid_evaluate(pos[0], target_pos[0], dt, &prev_error[0], &error_integral[0], constants, &motor_power, &motor_dir);
  motor_drive(motor, motor_power, motor_dir);

  return;
}

void set_target(uint32_t t, int32_t dt) {

  //64 encoder counts per rotation
  //We are only worried about 16 of them (when ENA is a rising edge)
  //because that's when the interrupts are called

  //Gear-ratio: 50:1
  
  int32_t motor_pos_change[4] = {0, 0, 0, 0}; 

  //mechanum wheels have diameter of 97mm
  //circumferance in meters = (pi*0.097) = 0.03
  //so 1 full rotation = 0.03m of travel

  //1 full rotation /0.03m = how many rotations per meter of travel
  //(1 full rotation / 0.03m) * (encoder pulses / full rotation) = encoder pulses / m

  t = t % 12; // time is in seconds


  //TO DO: change this to RPM
  //evaluating position change in encoder pulses
  //determines how many encoder pulses are needed to reach target velocity

  if(t < 4) {
    // do nothing
  }  else if(t < 8) {
    motor_pos_change[0] = (dt * PULSES_PER_CM) / MOTOR_VELOCITY_MAX;
    motor_pos_change[1] = (dt * PULSES_PER_CM) / MOTOR_VELOCITY_MAX;
    motor_pos_change[2] = (dt * PULSES_PER_CM) / MOTOR_VELOCITY_MAX;
    motor_pos_change[3] = (dt * PULSES_PER_CM) / MOTOR_VELOCITY_MAX;
  }  else{
    motor_pos_change[0] = -(dt * PULSES_PER_CM) / MOTOR_VELOCITY_MAX;
    motor_pos_change[1] = -(dt * PULSES_PER_CM) / MOTOR_VELOCITY_MAX;
    motor_pos_change[2] = -(dt * PULSES_PER_CM) / MOTOR_VELOCITY_MAX;
    motor_pos_change[3] = -(dt * PULSES_PER_CM) / MOTOR_VELOCITY_MAX;
  }  
  
  target_pos[0] += motor_pos_change[0];
  target_pos[1] += motor_pos_change[1];
  target_pos[2] += motor_pos_change[2];
  target_pos[3] += motor_pos_change[3];
}


template <int j>
void readEncoder(){
  int b = digitalRead(motor_array[j].ENCB);
  if(b > 0){
    global_pos[j]++;
  }
  else{
    global_pos[j]--;
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
    bool r_sgn = bitRead(rx_buf[0],0);
    bool y_sgn = bitRead(rx_buf[0],1);
    bool x_sgn = bitRead(rx_buf[0],2);
  
    r_vel_sp = rx_buf[1] * (r_sgn ? -1 : 1);
    y_vel_sp = rx_buf[2] * (y_sgn ? -1 : 1);
    x_vel_sp = rx_buf[3] * (x_sgn ? -1 : 1);
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
