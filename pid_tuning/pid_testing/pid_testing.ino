#include <Wire.h>
#include "theseus.h"

// MCU I2C address
const uint8_t SELF_ADDRESS = 0x08;

// number of motors
const uint8_t NMOTORS = 4;

//number of encoder pulses per full rotation
const uint32_t pulses_per_turn = 16*50;

// i2c message buffers 
const uint8_t RX_BUF_SIZE = 5;
const uint8_t TX_BUF_SIZE = 5;
volatile uint8_t rx_buf[RX_BUF_SIZE];
volatile uint8_t tx_buf[TX_BUF_SIZE];

float target_f[] = {0, 0, 0, 0};
long target[] = {0, 0, 0, 0};

// current velocity command from CCU
volatile int16_t x_cmd = 0;
volatile int16_t y_cmd = 0;
volatile int16_t r_cmd = 0;

// Globals
long prevT = 0;
volatile int posi[] = {0, 0, 0, 0};

// PID class instances
SimplePID pid[NMOTORS];

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

  motor_array[0] = {56, 57, 54, 55, 11};
  motor_array[1] = {61, 60, 58, 59, 10};
  motor_array[2] = {24, 25, 22, 23, 2};
  motor_array[3] = {28, 29, 26, 27, 3}; 


  for (int k = 0; k < NMOTORS; k++) {
    motor_initialize(motor_array[k]);
    pid[k].setParams(1, 0, 0, 150);
  }

  Wire.begin(SELF_ADDRESS);
  Wire.onRequest(event_tx_msg);
  Wire.onReceive(event_rx_msg);

  attachInterrupt(digitalPinToInterrupt(motor_array[0].ENCA),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(motor_array[1].ENCA),readEncoder<1>,RISING);
  attachInterrupt(digitalPinToInterrupt(motor_array[2].ENCA),readEncoder<3>,RISING);
  attachInterrupt(digitalPinToInterrupt(motor_array[3].ENCA),readEncoder<4>,RISING);
  
  //Serial.println("target pos");
}

void loop() {
  
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  setTarget(currT/1.0e6,deltaT);
  
  // Read the position in an atomic block to avoid a potential misread
  int pos[NMOTORS];
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  }
  
  // loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    // evaluate the control signal
    pid[k].evalu(pos[k], target[k], deltaT, pwr, dir);
    // signal the motor
    setMotor(dir, pwr, motor_array[k].EN, motor_array[k].IN1, motor_array[k].IN2);
  }

  /*
  for(int k = 0; k < NMOTORS; k++){
    Serial.print("Target:");
    Serial.print(target[k]);
    Serial.print(" ");
    Serial.print("Pos: ");
    Serial.print(pos[k]);
  }
  Serial.println(" ");
  */

  Serial.print("Target 0: ");
  Serial.print(target[0]);
  Serial.print(" ");
  Serial.print("Pos: ");
  Serial.print(pos[0]);
  Serial.println(" ");
  
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

/*
 * motor_initialize(): Initializes motor GPIO pins
 *   motor_t motor   - DC motor to initialize
 */
void motor_initialize(motor_t motor) {
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

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm, pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}


void setTarget(float t, float deltat){

  //64 encoder counts per rotation
  //We are only worried about 16 of them (when ENA is a rising edge)
  //because that's when the interrupts are called

  //Gear-ratio: 50:1
  
  float positionChange[4] = {0.0, 0.0, 0.0, 0.0};

  //There are 16*50 pulses per rotation of the output shaft
  float pulsesPerTurn = 16*50; 

  //mechanum wheels have diameter of 97mm
  //circumferance in meters = (pi*0.097) = 0.03
  //so 1 full rotation = 0.03m of travel

  //1 full rotation /0.03m = how many rotations per meter of travel
  //(1 full rotation / 0.03m) * (encoder pulses / full rotation) = encoder pulses / m
  float pulsesPerMeter = pulsesPerTurn*32.8;

  t = fmod(t,12); // time is in seconds

  //the current program goes 1 meter every 4 seconds
  float velocity = 0.25; // m/s

  //TO DO: change this to RPM
  
  //evaluating position change in encoder pulses
  //determines how many encoder pulses are needed to reach target velocity

  if(t < 4){
  }
  else if(t < 8){
    for(int k = 0; k < 4; k++){ 
      positionChange[k] = velocity*deltat*pulsesPerMeter;
    }
  }
  else{
    for(int k = 0; k < 4; k++){ 
      positionChange[k] = -velocity*deltat*pulsesPerMeter; 
    } 
  }  

  for(int k = 0; k < 4; k++){
    target_f[k] = target_f[k] + positionChange[k];
  }
  
  target[0] = (long) target_f[0];
  target[1] = (long) target_f[1];
  target[2] = (long) target_f[2];
  target[3] = (long) target_f[3];
}


template <int j>
void readEncoder(){
  int b = digitalRead(motor_array[j].ENCB);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
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
    //Serial.println("PRINTING RX");
    //Serial.print(rx_buf[bytes_read]);
    bytes_read++;
  }
  //Serial.println();

  Serial.print("Read message: ");
  for (int i = 0; i < RX_BUF_SIZE; i++){
    Serial.print(rx_buf[i]);
  }
  Serial.println();
  
  memcpy(tx_buf, rx_buf, sizeof(rx_buf[0]*RX_BUF_SIZE));

  Wire.write((uint8_t *) &tx_buf, TX_BUF_SIZE);

  return;
}
