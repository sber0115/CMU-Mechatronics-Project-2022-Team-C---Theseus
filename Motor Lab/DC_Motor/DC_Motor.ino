#include <util/atomic.h>

//push button (will toggle step_en_state)
const uint16_t BUTTON_PIN = 13;
uint16_t button_state = HIGH;
uint16_t last_button_state = HIGH;

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
uint16_t step_en_state = 1;
const uint16_t STEPS_PER_REV = 200;
uint32_t curr_step_angle = 0;
int curr_step_count = 0;
uint16_t step_dir = 0; //0 for CW, 1 for CCW

int target_angle = 0;

/****************************************/

void setup() {
  Serial.begin(115200); // Serial for Debugging

  //Stepper motor setup
  pinMode(STEP_DIR_PIN, OUTPUT);
  pinMode(STEP_EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  //Push button setup
  pinMode(BUTTON_PIN, INPUT);

}

void loop() {
  //digitalWrite(STEP_EN_PIN, step_en_state);

  //delete afterwards
  digitalWrite(STEP_EN_PIN, LOW);

  uint16_t button_reading = digitalRead(BUTTON_PIN);
  //Serial.println("Reading: " + String(button_reading));
  
  //enable_stepper(button_reading);

  stepper_rotate(step_en_state, 180);
  
  Serial.println("Reading: " + String(button_reading) + " Saved State: " + String(step_en_state)); 
  
  delay(50);

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

      // only enable the stepper if the new button state is HIGH
      if (button_state == LOW) {
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

  digitalWrite(STEP_DIR_PIN, LOW);
  
  curr_step_angle = map(curr_step_count,0,200,0,360);

  while (abs(curr_step_angle - target_angle) > 2){
    Serial.println("Step Count: " + String(curr_step_count) + " Step Angle: " + String(curr_step_angle) 
                   + " Target Angle: " + String(target_angle) + " Step State: " + String(step_en_state));
    
    if (step_dir) {
      curr_step_count--;
    }else{
      curr_step_count++;
    }
    
    //curr_step_count = (step_dir) ? curr_step_count-- : curr_step_count++; 
    curr_step_angle = map(curr_step_count,0,200,0,360);
    
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000);
  }
  return;
}
