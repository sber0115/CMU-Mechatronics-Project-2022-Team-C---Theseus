#include <Stepper.h>

// number of steps on the stepper motor
#define STEPS 1000

// create an instance of the stepper class and specify pins
Stepper stepper(STEPS, 7, 8);

int cur_step = 0;
int input_angle = 0;

void setup() {
  Serial.begin(115200);
  // set the speed of the motor to 30 RPMs
  stepper.setSpeed(50);
}

void loop() {
   if (Serial.available() > 0) {
    // read the incoming byte:
    input_angle = Serial.parseInt();
    Serial.print("Input angle: ");
    Serial.println(input_angle);
  }
  // angle per step
  int input_step = map(input_angle,0,360,0,200);

  // move to the target angle
  stepper.step((input_step - cur_step)*4);

  // save the new current angle
  cur_step = input_step;
}
