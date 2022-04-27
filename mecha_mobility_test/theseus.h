#include "Arduino.h"

#ifndef theseus_h
#define theseus_h

  struct motor_t {
    uint8_t EN;
    uint8_t IN1;
    uint8_t IN2;
    uint8_t ENCA;
    uint8_t ENCB;
  };

  struct pid_params_t {
    int32_t KP;
    int32_t KI;
    int32_t KD;
    int32_t BIAS;
    uint8_t UMIN;
    uint8_t UMAX;
  };

  enum move_t {
    FWD=0,
    BACK=1,
    LEFT=2,
    RIGHT=3,
    ROT_CW=4,
    ROT_CCW=5,
    STOP=6
  };
  
  enum dir_t {
    CW=1,
    CCW=2,
    BRAKE=0
  };

  struct directive_t {
    uint8_t speed;
    dir_t direction;
  };

  struct msg_test {
    uint8_t id;
    uint8_t data[3];
  };

    enum msg_t {
    TEST=0,
    MOVE=1
  };

  struct msg_move_t {
    uint8_t dir;
    uint8_t x_vel;
    uint8_t y_vel;
    uint8_t r_vel;
  };


#endif
