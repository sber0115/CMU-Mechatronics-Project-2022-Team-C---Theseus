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

  enum dir_t {
    CW,
    CCW,
    BRAKE
  };

  enum move_t {
    FWD=0,
    BACK=1,
    LEFT=2,
    RIGHT=3,
    FWD_L=4,
    FWD_R=5,
    BACK_L=6,
    BACK_R=7,
    ROTATE_CW=8,
    ROTATE_CCW=9,
    STOP=10
  };

  enum msg_t {
    TEST=0,
    MOVE=1
  };

  struct msg_test {
    uint8_t id;
    uint8_t data[2];
  };

  struct msg_move_t {
    uint8_t id;
    uint8_t directive;
    uint8_t speed;
  };

#endif