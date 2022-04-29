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

// for a 16 bit signed int, in second half of storage if abs(counts) > COUNT_OVERFLOW
#define COUNT_OVERFLOW 16374

// assuming max 2 m/s with 100mm wheels and 360 counts / rotation and 1 second between function calls ~ 1080 counts
// the smaller the number, the more likely we are to catch a reset by the more likely it is a false positive
#define COUNT_RESET 200

// total size of int
#define INT_MAX 65535

// get sign of a number
#define sign(a) (min(1, max(-1, a)))
