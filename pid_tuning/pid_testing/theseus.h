#include "Arduino.h"
#include <util/atomic.h>


#ifndef theseus_h
#define theseus_h

  struct motor_t{
    uint8_t IN1;
    uint8_t IN2;
    uint8_t ENCA;
    uint8_t ENCB;
    uint8_t EN;
  };

  struct motor_t motor_array[4];
  
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

  enum msg_t {
    TEST=0,
    MOVE=1
  };

  struct msg_test {
    uint8_t id;
    uint8_t data[3];
  };

  struct msg_move_t {
    uint8_t dir;
    uint8_t x_vel;
    uint8_t y_vel;
    uint8_t r_vel;
  };


// A class to compute the control signal
class SimplePID{
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage

  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    // error
    int e = target - value;
  
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
  
    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    }
  
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
  
    // store previous error
    eprev = e;
  }
  
};


#endif
