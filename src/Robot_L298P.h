/*
   code write for project:
   https://github.com/Ni3nayka/Robot_L298P

   author: Egor Bakay <egor_bakay@inbox.ru> Ni3nayka
   write:  February 2022
   modify: April 2023
*/

#pragma once

#include "PinChangeInterrupt.h"
#include "Pins.h"
#include "EncMotor.h"
#include "EncMotor_1.h"

#ifndef ENC_MOTOR_PARROT_CM
#define ENC_MOTOR_PARROT_CM 157
#endif
#ifndef ENC_MOTOR_PARROT_ANGLE
#define ENC_MOTOR_PARROT_ANGLE 25
#endif

class Robot_L298P {
  public:
    enc_motor motor_A;
    enc_motor_1 motor_B;
    
    void setup() {
      Robot_L298P::motor_A.setup(L298P_PWMA,L298P_DIRA,L298P_ENCA1,L298P_ENCA2);
      Robot_L298P::motor_B.setup(L298P_PWMB,L298P_DIRB,L298P_ENCB1,L298P_ENCB2);
    }
    void motors(long int speed_a, long int speed_b) {
      Robot_L298P::motor_A.run(speed_a);
      Robot_L298P::motor_B.run(speed_b);
    }
    void enc_run_update() {
      long int d = Robot_L298P::motor_A.enc-Robot_L298P::motor_B.enc;
      Robot_L298P::motor_A.enc_run_update(-d);
      Robot_L298P::motor_B.enc_run_update(-d);    
    }
  

//    void reverse_motor_A() { Robot_L298P::motor_A.reverse_motor = Robot_L298P::motor_A.reverse_motor==1?-1:1; }
//    void reverse_motor_B() { Robot_L298P::motor_B.reverse_motor = Robot_L298P::motor_B.reverse_motor==1?-1:1; }
//    void reverse_enc_A() { Robot_L298P::motor_A.reverse_enc = !Robot_L298P::motor_A.reverse_enc; }
//    void reverse_enc_B() { Robot_L298P::motor_B.reverse_enc = !Robot_L298P::motor_B.reverse_enc; }
    
//  private:
//    int reverse_MA, reverse_MB; 
//    bool reverse_EA, reverse_EB; 
};

Robot_L298P Robot;
