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

void Robot_L298P_function_for_encoder_A1_josdhejlnfsdejlnfdsvzjik();
void Robot_L298P_function_for_encoder_A2_josdhejlnfsdejlnfdsvzjik();
void Robot_L298P_function_for_encoder_B1_josdhejlnfsdejlnfdsvzjik();
void Robot_L298P_function_for_encoder_B2_josdhejlnfsdejlnfdsvzjik();

class test_class {
  public:
    static test_fun() {
      
    }
};

test_class test_object;

class Robot_L298P {
  public:
    bool enc_dat_A1_real,enc_dat_A2_real,enc_dat_B1_real,enc_dat_B2_real;
    bool enc_dat_A1_last,enc_dat_A2_last,enc_dat_B1_last,enc_dat_B2_last;
    long int enc_A, enc_B;

    void motor_A(long int speed) {
      speed = map(constrain(speed, -100, 100),-100,100,-255,255)*reverse_MA;
      analogWrite(L298P_PWMA, abs(speed));
      digitalWrite(L298P_DIRA, speed>0);
    }
    void motor_B(long int speed) {
      speed = map(constrain(speed, -100, 100),-100,100,-255,255)*reverse_MB;
      analogWrite(L298P_PWMB, abs(speed));
      digitalWrite(L298P_DIRB, speed>0);
    }
    void motors(long int speed_a, long int speed_b) {
      Robot_L298P::motor_A(speed_a);
      Robot_L298P::motor_B(speed_b);
    }
    
    void setup() {
      pinMode(L298P_PWMA, OUTPUT);
      pinMode(L298P_PWMB, OUTPUT);
      pinMode(L298P_DIRA, OUTPUT);
      pinMode(L298P_DIRB, OUTPUT);
    
      pinMode(L298P_ENCA1, INPUT);
      pinMode(L298P_ENCA2, INPUT);
      attachPCINT(digitalPinToPCINT(L298P_ENCA1), test_object.test_fun, CHANGE);
      attachPCINT(digitalPinToPCINT(L298P_ENCA2), Robot_L298P_function_for_encoder_A2_josdhejlnfsdejlnfdsvzjik, CHANGE);
      Robot_L298P::enc_dat_A1_real = digitalRead(L298P_ENCA1);
      Robot_L298P::enc_dat_A2_real = digitalRead(L298P_ENCA2);
      Robot_L298P::enc_dat_A1_last = Robot_L298P::enc_dat_A1_real;
      Robot_L298P::enc_dat_A2_last = Robot_L298P::enc_dat_A2_real;
      
      pinMode(L298P_ENCB1, INPUT);
      pinMode(L298P_ENCB2, INPUT);
      attachPCINT(digitalPinToPCINT(L298P_ENCB1), Robot_L298P_function_for_encoder_B1_josdhejlnfsdejlnfdsvzjik, CHANGE);
      attachPCINT(digitalPinToPCINT(L298P_ENCB2), Robot_L298P_function_for_encoder_B2_josdhejlnfsdejlnfdsvzjik, CHANGE);
      Robot_L298P::enc_dat_B1_real = digitalRead(L298P_ENCB1);
      Robot_L298P::enc_dat_B2_real = digitalRead(L298P_ENCB2);
      Robot_L298P::enc_dat_B1_last = Robot_L298P::enc_dat_B1_real;
      Robot_L298P::enc_dat_B2_last = Robot_L298P::enc_dat_B2_real;

      Robot_L298P::enc_A = 0;
      Robot_L298P::enc_B = 0;
      
      Robot_L298P::reverse_MA = 1;
      Robot_L298P::reverse_MB = 1;
      Robot_L298P::reverse_EA = 1;
      Robot_L298P::reverse_EB = 1;
    }

    void reverse_motor_A() { Robot_L298P::reverse_MA = Robot_L298P::reverse_MA==1?-1:1; }
    void reverse_motor_B() { Robot_L298P::reverse_MB = Robot_L298P::reverse_MB==1?-1:1; }
    void reverse_enc_A() { Robot_L298P::reverse_EA = !Robot_L298P::reverse_EA; }
    void reverse_enc_B() { Robot_L298P::reverse_EB = !Robot_L298P::reverse_EB; }

    void encoder_A_update_WARNING_not_using_without_class() {
      if ((Robot_L298P::enc_dat_A1_real!=Robot_L298P::enc_dat_A2_last)!=Robot_L298P::reverse_EA) Robot_L298P::enc_A++;
      else Robot_L298P::enc_A--;
      Robot_L298P::enc_dat_A1_last = Robot_L298P::enc_dat_A1_real;
      Robot_L298P::enc_dat_A2_last = Robot_L298P::enc_dat_A2_real;
    }
    void encoder_B_update_WARNING_not_using_without_class() {
      if ((Robot_L298P::enc_dat_B1_real!=Robot_L298P::enc_dat_B2_last)!=Robot_L298P::reverse_EB) Robot_L298P::enc_B++;
      else Robot_L298P::enc_B--;
      Robot_L298P::enc_dat_B1_last = Robot_L298P::enc_dat_B1_real;
      Robot_L298P::enc_dat_B2_last = Robot_L298P::enc_dat_B2_real;
    }
  private:
    int reverse_MA, reverse_MB; 
    bool reverse_EA, reverse_EB; 
};

Robot_L298P Robot;

void Robot_L298P_function_for_encoder_A1_josdhejlnfsdejlnfdsvzjik() {
  Robot.enc_dat_A1_real = !Robot.enc_dat_A1_real;
  Robot.encoder_A_update_WARNING_not_using_without_class();
}

void Robot_L298P_function_for_encoder_A2_josdhejlnfsdejlnfdsvzjik() {
  Robot.enc_dat_A2_real = !Robot.enc_dat_A2_real;
  Robot.encoder_A_update_WARNING_not_using_without_class();
}

void Robot_L298P_function_for_encoder_B1_josdhejlnfsdejlnfdsvzjik() {
  Robot.enc_dat_B1_real = !Robot.enc_dat_B1_real;
  Robot.encoder_B_update_WARNING_not_using_without_class();
}

void Robot_L298P_function_for_encoder_B2_josdhejlnfsdejlnfdsvzjik() {
  Robot.enc_dat_B2_real = !Robot.enc_dat_B2_real;
  Robot.encoder_B_update_WARNING_not_using_without_class();
}
