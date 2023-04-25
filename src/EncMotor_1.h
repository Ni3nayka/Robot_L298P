/*
   code write for project:
   https://github.com/Ni3nayka/Robot_L298P

   author: Egor Bakay <egor_bakay@inbox.ru> Ni3nayka
   write:  April 2022
   modify: April 2023
*/

#pragma once

#include "PinChangeInterrupt.h"

#define MAX_MOTOR_MANAGEMENT_VALUE   100
#define MAX_MOTOR_REAL_VALUE   255

#define MOTOR_ENC_DT 10

#ifndef enc_motor_1_KP
#define enc_motor_1_KP 0.03
#endif
#ifndef enc_motor_1_KD
#define enc_motor_1_KD 0.3
#endif

#ifndef enc_motor_1_PARROT_CM
#define enc_motor_1_PARROT_CM 157
#endif
#ifndef enc_motor_1_PARROT_ANGLE
#define enc_motor_1_PARROT_ANGLE 25
#endif

class enc_motor_1 {
  public:
    static long int enc;
    static bool enc_dat_A_real,enc_dat_B_real;
    static bool enc_dat_A_last,enc_dat_B_last;
    
    //long int enc;
    void setup(int pwm_pin,int dir_pin,int enc_A_pin,int enc_B_pin) {
      enc_motor_1::pwm_pin = pwm_pin;
      enc_motor_1::dir_pin = dir_pin;
      enc_motor_1::enc_A_pin = enc_A_pin;
      enc_motor_1::enc_B_pin = enc_B_pin;
      
      pinMode(enc_motor_1::pwm_pin, OUTPUT);
      pinMode(enc_motor_1::dir_pin, OUTPUT);
      pinMode(enc_motor_1::enc_A_pin, INPUT);
      pinMode(enc_motor_1::enc_B_pin, INPUT);
      
      attachPCINT(digitalPinToPCINT(enc_motor_1::enc_A_pin), enc_motor_1::Interrupt_enc_A, CHANGE);
      attachPCINT(digitalPinToPCINT(enc_motor_1::enc_B_pin), enc_motor_1::Interrupt_enc_B, CHANGE);
      enc_motor_1::enc_dat_A_real = digitalRead(enc_motor_1::enc_A_pin);
      enc_motor_1::enc_dat_B_real = digitalRead(enc_motor_1::enc_B_pin);
      enc_motor_1::enc_dat_A_last = enc_motor_1::enc_dat_A_real;
      enc_motor_1::enc_dat_B_last = enc_motor_1::enc_dat_B_real;

      enc_motor_1::enc = 0;
      enc_motor_1::reverse_motor_flag = 1;
      enc_motor_1::reverse_enc_flag = 1;
      enc_motor_1::enc_motor_1_time = 0;
      enc_motor_1::last_enc = 0;
      enc_motor_1::motor_speed = 0.0;
      enc_motor_1::e_old = 0;
      enc_motor_1::motor_on_speed = 0;
    }

    
    void reverse_motor() { 
      enc_motor_1::reverse_motor_flag = enc_motor_1::reverse_motor_flag==1?-1:1; 
    }
    void reverse_enc() { 
      enc_motor_1::reverse_enc_flag = !enc_motor_1::reverse_enc_flag; 
    }

    //void 

    void run(long int speed) {
      speed = map(constrain(speed, -MAX_MOTOR_MANAGEMENT_VALUE, MAX_MOTOR_MANAGEMENT_VALUE),
                  -MAX_MOTOR_MANAGEMENT_VALUE,
                  MAX_MOTOR_MANAGEMENT_VALUE,
                  -MAX_MOTOR_REAL_VALUE,MAX_MOTOR_REAL_VALUE
              )*enc_motor_1::reverse_motor_flag;
      analogWrite(enc_motor_1::pwm_pin, abs(speed));
      digitalWrite(enc_motor_1::dir_pin, speed>0);
    }

    void enc_run_speed(double speed) {
      enc_motor_1::motor_speed = speed;
      enc_motor_1::enc_run_update();
    }
    void enc_run_distanse(long int distance) {
      
    }
    void enc_run_update() {
      unsigned long int dt = millis()-enc_motor_1::enc_motor_1_time;
      if (dt>MOTOR_ENC_DT) {
        // calc real_speed => e
        double real_speed = double(enc_motor_1::enc - enc_motor_1::last_enc)/dt;
        enc_motor_1::last_enc = enc_motor_1::enc;
        long int e = (enc_motor_1::motor_speed - real_speed)*100;
        // PID
        long int P = e;
        long int D = e - enc_motor_1::e_old;
        enc_motor_1::e_old = e;
        long int PID = P*enc_motor_1_KP + D*enc_motor_1_KD;
        enc_motor_1::motor_on_speed += PID;
        enc_motor_1::motor_on_speed = constrain(enc_motor_1::motor_on_speed,-MAX_MOTOR_MANAGEMENT_VALUE,MAX_MOTOR_MANAGEMENT_VALUE);
        if (enc_motor_1::motor_on_speed!=0) enc_motor_1::motor_on_speed -= enc_motor_1::motor_on_speed/abs(enc_motor_1::motor_on_speed);
        enc_motor_1::run(enc_motor_1::motor_on_speed);
        // time update
        enc_motor_1::enc_motor_1_time = millis();
      }
    }
    
  private:
    int pwm_pin,dir_pin,enc_A_pin,enc_B_pin;
    int reverse_motor_flag; 
    static bool reverse_enc_flag; 
    unsigned long int enc_motor_1_time;
    long int last_enc, e_old, motor_on_speed;
    double motor_speed;

    static void Interrupt_enc_A() {
      enc_motor_1::enc_dat_A_real = !enc_motor_1::enc_dat_A_real;
      enc_motor_1::encoder_update();
    }
    static void Interrupt_enc_B() {
      enc_motor_1::enc_dat_B_real = !enc_motor_1::enc_dat_B_real;
      enc_motor_1::encoder_update();
    }
    static void enc_motor_1::encoder_update() {
      if ((enc_motor_1::enc_dat_A_real!=enc_motor_1::enc_dat_B_last)!=enc_motor_1::reverse_enc_flag) enc_motor_1::enc++;
      else enc_motor_1::enc--;
      enc_motor_1::enc_dat_A_last = enc_motor_1::enc_dat_A_real;
      enc_motor_1::enc_dat_B_last = enc_motor_1::enc_dat_B_real;
    }
};

long int enc_motor_1::enc = 0;
bool enc_motor_1::enc_dat_A_real = 0,enc_motor_1::enc_dat_B_real = 0;
bool enc_motor_1::enc_dat_A_last = 0,enc_motor_1::enc_dat_B_last = 0;
bool enc_motor_1::reverse_enc_flag = 0;
