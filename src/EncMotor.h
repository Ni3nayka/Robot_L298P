/*
   code write for project:
   https://github.com/Ni3nayka/Robot_L298P

   author: Egor Bakay <egor_bakay@inbox.ru> Ni3nayka
   write:  April 2022
   modify: April 2023
*/

#pragma once

#include "PinChangeInterrupt.h"
// https://learn.microsoft.com/ru-ru/cpp/cpp/storage-classes-cpp?view=msvc-170

#define MAX_MOTOR_MANAGEMENT_VALUE   100
#define MAX_MOTOR_REAL_VALUE   255

class enc_motor {
  public:
    static long int enc;
    
    int pwm_pin,dir_pin,enc_A_pin,enc_B_pin;
    static bool enc_dat_A_real,enc_dat_B_real;
    static bool enc_dat_A_last,enc_dat_B_last;
    int reverse_motor; 
    static bool reverse_enc; 
    
    //long int enc;
    void setup(int pwm_pin,int dir_pin,int enc_A_pin,int enc_B_pin) {
      enc_motor::pwm_pin = pwm_pin;
      enc_motor::dir_pin = dir_pin;
      enc_motor::enc_A_pin = enc_A_pin;
      enc_motor::enc_B_pin = enc_B_pin;
      
      pinMode(enc_motor::pwm_pin, OUTPUT);
      pinMode(enc_motor::dir_pin, OUTPUT);
      pinMode(enc_motor::enc_A_pin, INPUT);
      pinMode(enc_motor::enc_B_pin, INPUT);
      
      attachPCINT(digitalPinToPCINT(enc_motor::enc_A_pin), enc_motor::Interrupt_enc_A, CHANGE);
      attachPCINT(digitalPinToPCINT(enc_motor::enc_B_pin), enc_motor::Interrupt_enc_B, CHANGE);
      enc_motor::enc_dat_A_real = digitalRead(enc_motor::enc_A_pin);
      enc_motor::enc_dat_B_real = digitalRead(enc_motor::enc_B_pin);
      enc_motor::enc_dat_A_last = enc_motor::enc_dat_A_real;
      enc_motor::enc_dat_B_last = enc_motor::enc_dat_B_real;

      enc_motor::enc = 0;
      
      enc_motor::reverse_motor = 1;
      enc_motor::reverse_enc = 1;
      
    }

    void motor(long int speed) {
      speed = map(constrain(speed, -MAX_MOTOR_MANAGEMENT_VALUE, MAX_MOTOR_MANAGEMENT_VALUE),
                  -MAX_MOTOR_MANAGEMENT_VALUE,
                  MAX_MOTOR_MANAGEMENT_VALUE,
                  -MAX_MOTOR_REAL_VALUE,MAX_MOTOR_REAL_VALUE
              )*enc_motor::reverse_motor;
      analogWrite(enc_motor::pwm_pin, abs(speed));
      digitalWrite(enc_motor::dir_pin, speed>0);
    }

//    long int get_enc() {
//      return enc_motor::enc;
//    }
  private:

    static void Interrupt_enc_A() {
      enc_motor::enc_dat_A_real = !enc_motor::enc_dat_A_real;
      enc_motor::encoder_update();
    }
    static void Interrupt_enc_B() {
      enc_motor::enc_dat_B_real = !enc_motor::enc_dat_B_real;
      enc_motor::encoder_update();
    }
    static void enc_motor::encoder_update() {
      if ((enc_motor::enc_dat_A_real!=enc_motor::enc_dat_B_last)!=enc_motor::reverse_enc) enc_motor::enc++;
      else enc_motor::enc--;
      enc_motor::enc_dat_A_last = enc_motor::enc_dat_A_real;
      enc_motor::enc_dat_B_last = enc_motor::enc_dat_B_real;
    }
};

long int enc_motor::enc = 0;
bool enc_motor::enc_dat_A_real = 0,enc_motor::enc_dat_B_real = 0;
bool enc_motor::enc_dat_A_last = 0,enc_motor::enc_dat_B_last = 0;
bool enc_motor::reverse_enc = 0;
