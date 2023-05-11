/*
   code write for project:
   https://github.com/Ni3nayka/Robot_L298P

   author: Egor Bakay <egor_bakay@inbox.ru> Ni3nayka
   write:  April 2022
   modify: May 2023
*/

#pragma once

#include "PinChangeInterrupt.h"
#include "EncMotorConst.h"

class enc_motor_1 {
  public:
    // variables
    static long int enc;
    static bool enc_dat_A_real,enc_dat_B_real;
    static bool enc_dat_A_last,enc_dat_B_last;
    // setup
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
      enc_motor_1::enc_motor_1_end_distanse = 0;
      enc_motor_1::last_enc = 0;
      enc_motor_1::motor_speed = 0.0;
      enc_motor_1::e_old = 0;
      enc_motor_1::motor_on_speed = 0;
      enc_motor_1::motor_distanse = 0;
      //enc_motor_1::motor_start_distanse = 0;
      enc_motor_1::motor_speed_distance_mode = 0;
      enc_motor_1::end_distanse = 0;
    }
    void reverse_motor() { 
      enc_motor_1::reverse_motor_flag = enc_motor_1::reverse_motor_flag==1?-1:1; 
    }
    void reverse_enc() { 
      enc_motor_1::reverse_enc_flag = !enc_motor_1::reverse_enc_flag; 
    }
    // motor
    void run(long int speed) {
      speed = map(constrain(speed, -MAX_MOTOR_MANAGEMENT_VALUE, MAX_MOTOR_MANAGEMENT_VALUE),
                  -MAX_MOTOR_MANAGEMENT_VALUE,
                  MAX_MOTOR_MANAGEMENT_VALUE,
                  -MAX_MOTOR_REAL_VALUE,MAX_MOTOR_REAL_VALUE
              )*enc_motor_1::reverse_motor_flag;
      analogWrite(enc_motor_1::pwm_pin, abs(speed));
      digitalWrite(enc_motor_1::dir_pin, speed>0);
    }
    // enc motor
    void enc_run_speed(double speed) {
      enc_motor_1::motor_speed_distance_mode = 0;
      enc_motor_1::motor_speed = speed;
      enc_motor_1::motor_distanse = 0;
      enc_motor_1::enc_run_update();
    }
    void enc_run_distanse(long int distance) {
      enc_motor_1::motor_speed_distance_mode = 1;
      enc_motor_1::motor_speed = 0;
      enc_motor_1::motor_distanse = distance;
      enc_motor_1::enc = 0;
      enc_motor_1::distanse_razgon = 0.0;
      enc_motor_1::enc_run_update();
    }
    bool end_run_distanse() {
      return enc_motor_1::end_distanse;
    }
    void enc_run_update(long int e_between_motor=0) {
      long int dt = millis()-enc_motor_1::enc_motor_1_time;
      if (dt>MOTOR_ENC_DT) {
        if (enc_motor_1::motor_speed_distance_mode) {
          long int e = (enc_motor_1::motor_distanse - enc_motor_1::enc)/dt;
//          Serial.print(e);
          if (abs(e)>ENC_MOTOR_DISTANSE_MAX_E_END)  {
            enc_motor_1::enc_motor_1_end_distanse = millis(); 
            enc_motor_1::end_distanse = 0;
          }
          else if (millis() - enc_motor_1::enc_motor_1_end_distanse < ENC_MOTOR_DISTANSE_END_TIME) {
            enc_motor_1::end_distanse = 1;
          }
          if (!enc_motor_1::end_distanse) {
            long int P = e;
            long int K = constrain(e,-ENC_MOTOR_DISTANSE_MAX_E_K,ENC_MOTOR_DISTANSE_MAX_E_K);
            K = K*K*K;
            long int D = e - enc_motor_1::e_old;
            enc_motor_1::e_old = e;
            long int PID = P*ENC_MOTOR_DISTANSE_KP + D*ENC_MOTOR_DISTANSE_KD + K*ENC_MOTOR_DISTANSE_KK;
            PID = constrain(PID,-MAX_MOTOR_MANAGEMENT_VALUE,MAX_MOTOR_MANAGEMENT_VALUE);
            PID *= enc_motor_1::distanse_razgon;
            if (enc_motor_1::distanse_razgon<1.0) {
              enc_motor_1::distanse_razgon += ENC_MOTOR_DISTANSE_RAZGON_PLUS;
            }
            e_between_motor = e_between_motor*ENC_MOTOR_DISTANSE_BETWEEN_KP;
            PID += constrain(e_between_motor,-MAX_MOTOR_MANAGEMENT_VALUE,MAX_MOTOR_MANAGEMENT_VALUE);
//            Serial.print(" ");
//            Serial.print(PID);
            enc_motor_1::run(PID);
          }
          else enc_motor_1::run(0);
//          Serial.println();
        }
        else {
          // calc real_speed => e
          double real_speed = double(enc_motor_1::enc - enc_motor_1::last_enc)/dt;
          enc_motor_1::last_enc = enc_motor_1::enc;
          long int e = (enc_motor_1::motor_speed - real_speed)*100;
          // PID
          long int P = e;
          long int D = e - enc_motor_1::e_old;
          enc_motor_1::e_old = e;
          long int PID = P*ENC_MOTOR_SPEED_KP + D*ENC_MOTOR_SPEED_KD;
          enc_motor_1::motor_on_speed += PID;
          enc_motor_1::motor_on_speed = constrain(enc_motor_1::motor_on_speed,-MAX_MOTOR_MANAGEMENT_VALUE,MAX_MOTOR_MANAGEMENT_VALUE);
          if (enc_motor_1::motor_on_speed!=0) enc_motor_1::motor_on_speed -= enc_motor_1::motor_on_speed/abs(enc_motor_1::motor_on_speed);
          enc_motor_1::run(enc_motor_1::motor_on_speed);
        }
        // time update
        enc_motor_1::enc_motor_1_time = millis();
      }
    }
    
  private:
    // basic
    int pwm_pin,dir_pin,enc_A_pin,enc_B_pin;
    int reverse_motor_flag; 
    static bool reverse_enc_flag; 
    // speed and distance enc
    unsigned long int enc_motor_1_time,enc_motor_1_end_distanse;
    long int last_enc, e_old, motor_on_speed;
    double motor_speed,distanse_razgon;
    long int motor_distanse;//,motor_start_distanse;
    bool motor_speed_distance_mode; // 0 - speed, 1 - distance
    bool end_distanse; 

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
