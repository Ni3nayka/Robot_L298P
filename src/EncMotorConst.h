/*
   code write for project:
   https://github.com/Ni3nayka/Robot_L298P

   author: Egor Bakay <egor_bakay@inbox.ru> Ni3nayka
   write:  May 2022
   modify: May 2023
*/

#pragma once

#define MAX_MOTOR_MANAGEMENT_VALUE   100
#define MAX_MOTOR_REAL_VALUE   255

#define MOTOR_ENC_DT 10

#ifndef ENC_MOTOR_SPEED_KP
#define ENC_MOTOR_SPEED_KP 0.03
#endif
#ifndef ENC_MOTOR_SPEED_KD
#define ENC_MOTOR_SPEED_KD 0.3
#endif

#ifndef ENC_MOTOR_DISTANSE_KP
#define ENC_MOTOR_DISTANSE_KP 1 // 1
#endif 
#ifndef ENC_MOTOR_DISTANSE_KD
#define ENC_MOTOR_DISTANSE_KD 8 // 8
#endif 
#ifndef ENC_MOTOR_DISTANSE_MAX_E_K
#define ENC_MOTOR_DISTANSE_MAX_E_K 5
#endif 
#ifndef ENC_MOTOR_DISTANSE_RAZGON_PLUS
#define ENC_MOTOR_DISTANSE_RAZGON_PLUS 0.02
#endif 
#ifndef ENC_MOTOR_DISTANSE_KK
#define ENC_MOTOR_DISTANSE_KK 0.4 //0.4
#endif 
#ifndef ENC_MOTOR_DISTANSE_BETWEEN_KP
#define ENC_MOTOR_DISTANSE_BETWEEN_KP 1 //1
#endif 
#ifndef ENC_MOTOR_DISTANSE_END_TIME
#define ENC_MOTOR_DISTANSE_END_TIME 1000
#endif
