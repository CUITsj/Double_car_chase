#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "common.h"
#include "include.h"

#define MOTOR_FTM FTM3
#define MOTOR1_CH FTM_CH2 //左轮前
#define MOTOR2_CH FTM_CH6 //左轮后  
#define MOTOR3_CH FTM_CH4 //右轮前
#define MOTOR4_CH FTM_CH3 //右轮后 
#define MOTOR_HZ 10000

extern int16 MOTOR1_speed;
extern int16 MOTOR2_speed;

extern int Set_speed;

extern int MOTOR1_DUTY;
extern int MOTOR2_DUTY;
extern int MOTOR3_DUTY;
extern int MOTOR4_DUTY;

extern uint8 color;

void MOTOR_measure();
void MOTOR_pid(int16 gsp);
void stop_pid();

#endif