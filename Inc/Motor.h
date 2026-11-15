#ifndef __MOTOR_H__
#define __MOTOR_H__



#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "usart.h"


									

void Motor_Init(void);//电机初始化函数
void Motor_Left_SetSpeed(float Setspeed);//左轮运动
void Motor_Right_SetSpeed(float Setspeed);//右轮运动
void Motor_Go_SetSpeed(float Setspeed1,float Setspeed2);//小车运动函数
void Motor_Read_MotorSpeed(float *Setspeed1, float *Setspeed2);//读取车轮速度
void Motor_Read_MotorMileage(float Time,float *Mileage);
void Motor_PID_Go_SetSpeed(float Motorspeed1, float Motorspeed2);//PID速度环
void Motor_PID_Up_SetSpeed(void);
void Motor_PID_Down_SetSpeed(void);
#endif
