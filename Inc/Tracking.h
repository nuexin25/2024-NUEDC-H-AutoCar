#ifndef __TRACKING_H__
#define __TRACKING_H__

#include "stm32f1xx_hal.h"
#include "main.h"

#define READ_L1_OUT HAL_GPIO_ReadPin(L1_GPIO_Port,L1_Pin)
#define READ_L2_OUT HAL_GPIO_ReadPin(L2_GPIO_Port,L2_Pin)
#define READ_L3_OUT HAL_GPIO_ReadPin(L3_GPIO_Port,L3_Pin)
#define READ_R1_OUT HAL_GPIO_ReadPin(R1_GPIO_Port,R1_Pin)
#define READ_R2_OUT HAL_GPIO_ReadPin(R2_GPIO_Port,R2_Pin)
#define READ_R3_OUT HAL_GPIO_ReadPin(R3_GPIO_Port,R3_Pin)

/* 传感器全黑检测（到达顶点） */
uint8_t Tracking_All_Black(void);
/* 传感器一个都不在黑线上（完全脱线） */
uint8_t Tracking_All_White(void);
/* 巡线跟随：根据灰度传感器返回差速值 */
void Tracking_LineFollow(float baseSpeed, float *leftSpeed, float *rightSpeed);
/* 弧线跟随：陀螺仪辅助巡线 */
void Tracking_ArcFollow(float baseSpeed, float *leftSpeed, float *rightSpeed, float targetYawRate);
/* 获取位置偏差（用于PID修正） */
float Tracking_GetError(void);

#endif
