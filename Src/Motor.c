#include "Motor.h"
#include "pid.h"
#include "stm32f1xx_it.h"

extern tPid pidMotor1Speed;
extern tPid pidMotor2Speed;
extern float speed1;
extern float speed2;
float Motor_Speet_Up = 0;



//�����ʼ������
void Motor_Init(void)
{
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);//������ʱ��
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim3);
//	HAL_TIM_Base_Start_IT(&htim1);//������ʱ���ж�
}



//�����˶�
//speed���ٶ�   -100-100
void Motor_Left_SetSpeed(float Setspeed)
{
	if(Setspeed >= 0)
	{
		HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,Setspeed);
	}
	if(Setspeed < 0)
	{
		HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,-Setspeed);
	}
	
}



//�����˶�
//speed���ٶ�  -100-100
void Motor_Right_SetSpeed(float Setspeed)
{
		if(Setspeed >= 0)
	{
		HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,Setspeed);
	}
	if(Setspeed < 0)
	{
		HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,-Setspeed);
	}
}



//С���˶�����
//speed1�������ٶ�   -100-100
//speed2�������ٶ�   -100-100
void Motor_Go_SetSpeed(float Setspeed1, float Setspeed2)
{
	Motor_Left_SetSpeed(Setspeed1);
	Motor_Right_SetSpeed(Setspeed2);
}


//��ȡ�����ٶ�
//*speed1������
//*speed2���ҳ���
void Motor_Read_MotorSpeed(float *Setspeed1, float *Setspeed2)
{
	short Encode1Count=0;
	short Encode2Count=0;

	Encode1Count=-(short)__HAL_TIM_GET_COUNTER(&htim2);
	Encode2Count=-(short)__HAL_TIM_GET_COUNTER(&htim4);
	__HAL_TIM_SET_COUNTER(&htim4,0);
	__HAL_TIM_SET_COUNTER(&htim2,0);
	*Setspeed1=(float)Encode1Count*100/21.3/11/4;
	*Setspeed2=(float)Encode2Count*100/21.3/11/4;
}

//PID���Ƴ����ٶ�
//*speed1������ -10-10
//*speed2���ҳ���	-10-10
void Motor_PID_Go_SetSpeed(float Motorspeed1, float Motorspeed2)
{
	pidMotor1Speed.target_val=Motorspeed1;
	pidMotor2Speed.target_val=Motorspeed2;
	
	Motor_Go_SetSpeed(PID_realize(&pidMotor1Speed,speed1),PID_realize(&pidMotor2Speed,speed2));
}


void Motor_PID_Up_SetSpeed(void)
{
	if(Motor_Speet_Up <= 4)Motor_Speet_Up +=0.1;
	Motor_PID_Go_SetSpeed(Motor_Speet_Up,Motor_Speet_Up);
}

void Motor_PID_Down_SetSpeed(void)
{
	if(Motor_Speet_Up >= 0)Motor_Speet_Up -=0.1;
	Motor_PID_Go_SetSpeed(Motor_Speet_Up,Motor_Speet_Up);
}

//��̺���
//Time:���û�ȡһ�ε�ʱ��
//���Mileage:�������
void Motor_Read_MotorMileage(float Time,float *Mileage)
{
	*Mileage += Time*speed2*3.14*7;
}
