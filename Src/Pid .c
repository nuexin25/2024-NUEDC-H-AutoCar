#include "pid.h"

//定义一个结构体类型变量
tPid pidMotor1Speed;
tPid pidMotor2Speed;
tPid pidPosition;
//给结构体类型变量赋初值
void PID_init()
{
	pidMotor1Speed.target_val=0.0;//目标值
	pidMotor1Speed.actual_val=0.0;//实际值
	pidMotor1Speed.err=0.0;//当前偏差
	pidMotor1Speed.err_last=0.0;//上次偏差
	pidMotor1Speed.err_sum=0.0;//误差累计值
	pidMotor1Speed.Kp=10;//比例，积分，微分系数
	pidMotor1Speed.Ki=8;
	pidMotor1Speed.Kd=0;
	
	
	pidMotor2Speed.target_val=0.0;//目标值
	pidMotor2Speed.actual_val=0.0;//实际值
	pidMotor2Speed.err=0.0;//当前偏差
	pidMotor2Speed.err_last=0.0;//上次偏差
	pidMotor2Speed.err_sum=0.0;//误差累计值
	pidMotor2Speed.Kp=10;//比例，积分，微分系数
	pidMotor2Speed.Ki=8;
	pidMotor2Speed.Kd=0;
	
	pidPosition.target_val=0.0;//目标值
	pidPosition.actual_val=0.0;//实际值
	pidPosition.err=0.0;//当前偏差
	pidPosition.err_last=0.0;//上次偏差
	pidPosition.err_sum=0.0;//误差累计值
	pidPosition.Kp=6;//比例，积分，微分系数
	pidPosition.Ki=3;
	pidPosition.Kd=0.2;
}
//比例p调节控制函数
float P_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//传递真实值
	pid->err = pid->target_val - pid->actual_val;//当前误差=目标值-真实值
	//比例控制调节    输出=Kp*当前误差
	pid->actual_val = pid->Kp*pid->err;
	return pid->actual_val;
}
//比例P 积分I 控制函数
float PI_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//传递真实值
	pid->err = pid->target_val - pid->actual_val;//当前误差=目标值-真实值
	pid->err_sum += pid->err;//误差累计值 = 当前误差累计和
	//使用PI控制 输出=Kp*当前误差+Ki*误差累计值
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum;
	return pid->actual_val;
}
// PID控制函数
float PID_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//传递真实值
	pid->err = pid->target_val - pid->actual_val;////当前误差=目标值-真实值
	pid->err_sum += pid->err;//误差累计值 = 当前误差累计和
	//使用PID控制 输出 = Kp*当前误差 + Ki*误差累计值 + Kd*(当前误差-上次误差)
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err - pid->err_last);
	//保存上次误差：这次误差赋值给上次误差
	pid->err_last = pid->err;


	return pid->actual_val;
}


