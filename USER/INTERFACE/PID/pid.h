#ifndef _PID_H
#define _PID_H
#include "sys.h"

typedef struct         //pid的参数
{   
	float kp,ki,kd;           //比例积分微分参数
	int32_t SetValue;         //设定值
	int32_t ActualVaule;      //当前真实值
	float Error;              //误差
	float iError;             //误差积分
	float LastError;          //上次误差
	float PrevError;          //上上次误差
	int16_t OutputValue;      //16位输出值
	int32_t outputValue_Int32;//32位输出值
	float LastOutput;         //上次输出值
}PID_t;


int32_t Location_Pid_Int32(PID_t *pid,float actualValue);

#endif
