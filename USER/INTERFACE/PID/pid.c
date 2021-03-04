#include "stm32f4xx.h"
#include "pid.h"
#include "maths.h"

/*
*功能：对传入数据进行位置式pid运算并输出值
*传入：1.PID结构体名称   2.当前值
*传出：对该设备的控制量
*说明：目标值在闭环控制函数里赋值
*/
int32_t Location_Pid_Int32(PID_t *pid , float actualValue) 
{
	int32_t dError;
	
	pid->Error = pid->SetValue - actualValue;             
	pid->iError += pid->Error;                            //误差积分
	pid->iError=limit_int32(pid->iError,10000,-10000);    //积分限幅
	dError = pid->Error - pid->LastError;	                //微分
	
	pid->outputValue_Int32=(pid->kp * pid->Error) + (pid->ki * pid->iError) + (pid->kd *  dError);   //输出pid运算
	
	pid->LastError=pid->Error;   
	
	return pid->outputValue_Int32;
}
