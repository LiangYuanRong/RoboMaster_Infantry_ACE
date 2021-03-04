#include "stm32f4xx.h"
#include "pid.h"
#include "maths.h"

/*
*���ܣ��Դ������ݽ���λ��ʽpid���㲢���ֵ
*���룺1.PID�ṹ������   2.��ǰֵ
*�������Ը��豸�Ŀ�����
*˵����Ŀ��ֵ�ڱջ����ƺ����︳ֵ
*/
int32_t Location_Pid_Int32(PID_t *pid , float actualValue) 
{
	int32_t dError;
	
	pid->Error = pid->SetValue - actualValue;             
	pid->iError += pid->Error;                            //������
	pid->iError=limit_int32(pid->iError,10000,-10000);    //�����޷�
	dError = pid->Error - pid->LastError;	                //΢��
	
	pid->outputValue_Int32=(pid->kp * pid->Error) + (pid->ki * pid->iError) + (pid->kd *  dError);   //���pid����
	
	pid->LastError=pid->Error;   
	
	return pid->outputValue_Int32;
}
