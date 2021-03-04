#ifndef _PID_H
#define _PID_H
#include "sys.h"

typedef struct         //pid�Ĳ���
{   
	float kp,ki,kd;           //��������΢�ֲ���
	int32_t SetValue;         //�趨ֵ
	int32_t ActualVaule;      //��ǰ��ʵֵ
	float Error;              //���
	float iError;             //������
	float LastError;          //�ϴ����
	float PrevError;          //���ϴ����
	int16_t OutputValue;      //16λ���ֵ
	int32_t outputValue_Int32;//32λ���ֵ
	float LastOutput;         //�ϴ����ֵ
}PID_t;


int32_t Location_Pid_Int32(PID_t *pid,float actualValue);

#endif
