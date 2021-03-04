#ifndef __BMI160_H
#define __BMI160_H

#include "stm32f4xx.h"


typedef struct
{
    float Accel_X;  //ת����ʵ�ʵ�X����ٶȣ�
    float Accel_Y;  //ת����ʵ�ʵ�Y����ٶȣ�
    float Accel_Z;  //ת����ʵ�ʵ�Z����ٶȣ�
    float Temp;     //ת����ʵ�ʵ��¶ȣ���λΪ���϶�
    float Gyro_X;   //ת����ʵ�ʵ�X��Ǽ��ٶȣ�
    float Gyro_Y;   //ת����ʵ�ʵ�Y��Ǽ��ٶȣ�
    float Gyro_Z;   //ת����ʵ�ʵ�Z��Ǽ��ٶ�
		float yaw_angle;
		float pitch_angle;
	  float roll_angle;
}BMI160_Data_Typedef;


typedef struct
{
    float Motor_1;          //���̵��1������A��
    float Motor_2;          //���̵��2������A��
    float Motor_3;          //���̵��3������A��
    float Motor_4;          //���̵��4������A��
	  float Cap_5;            //���ݳ�������A��
    float chassis_power_sum;//�����ܹ��� (W)
	  float Cap_charge_power; //�������ݳ�������W��
	  int32_t SumOutValue;    //���������
	  int32_t LimitOutValue;  //�����������
}POWER_Data_Typedef;


extern BMI160_Data_Typedef BMI160_Data;
extern BMI160_Data_Typedef Sec_BMI160_Data;
extern POWER_Data_Typedef  POWER_Data;

void BMI_Data_Deal(void);
void Power_Deal(void);

void BMI160_Zero_Correct(void);
void BMI160_AngleZ_Zero(void);
void Gyro_usart_iwdg(void);


#endif
