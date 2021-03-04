#ifndef __CAPACITOR_H
#define __CAPACITOR_H
#include "stm32f4xx.h"
#include "Chassis_Task.h"
#include "Remote_Task.h"

//����������ز����ṹ��
typedef struct         
{   
  u8 Cap_Charge_Flag;    //�����Ƿ�������
	u8 Cap_Out_Flag;       //�����Ƿ�����ŵ�
	int16_t Cap_Charge_PWM;//ͨ��pwm���Ƶ��ݳ������ѹ
	int16_t Cap_Charge_PWM_filt;//�����ĳ��pwm
}Super_Cap;

extern Super_Cap super_cap_data; 

void Capacitor_control_chassis(PowerLimit_t *Power_data , float Charge_current , int16_t motor_out_1, int16_t motor_out_2, int16_t motor_out_3, int16_t motor_out_4);


#endif
