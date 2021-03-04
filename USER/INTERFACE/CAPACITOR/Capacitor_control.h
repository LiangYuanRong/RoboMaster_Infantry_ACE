#ifndef __CAPACITOR_H
#define __CAPACITOR_H
#include "stm32f4xx.h"
#include "Chassis_Task.h"
#include "Remote_Task.h"

//超级电容相关参数结构体
typedef struct         
{   
  u8 Cap_Charge_Flag;    //电容是否允许充电
	u8 Cap_Out_Flag;       //电容是否允许放电
	int16_t Cap_Charge_PWM;//通过pwm控制电容充电板充电电压
	int16_t Cap_Charge_PWM_filt;//处理后的充电pwm
}Super_Cap;

extern Super_Cap super_cap_data; 

void Capacitor_control_chassis(PowerLimit_t *Power_data , float Charge_current , int16_t motor_out_1, int16_t motor_out_2, int16_t motor_out_3, int16_t motor_out_4);


#endif
