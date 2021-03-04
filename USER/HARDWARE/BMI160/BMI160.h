#ifndef __BMI160_H
#define __BMI160_H

#include "stm32f4xx.h"


typedef struct
{
    float Accel_X;  //转换成实际的X轴加速度，
    float Accel_Y;  //转换成实际的Y轴加速度，
    float Accel_Z;  //转换成实际的Z轴加速度，
    float Temp;     //转换成实际的温度，单位为摄氏度
    float Gyro_X;   //转换成实际的X轴角加速度，
    float Gyro_Y;   //转换成实际的Y轴角加速度，
    float Gyro_Z;   //转换成实际的Z轴角加速度
		float yaw_angle;
		float pitch_angle;
	  float roll_angle;
}BMI160_Data_Typedef;


typedef struct
{
    float Motor_1;          //底盘电机1电流（A）
    float Motor_2;          //底盘电机2电流（A）
    float Motor_3;          //底盘电机3电流（A）
    float Motor_4;          //底盘电机4电流（A）
	  float Cap_5;            //电容充电电流（A）
    float chassis_power_sum;//底盘总功率 (W)
	  float Cap_charge_power; //超级电容充电电流（W）
	  int32_t SumOutValue;    //底盘总输出
	  int32_t LimitOutValue;  //底盘限制输出
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
