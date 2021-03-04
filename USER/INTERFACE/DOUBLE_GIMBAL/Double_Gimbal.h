#ifndef  __DOUBLE_GIMBAL_H
#define __DOUBLE_GIMBAL_H
#include "stdint.h"

typedef struct
{
  int16_t yaw_contral_data;  //y轴控制量 
	int16_t pitch_contral_data;//p轴控制量
	
	float   second_yaw_angle;  //副云台y轴真实角度
	float   second_pitch_angle;//副云台p轴真实角度
	float   last_second_yaw_angle; //上次y
	float   last_second_pitch_angle;//上次p
	
	int16_t yaw_middle_flag;   //y轴经过中间标志位
	int16_t yaw_init_flag;     //y轴初始化标志位
	int16_t pitch_block_flag;  //p轴堵转标志位（陀螺仪判断）
	int16_t pitch_block_count; //p轴堵转计数
	int16_t pitch_init_flag;   //p轴初始化标志位
	
}Second_Gimbal_Data_Typedef;

extern Second_Gimbal_Data_Typedef second_gimbal;

void Second_Gimbal_Control(int16_t yaw_control_data, int16_t pitch_contral_data);


#endif

