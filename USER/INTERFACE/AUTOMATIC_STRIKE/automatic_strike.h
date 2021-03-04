#ifndef __AUTOMATIC_STRIKE_H
#define __AUTOMATIC_STRIKE_H
#include "main.h"


/*****视觉相关结构体****/
typedef struct
{
  float auto_pitch_angle;      //视觉传回P轴差角
	float auto_yaw_angle;        //视觉传回Y轴差角
  float last_Auto_Pitch_Angle; //上一次的P轴差角
	float last_Auto_Yaw_Angle;   //上一次的Y轴差角
	float len;                   //视觉传回距离
	float auto_yaw_speed;        //视觉回传数据算出的角速度
	float auto_pitch_sum;        //传回P轴角度积分
  float auto_pitch_angle_kf;   //电控卡尔曼处理后的P轴差角
	float auto_yaw_angle_kf;     //电控卡尔曼处理后的Y轴差角

	int16_t auto_lost_data_count;  //丢失目标计数
	int16_t auto_lost_data_flag;   //丢失目标标志位
	
	float pitch_control_data;     //P轴视觉控制量
	float yaw_control_data;       //y轴视觉控制量
	
}VISION_AUTO_DATA; 


//=====================结构体声明
extern VISION_AUTO_DATA  Vision_Auto_Data;
//=====================函数声明
void MiniPC_Kalman_Data_Init(void);
void MiniPC_Send_Data(u8 data, u8 mode, u8 shoot_speed);
void MiniPC_Data_Deal(void); //视觉（裁判）系统数据
float hex2Float(uint8_t HighByte, uint8_t LowByte);



#endif
