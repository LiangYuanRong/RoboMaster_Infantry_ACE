#ifndef __AUTOMATIC_STRIKE_H
#define __AUTOMATIC_STRIKE_H
#include "main.h"


/*****�Ӿ���ؽṹ��****/
typedef struct
{
  float auto_pitch_angle;      //�Ӿ�����P����
	float auto_yaw_angle;        //�Ӿ�����Y����
  float last_Auto_Pitch_Angle; //��һ�ε�P����
	float last_Auto_Yaw_Angle;   //��һ�ε�Y����
	float len;                   //�Ӿ����ؾ���
	float auto_yaw_speed;        //�Ӿ��ش���������Ľ��ٶ�
	float auto_pitch_sum;        //����P��ǶȻ���
  float auto_pitch_angle_kf;   //��ؿ������������P����
	float auto_yaw_angle_kf;     //��ؿ������������Y����

	int16_t auto_lost_data_count;  //��ʧĿ�����
	int16_t auto_lost_data_flag;   //��ʧĿ���־λ
	
	float pitch_control_data;     //P���Ӿ�������
	float yaw_control_data;       //y���Ӿ�������
	
}VISION_AUTO_DATA; 


//=====================�ṹ������
extern VISION_AUTO_DATA  Vision_Auto_Data;
//=====================��������
void MiniPC_Kalman_Data_Init(void);
void MiniPC_Send_Data(u8 data, u8 mode, u8 shoot_speed);
void MiniPC_Data_Deal(void); //�Ӿ������У�ϵͳ����
float hex2Float(uint8_t HighByte, uint8_t LowByte);



#endif