#ifndef _RMMOTOR_H
#define _RMMOTOR_H

#include "pid.h"
#include "maths.h"
#include "math.h"

/*****�����ʵֵ�ṹ��****/
typedef struct
{
	int32_t position;
	int16_t speed;
	int16_t speed_filt;
	int16_t last_Position;
	int16_t first_Flag;
	int32_t actual_Position;
}Motor_Deal_Data; 
  

/*****���̵��******/
extern PID_t CHASSIS_1_S_PID;
extern PID_t CHASSIS_2_S_PID;
extern PID_t CHASSIS_3_S_PID;
extern PID_t CHASSIS_4_S_PID;
 
extern PID_t CHASSIS_1_P_PID;
extern PID_t CHASSIS_2_P_PID;
extern PID_t CHASSIS_3_P_PID;
extern PID_t CHASSIS_4_P_PID;

extern PID_t CHASSIS_MOVE_FOLLOW_PID; // ���̸���PID
extern PID_t CHASSIS_ROTATE_FOLLOW_PID; // ���̸���PID

/******��̨���******/
extern PID_t GIMBAL_P_PITCH_PID;
extern PID_t GIMBAL_S_PITCH_PID;
extern PID_t GIMBAL_P_YAW_PID;
extern PID_t GIMBAL_S_YAW_PID;

extern PID_t GIMBAL_INDEPENDENT_P_YAW_PID;
extern PID_t GIMBAL_INDEPENDENT_S_YAW_PID;

extern PID_t GIMBAL_AUTO_P_PITCH_PID;
extern PID_t GIMBAL_AUTO_S_PITCH_PID;
extern PID_t GIMBAL_AUTO_P_YAW_PID;
extern PID_t GIMBAL_AUTO_S_YAW_PID;

extern PID_t GIMBAL_AUTO_BUFF_P_PITCH_PID;
extern PID_t GIMBAL_AUTO_BUFF_S_PITCH_PID;
extern PID_t GIMBAL_AUTO_BUFF_P_YAW_PID;
extern PID_t GIMBAL_AUTO_BUFF_S_YAW_PID;

extern PID_t GIMBAL_DOUBLE_P_PITCH_PID;
extern PID_t GIMBAL_DOUBLE_S_PITCH_PID;
extern PID_t GIMBAL_DOUBLE_P_YAW_PID;
extern PID_t GIMBAL_DOUBLE_S_YAW_PID;

/******�������********/
extern PID_t FIRE_S_PID;  


/******�������ʵֵ********/
extern Motor_Deal_Data Chassis_Motor[4];  //�����ĸ����
extern Motor_Deal_Data Pitch_Motor_Data;  //P��������
extern Motor_Deal_Data Yaw_Motor_Data;    //Y��������
extern Motor_Deal_Data Fire_GD_Motor;     //�����������
extern Motor_Deal_Data Second_Yaw_Motor_Data;     //�����������

/******�������********/
#define PI  3.1415926f                     //Բ����                 
#define YAW_RATIO  3*19                   //Y�������ٱ�   Ƥ��ת����3:1  ������ٱ�19:1 


/*=====��������========*/
void Motor_Actual_Position(Motor_Deal_Data *rmMotor, int16_t gear_Ratio,int16_t lap_encoder); //������ʵ����ֵ
int16_t Angle_Limiting_Int16(int16_t Angl_Err, int16_t lap_encoder);                           //�ٽǴ���16λ
int32_t Angle_Limiting_Int32(int32_t Angl_Error,int16_t buff,int16_t lap_encoder);            //�ٽǴ���32λ�����ٱȼ���
int32_t Check_CodeValue(int32_t value, int16_t gear_Ratio, int16_t lap_encoder);               //���ٽ�ֵ��λ����ֵ
int16_t Check_Motor_Block(int16_t position);                                                   //�������ת
float Get_Yaw_Different_Angle(Motor_Deal_Data *yaw_position , int16_t Ratio);                  //��ȡ��̨����̲��
int16_t Encoder_Real(int32_t read_code);                                                       //��Ȧ����ֵ����������ת��
int16_t Yaw_Actual_Code_Conversion(int16_t actual_code , int16_t max_code , int16_t middle_code);//Yaw����ʵλ�ã���Ȧ���������������Ҹ���

int16_t Rmmotor_Speed_control(PID_t *spid, int16_t setSpeed, int16_t actualSpeed, int16_t current_limit);  //����ٶȱջ�
int16_t Motor_Position_Speed_Control(PID_t *speed_pid, PID_t *position_pid, int16_t actual_position , int16_t actual_speed , int16_t setPosition, int16_t current_limit);


#endif
