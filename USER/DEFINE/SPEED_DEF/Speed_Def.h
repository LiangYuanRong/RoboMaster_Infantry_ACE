#ifndef __SPEEDDEF_H
#define __SPEEDDEF_H

/*****���̵���ƶ��ٶ��趨*****/ 
#define M3508_MAX_OUTPUT_CURRENT  5000  //m3508������������  
#define M2006_MAX_OUTPUT_CURRENT  9500   //m2006������������

#define NORMAL_FORWARD_BACK_SPEED 	300   //������ֱͨ���ٶ�
#define NORMAL_LEFT_RIGHT_SPEED   	300   //������ͨƽ���ٶ�

#define HIGH_FORWARD_BACK_SPEED 	600     //���̼���ֱ���ٶ�
#define HIGH_LEFT_RIGHT_SPEED   	600     //���̼���ƽ���ٶ�

#define CHASSIS_SPEED_GAIN        12.0f    //������ʻ�ٶ�����
#define TWIST_SPEED_GAIN          2.0f     //Ť����ʻ�ٶ�����

#define CHASSIS_ROTATION_SPEED    2000      //С���ݵ���ת�ٶ� 
#define CHASSIS_ROTATION_MOVE_SPEED  1700   //С�����ƶ�ʱΪ��ֹ�켣ʧ���ת�� 
#define CHASSIS_TWIST_SPEED       1600      //Ť���ٶ�


/*****�������ң���ٶ�����******/ 
#define MOUSE_YAW_SPEED 			0.021    //���yaw���ٶ�����
#define MOUSE_PITCH_SPEED 		0.08     //���pitch���ٶ�����0.13
#define RC_YAW_SPEED          0.0026   //ң����yaw���ٶ�����
#define RC_PITCH_SPEED        0.0026   //ң����pitch���ٶ����� 0.0026


/*****����ϵͳ�ٶ��趨*******/ //�㼶��15m/s   һ����18m/s  ������22m/s  ������30m/s
#define THREE_SHOOT_SPEED     1226       //30Ħ���ָ���pwm
#define TWO_SHOOT_SPEED       1193       //22Ħ���ָ���pwm
#define ONE_SHOOT_SPEED       1172       //18Ħ���ָ���pwm
#define ZERO_SHOOT_SPEED      1165       //15Ħ���ֵ���pwm    1160-10.7m/s   1180-17.3m/s   1200-20.4m/s   1220-25m/s   1240
#define SHOOT_STOP            1000       //0Ħ����ֹͣpwm
#define LOADING_SPEED         -3000      //��������ٶ�

/*****�����̹���***********/  //�㼶��50w   һ����60w   ������70w   ������100w
#define ZERO_CHASSIS_POWER  50
#define ONE_CHASSIS_POWER   60
#define TWO_CHASSIS_POWER   70
#define THREE_CHASSIS_POWER 100


/*****�˶����ٶ�����*******/
#define STRAIGHT_ACCELERAD        3.5f      //ֱ�е��̼��ٶ�����
#define TRANSLATION_ACCELERAD     5.5f      //ƽ�Ƶ��̼��ٶ�����
#define ROTATING_ACCELERAD        19.0f     //��ת���̼��ٶ�����
#define GIMBAL_PITCH_ACCELERAD    2         //��̨�������ٶ�����
#define GIMBAL_YAW_ACCELERAD      2         //��̨ƫ�����ٶ�����
#define GIMBAL_AUTO_YAW_ACCELERAD 1         //��̨�Զ�ƫ�����ٶ�����


/*************��ʼ����̨�ٶ�******************/
#define YAW_INIT_SPEED 150


/*************�������Ʋ���**********************/
#define PowerLimit_Param  6.5f         //���������������
#define PowerLimit_Thres  55.0f        //����������ֵ�����幦��ʣ������50.0f


/****��������ͨ�˲�����**********/
#define Chassis_Fir_Ord_Low_Fil_Param       0.0110f  //0.0097fԽСԽƽ�ȣ�������Խ�ͣ�Խ��������ȣ��������ȸ���
#define Gimbal_Pitch_Fir_Ord_Low_Fil_Param  0.0864f
#define Gimbal_Yaw_Fir_Ord_Low_Fil_Param    0.215f
#define Gimbal_Remote_Fir_Ord_Low_Fil_Param 0.123f

/*************pitch��yaw���������************/
#define YAW_OUTPUT_LIMIT        11000 
#define YAW_INIT_OUTPUT_LIMIT   8000
#define PITCH_OUTPUT_LIMIT      8000 
#define PITCH_INIT_OUTPUT_LIMIT 5000

/*************���ٵ��������������������������**********/
#define GEAR_MOTOR_START_CURRENT   300
#define GEAR_MOTOR_REDUCE_CURRENT  1.21






#endif
