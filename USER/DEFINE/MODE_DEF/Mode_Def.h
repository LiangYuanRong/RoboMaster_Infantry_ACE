#ifndef __MODEDEF_H
#define __MODEDEF_H

//�������Ƶĺ궨��


/*ģ�鹤������*/
#define watch_dog                //�������Ź�
#define gimbal_work              //��̨����
#define chassis_work             //���̹���
//#define fire_work                //�䵯ģʽ���� (��Ħ����)
#define power_limit              //������������
#define double_gimbal            //ʹ��˫��̨
//#define super_capacitor        //ʹ�ó�������
//#define chassis_exclusive_use  //�������ʹ��


/*
*���������˴���
*���ذ�����
*/
#define infantry1          //infantry1                     											   //��������(�������ղ�����������ɴ�С)
#define board_gimbal       //board_gimbal//board_chassis    											 //��������̨�廹�ǵ��̰�



/*****�������˵���̨��ֵ(����õ�������У׼��Ŀǰ2020��������P���õ���������ʼ��)******/
#ifdef infantry1
	#define Pitch_Middle_Angle 190  //����1:6052   ����2:1530
	#define Pitch_UP_Angle     190  //����1:6052   ����2:1530	
#endif

#ifdef infantry2
	#define Pitch_Middle_Angle 573  //����1:6052   ����2:1530
#endif


/*****�����Ƿ����������*****/
#define PITCH_GYRO_ANGULAR_SPEED  0//-(MPU6050_Real_Data.Gyro_Y) //P����ٶ�
#define YAW_GYRO_ANGULAR_SPEED    0//-(MPU6050_Real_Data.Gyro_Z) //Y����ٶ�       
#define YAW_POSTION_OUTPUT_FLAG   (1) 
#define YAW_ANGLE_FLAG            (1)                        //������λ�ö�Y��Ƕȵ�Ӱ��
#define YAW_SPEED_OUTPUT_FLAG     (-1)                       //���ٶȻ�Y����ٶȷ���
#define PITCH_POSTION_OUTPUT_FLAG (-1)
#define PITCH_SPEED_OUTPUT_FLAG   (-1)


/*****������ͨ�������ѡ��PID����*********/
#define test_short_focus  1      //�̽����
#define test_long_focus   0      //�������
#define test_industry     0      //��ҵ���

/*************��̨pitch��yaw�Ƕ�����************/
#define PITCH_ANGLE_LIMIT_UP    38
#define PITCH_ANGLE_LIMIT_DOWN  -22
#define YAW_ANGLE_LIMIT         130


/************��� ������*���ٱ� ***************/
#define YAW_RATIO  3*19         //Yaw��



#endif