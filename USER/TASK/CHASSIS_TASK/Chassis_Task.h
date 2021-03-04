#ifndef __CHASSISTASK_H
#define __CHASSISTASK_H

/*OS�������������Լ�����ʱ��*/
#define CHASSIS_TASK_INIT_TIME 	5
#define CHASSIS_CONTROL_TIME_MS 2

/*����ģʽ*/
typedef enum 
{
  FOLLOWING,     //����
	INDEPENDENT,   //���̶���ģʽ
	TWIST_WAIST,   //Ť��
	ROTATION,      //С����
	STOP,          //ֹͣ�˶�
	
}Chassis_WorkStatus;

/*���̹�������*/
typedef struct Power_Limit
{
  float   Real_Power[3] ;//2���� 1�ϴ� 0���ϴ�
	float   RemainPower[3];
	int32_t SumOutValue;
	int32_t LimitOutValue;
	float   scale;
}PowerLimit_t;


/*ȫ������*/
void Chassis_Task(void *pvParameters);    
extern Chassis_WorkStatus  chassis_workStatus;      //����Ĭ�Ϲ���ģʽ




#endif
