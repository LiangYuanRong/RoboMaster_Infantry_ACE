#ifndef __GIMBALTASK_H
#define __GIMBALTASK_H

/*OS�������������Լ�����ʱ��*/
#define GIMBAL_TASK_INIT_TIME 	1
#define GIMBAL_CONTROL_TIME_MS  2

/*�Ӿ�ģʽ*/
typedef enum 
{
  SHORT_FOUCUS,   //ʹ�ö̽�
	LONG_FOUCUS,    //ʹ�ó���
	INDUSTRY,       //ʹ�ù�ҵ
	
}vision_WorkStatus;


/*��������*/
void Gimbal_Task(void *pvParameters);   //��̨����

          


#endif
