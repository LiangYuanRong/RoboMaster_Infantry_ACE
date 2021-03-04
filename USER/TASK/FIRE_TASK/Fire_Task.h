#ifndef __FIRETASK_H
#define __FIRETASK_H

/*OS�������������Լ�����ʱ��*/
#define FIRE_TASK_INIT_TIME  5
#define FIRE_CONTROL_TIME_MS 2

/*��ؽṹ��*/
typedef struct Fire_param
{
   int16_t GD_output;          //����������
   uint8_t shoot_speed;        //��ǰ��������
	 uint8_t dead_mode;          //����ģʽ����
}Fire_task_t;

/*����ģʽ*/
typedef enum 
{
  FIRE,          //����
	AUTO_FIRE,     //�Զ�����
	STOP_FIRE,     //ֹͣ����
	BACK,          //�˵�
	
}Fire_WorkStatus;


/*Ħ����ģʽ*/
typedef enum 
{
  LOW_SPEED,
  HIGH_SPEED,
	STOP_SHOOT,
	
}Shoot_WorkStatus;



/*ȫ������*/
void Fire_Task(void *pvParameters);
extern Fire_WorkStatus     fire_workstatus;         //�䵯ģʽ
extern Shoot_WorkStatus    shoot_workstatus;        //Ħ����ģʽ
extern Fire_task_t Fire_param; 


/*Ħ����*/
#define PWM_Shoot_Left   TIM4->CCR1  //PD12
#define PWM_Shoot_Right  TIM4->CCR2	 //PD13


#endif
