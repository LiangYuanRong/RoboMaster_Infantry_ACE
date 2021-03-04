#include "main.h"
#include "Remote_Task.h"
#include "Fire_Task.h"
#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "Gyro_Task.h"
#include "Mode_Def.h"
#include "Safecheck_Task.h"

 /***********************************
 *  ע�⣺����������ȼ���Ϊ10       *
***********************************/


/*************************�����������************************/
/*���ܣ���ʼ����*/	
		//�����ջ��С	
		#define START_STK_SIZE 		50
		//�������ȼ�
		#define START_TASK_PRIO		1
		//������
		TaskHandle_t StartTask_Handler;
		//������
		void start_task(void *pvParameters);

/*���ܣ�ң������*/ 
		//�����ջ��С	
		#define REMOTE_STK_SIZE 	150
		//�������ȼ�
		#define REMOTE_TASK_PRIO	9
		//������
		TaskHandle_t RemoteTask_Handler;

/*���ܣ���������*/ 
		//�����ջ��С	
		#define CHASSIS_STK_SIZE 	 150
    //�������ȼ�
    #define CHASSIS_TASK_PRIO	 6
		//������
		TaskHandle_t ChassisTask_Handler;


/*���ܣ���̨����*/   
		//�����ջ��С
		#define GIMBAL_STK_SIZE   200
		//�������ȼ�
		#define GIMBAL_TASK_PRIO  6
		//������
		TaskHandle_t GimbalTask_Handler;


/*���ܣ��������*/ 
		//�����ջ��С	
		#define FIRE_STK_SIZE 	50
		//�������ȼ�
		#define FIRE_TASK_PRIO	6
		//������
		TaskHandle_t FireTask_Handler;


/*���ܣ�����������*/ 
		//�����ջ��С	
		#define GYRO_STK_SIZE 	2000
		//�������ȼ�
		#define GYRO_TASK_PRIO	5
		//������
		TaskHandle_t GyroTask_Handler;


/*���ܣ���ȫ�������*/ 
		//�����ջ��С	
		#define SAFE_STK_SIZE 	80
		//�������ȼ�
		#define SAFE_TASK_PRIO	4
		//������
		TaskHandle_t SafecheckTask_Handler;
		
		

/********************ϵͳģʽ�ṹ������**************************/
WorkStatus          workStatus         = POWEROFF;       //Ĭ�Ϲ���ģʽΪ����ģʽ

/**������ʼ����**/
void Start_Task(void)
{
	xTaskCreate((TaskFunction_t )start_task,      //������
				(const char*    )"start_task",          //��������
				(uint16_t       )START_STK_SIZE,        //�����ջ��С
				(void*          )NULL,                  //���ݸ��������Ĳ���
				(UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
				(TaskHandle_t*  )&StartTask_Handler);   //������              							
}




//��ʼ���� ������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���

	//����ң������
	xTaskCreate((TaskFunction_t )Remote_Task,     
                (const char*    )"Remote_Task",   
                (uint16_t       )REMOTE_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )REMOTE_TASK_PRIO,
                (TaskHandle_t*  )&RemoteTask_Handler);
	
	//�����������
	xTaskCreate((TaskFunction_t )Fire_Task,     
                (const char*    )"Fire_Task",   
                (uint16_t       )FIRE_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )FIRE_TASK_PRIO,
                (TaskHandle_t*  )&FireTask_Handler);
	
	//������������
	xTaskCreate((TaskFunction_t )Chassis_Task,      
							(const char*    )"Chassis_Task",   
							(uint16_t       )CHASSIS_STK_SIZE, 
							(void*          )NULL,
							(UBaseType_t    )CHASSIS_TASK_PRIO,
							(TaskHandle_t*  )&ChassisTask_Handler);

	//������̨����					
	xTaskCreate((TaskFunction_t )Gimbal_Task,       
							(const char*    )"Gimbal_Task",   
							(uint16_t       )GIMBAL_STK_SIZE, 
							(void*          )NULL,
							(UBaseType_t    )GIMBAL_TASK_PRIO,
							(TaskHandle_t*  )&GimbalTask_Handler);

	//������ȫ�������					
	xTaskCreate((TaskFunction_t )Safecheck_Task,       
							(const char*    )"Safecheck_Task",   
							(uint16_t       )SAFE_STK_SIZE, 
							(void*          )NULL,
							(UBaseType_t    )SAFE_TASK_PRIO,
							(TaskHandle_t*  )&SafecheckTask_Handler);
							
							
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}

