#include "main.h"
#include "Remote_Task.h"
#include "Fire_Task.h"
#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "Gyro_Task.h"
#include "Mode_Def.h"
#include "Safecheck_Task.h"

 /***********************************
 *  注意：最大任务优先级设为10       *
***********************************/


/*************************任务基本配置************************/
/*功能：开始任务*/	
		//任务堆栈大小	
		#define START_STK_SIZE 		50
		//任务优先级
		#define START_TASK_PRIO		1
		//任务句柄
		TaskHandle_t StartTask_Handler;
		//任务函数
		void start_task(void *pvParameters);

/*功能：遥控任务*/ 
		//任务堆栈大小	
		#define REMOTE_STK_SIZE 	150
		//任务优先级
		#define REMOTE_TASK_PRIO	9
		//任务句柄
		TaskHandle_t RemoteTask_Handler;

/*功能：底盘任务*/ 
		//任务堆栈大小	
		#define CHASSIS_STK_SIZE 	 150
    //任务优先级
    #define CHASSIS_TASK_PRIO	 6
		//任务句柄
		TaskHandle_t ChassisTask_Handler;


/*功能：云台任务*/   
		//任务堆栈大小
		#define GIMBAL_STK_SIZE   200
		//任务优先级
		#define GIMBAL_TASK_PRIO  6
		//任务句柄
		TaskHandle_t GimbalTask_Handler;


/*功能：火控任务*/ 
		//任务堆栈大小	
		#define FIRE_STK_SIZE 	50
		//任务优先级
		#define FIRE_TASK_PRIO	6
		//任务句柄
		TaskHandle_t FireTask_Handler;


/*功能：陀螺仪任务*/ 
		//任务堆栈大小	
		#define GYRO_STK_SIZE 	2000
		//任务优先级
		#define GYRO_TASK_PRIO	5
		//任务句柄
		TaskHandle_t GyroTask_Handler;


/*功能：安全监测任务*/ 
		//任务堆栈大小	
		#define SAFE_STK_SIZE 	80
		//任务优先级
		#define SAFE_TASK_PRIO	4
		//任务句柄
		TaskHandle_t SafecheckTask_Handler;
		
		

/********************系统模式结构体声明**************************/
WorkStatus          workStatus         = POWEROFF;       //默认工作模式为待机模式

/**创建开始任务**/
void Start_Task(void)
{
	xTaskCreate((TaskFunction_t )start_task,      //任务函数
				(const char*    )"start_task",          //任务名称
				(uint16_t       )START_STK_SIZE,        //任务堆栈大小
				(void*          )NULL,                  //传递给任务函数的参数
				(UBaseType_t    )START_TASK_PRIO,       //任务优先级
				(TaskHandle_t*  )&StartTask_Handler);   //任务句柄              							
}




//开始任务 任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区

	//创建遥控任务
	xTaskCreate((TaskFunction_t )Remote_Task,     
                (const char*    )"Remote_Task",   
                (uint16_t       )REMOTE_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )REMOTE_TASK_PRIO,
                (TaskHandle_t*  )&RemoteTask_Handler);
	
	//创建火控任务
	xTaskCreate((TaskFunction_t )Fire_Task,     
                (const char*    )"Fire_Task",   
                (uint16_t       )FIRE_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )FIRE_TASK_PRIO,
                (TaskHandle_t*  )&FireTask_Handler);
	
	//创建底盘任务
	xTaskCreate((TaskFunction_t )Chassis_Task,      
							(const char*    )"Chassis_Task",   
							(uint16_t       )CHASSIS_STK_SIZE, 
							(void*          )NULL,
							(UBaseType_t    )CHASSIS_TASK_PRIO,
							(TaskHandle_t*  )&ChassisTask_Handler);

	//创建云台任务					
	xTaskCreate((TaskFunction_t )Gimbal_Task,       
							(const char*    )"Gimbal_Task",   
							(uint16_t       )GIMBAL_STK_SIZE, 
							(void*          )NULL,
							(UBaseType_t    )GIMBAL_TASK_PRIO,
							(TaskHandle_t*  )&GimbalTask_Handler);

	//创建安全检查任务					
	xTaskCreate((TaskFunction_t )Safecheck_Task,       
							(const char*    )"Safecheck_Task",   
							(uint16_t       )SAFE_STK_SIZE, 
							(void*          )NULL,
							(UBaseType_t    )SAFE_TASK_PRIO,
							(TaskHandle_t*  )&SafecheckTask_Handler);
							
							
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

