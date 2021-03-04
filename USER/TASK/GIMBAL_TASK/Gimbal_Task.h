#ifndef __GIMBALTASK_H
#define __GIMBALTASK_H

/*OS控制任务周期以及启动时间*/
#define GIMBAL_TASK_INIT_TIME 	1
#define GIMBAL_CONTROL_TIME_MS  2

/*视觉模式*/
typedef enum 
{
  SHORT_FOUCUS,   //使用短焦
	LONG_FOUCUS,    //使用长焦
	INDUSTRY,       //使用工业
	
}vision_WorkStatus;


/*函数声明*/
void Gimbal_Task(void *pvParameters);   //云台任务

          


#endif
