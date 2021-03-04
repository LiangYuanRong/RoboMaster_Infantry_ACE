#ifndef __CHASSISTASK_H
#define __CHASSISTASK_H

/*OS控制任务周期以及启动时间*/
#define CHASSIS_TASK_INIT_TIME 	5
#define CHASSIS_CONTROL_TIME_MS 2

/*底盘模式*/
typedef enum 
{
  FOLLOWING,     //跟随
	INDEPENDENT,   //底盘独立模式
	TWIST_WAIST,   //扭腰
	ROTATION,      //小陀螺
	STOP,          //停止运动
	
}Chassis_WorkStatus;

/*底盘功率限制*/
typedef struct Power_Limit
{
  float   Real_Power[3] ;//2最新 1上次 0上上次
	float   RemainPower[3];
	int32_t SumOutValue;
	int32_t LimitOutValue;
	float   scale;
}PowerLimit_t;


/*全局声明*/
void Chassis_Task(void *pvParameters);    
extern Chassis_WorkStatus  chassis_workStatus;      //底盘默认工作模式




#endif
