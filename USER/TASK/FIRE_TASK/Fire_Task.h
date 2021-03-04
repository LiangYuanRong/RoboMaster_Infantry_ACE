#ifndef __FIRETASK_H
#define __FIRETASK_H

/*OS控制任务周期以及启动时间*/
#define FIRE_TASK_INIT_TIME  5
#define FIRE_CONTROL_TIME_MS 2

/*火控结构体*/
typedef struct Fire_param
{
   int16_t GD_output;          //供弹电机输出
   uint8_t shoot_speed;        //当前设置射速
	 uint8_t dead_mode;          //死亡模式开关
}Fire_task_t;

/*发弹模式*/
typedef enum 
{
  FIRE,          //发射
	AUTO_FIRE,     //自动发射
	STOP_FIRE,     //停止发射
	BACK,          //退弹
	
}Fire_WorkStatus;


/*摩擦轮模式*/
typedef enum 
{
  LOW_SPEED,
  HIGH_SPEED,
	STOP_SHOOT,
	
}Shoot_WorkStatus;



/*全局声明*/
void Fire_Task(void *pvParameters);
extern Fire_WorkStatus     fire_workstatus;         //射弹模式
extern Shoot_WorkStatus    shoot_workstatus;        //摩擦轮模式
extern Fire_task_t Fire_param; 


/*摩擦轮*/
#define PWM_Shoot_Left   TIM4->CCR1  //PD12
#define PWM_Shoot_Right  TIM4->CCR2	 //PD13


#endif
