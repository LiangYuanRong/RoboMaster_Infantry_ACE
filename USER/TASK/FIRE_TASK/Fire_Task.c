/*==================================================*
*文件功能：1.控制整个发弹系统的任务
*         2.供弹系统的控制和发射摩擦轮的控制
*         3.热量限制（按等级）
*             
*备注：該文件还没加入裁判系统的热量限制
*
*发射模式：1.高射速模式（即用到了射速三个等级中的最高射速来给摩擦轮）（精确度高。打风车时或者远程时用到）
          2.低射速模式（默认用最低射速限制15m/s速度，保证高射频）（精确度低。2.5m范围内近战用）
					3.换血模式  （关闭热量限制，用血量换取热量，以最高射频进行快速打击，用在最后近距离打基地用，注意射速不要拉高，不然打两发就死）
				
*==================================================*/

#include "main.h"
#include "Fire_Task.h"
#include "Speed_Def.h"
#include "PID_Def.h"
#include "Mode_Def.h"
#include "Remote_Task.h"



//================参数定义
int fire_heart=0;  //火控任务心跳

//================结构体定义
Fire_WorkStatus     fire_workstatus    = STOP_FIRE;      //射弹模式默认为停火
Shoot_WorkStatus    shoot_workstatus   = LOW_SPEED;      //摩擦轮模式低速模式 (没用啦，新规则的热量跟射速无关，直接按最大的怼就行了)
Fire_task_t Fire_param; 
extern REFEREE_t REFEREE;             //裁判系统数据
extern TaskHandle_t FireTask_Handler; // 栈大小

//REFEREE.RobotStatus.shooter_heat0_cooling_limit - REFEREE.PowerHeat.shooter_heat0  //当前限制热量减去剩余热量（cooling_rate 冷却速度）
//REFEREE.RobotStatus.shooter_heat0_speed_limit   //当前射速限制

//================内部函数定义
static void Fire_Control(void);    //发弹系统控制
static void Fire_param_init(void); //参数初始化

 
/**火控任务**/
void Fire_Task(void *pvParameters)
{
	//加载时间
  vTaskDelay(FIRE_TASK_INIT_TIME);
	
	while(1)
	{
		fire_heart=!fire_heart;           //火控任务心跳
		LEDE3 = fire_heart;
		 
		Fire_param_init();
		Fire_Control();
		
		//任务周期
    vTaskDelay(FIRE_CONTROL_TIME_MS);
	}
}




/**参数初始化**/
static void Fire_param_init(void)
{
	Fire_param.GD_output=0;
	Fire_param.shoot_speed=0;
}


/**发弹系统控制**/
int16_t fire_count=0;
static void Fire_Control(void)
{
	if(workStatus==POWEROFF || workStatus==INITIALIZE)   //初始化或待机状态时摩擦轮停止
	{
		PWM_Shoot_Left = PWM_Shoot_Right = SHOOT_STOP;
		CAN1_SendCommand_Gimbal_Fire(0,0,0);			
	}
	else                                                  //进入控制状态后
	{		    					
		/*发射控制*/
		if(shoot_workstatus != STOP_SHOOT)                  //摩擦轮允许启动才能启动供弹
		{
			/*=========发射摩擦轮速度设定===========*/
//			if(shoot_workstatus == LOW_SPEED)
//			{
//				if(fire_count<=300)   //临时加的一个延时启动
//				{
//					PWM_Shoot_Left = ZERO_SHOOT_SPEED;
//					PWM_Shoot_Right = SHOOT_STOP;
//				  fire_count++; 
//				}
//				else if(fire_count>300)
//				{
//				  PWM_Shoot_Left = PWM_Shoot_Right = ZERO_SHOOT_SPEED;
//				}
//			}
			if(shoot_workstatus == LOW_SPEED)
			{
				//这里加射速等级变化
				if(REFEREE.RobotStatus.shooter_heat0_speed_limit==15) //零级
				{
					PWM_Shoot_Left = PWM_Shoot_Right = ZERO_SHOOT_SPEED;								
				}	
				if(REFEREE.RobotStatus.shooter_heat0_speed_limit==18) //一级
				{
					PWM_Shoot_Left = PWM_Shoot_Right = ONE_SHOOT_SPEED;								
				}					
				if(REFEREE.RobotStatus.shooter_heat0_speed_limit==22) //二级
				{
					PWM_Shoot_Left = PWM_Shoot_Right = TWO_SHOOT_SPEED;								
				}					
				if(REFEREE.RobotStatus.shooter_heat0_speed_limit==30) //三级
				{
					PWM_Shoot_Left = PWM_Shoot_Right = THREE_SHOOT_SPEED;								
				}									
			}			
			
			/*=========供弹电机控制============*/
			if(fire_workstatus == STOP_FIRE)
			{
				Fire_param.GD_output = Rmmotor_Speed_control(&FIRE_S_PID ,0 ,Fire_GD_Motor.speed,M2006_MAX_OUTPUT_CURRENT);
			}
			else if(fire_workstatus == FIRE)
			{
				//热量限制
				Fire_param.GD_output = Rmmotor_Speed_control(&FIRE_S_PID ,-LOADING_SPEED ,Fire_GD_Motor.speed,M2006_MAX_OUTPUT_CURRENT);				
			}
			else if(fire_workstatus == BACK)
			{
        Fire_param.GD_output = Rmmotor_Speed_control(&FIRE_S_PID ,LOADING_SPEED ,Fire_GD_Motor.speed,M2006_MAX_OUTPUT_CURRENT);							
			}						
		}
		else                                                 //手动关闭发射摩擦轮
		{
		  PWM_Shoot_Left = PWM_Shoot_Right = SHOOT_STOP;
      Fire_param.GD_output = 0;	
      fire_count = 0;			
		}
	}
	
	  
#ifdef fire_work          //定义了火控工作
	
	#ifdef board_chassis   //如果是底盘板，只输出供弹can信号  （因为发送冲突原因，can1发送在云台任务里底盘板上一起发）
 				PWM_Shoot_Left = PWM_Shoot_Right = SHOOT_STOP;
	#endif
#else                     //不定义火控工作，摩擦轮停止
			PWM_Shoot_Left = PWM_Shoot_Right = SHOOT_STOP;
	    fire_workstatus = STOP_FIRE;
#endif
	
}

