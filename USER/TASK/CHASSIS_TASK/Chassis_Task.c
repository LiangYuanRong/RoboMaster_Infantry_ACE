/*==================================================*
*文件功能：1.底盘控制任务
*         2.各种底盘模式的控制算法
*         3.功率限制（按等级）
*            
*备注：該文件还没加入裁判系统的分等级功率限制
*           未加超级电容控制部分
*==================================================*/

/*=====================*
*文件名：
*简介：
*函数功能：
*======================*/


#include "main.h"
#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "Remote_Task.h"
#include "Speed_Def.h"
#include "PID_Def.h"
#include "Mode_Def.h"
#include "Capacitor_control.h"


//==============参数定义
int chassis_heart = 0;             //底盘任务心跳
float Yaw_different_angle = 0.0f;  //Y轴与底盘的角度差
extern int16_t Sec_chassis_angle;  //副云台相对底盘的差角
extern int8_t  Vision_flag;        //视觉模式切换标志（0：关闭，1：自瞄，2：能量机关）

int ChassisTask_water=0;
static int16_t chassis_input[4] =  {0};                              //底盘电机处理输入
static int16_t chassis_output[4] = {0};                              //底盘电机处理输出
static float Chassis_ch0=0.0f, Chassis_ch1=0.0f, Chassis_ch2=0.0f ;  //底盘电机受控量



//==============结构体、枚举定义
extern REFEREE_t REFEREE;
extern TaskHandle_t ChassisTask_Handler; // 栈大小
PowerLimit_t Chassis_PowerLimit;                         //底盘功率限制结构体
Chassis_WorkStatus  chassis_workStatus = FOLLOWING;      //底盘默认工作模式为跟随

//==============内部函数声明
static void Chassis_control(void);                                               //底盘模式处理
static void Chassis_Follow(void);                                                //底盘跟随
static void Chassis_Twist(void);																								 //底盘扭腰
static void Chassis_Rotation(void);                                              //底盘小陀螺
static void Chassis_Independent(void);                                           //底盘不跟随
static void Chassis_Input(int16_t ch0, int16_t ch1, int16_t ch2);                //底盘控制数据处理
static void Chassis_to_Gimbal(void);                                             //底盘发数据到云台
static void Power_Init(void);                                                    //功率限制参数初始化
static void Power_Real(void);                                                    //功率参数更新
static void PowerLimitLoop(void);                                                //旧功率限制数据处理
static void Chassis_accelerated_Control(int16_t *ch0,int16_t *ch1,int16_t *ch2); //加速度限制处理
static void Keep_Driv_Track(int16_t *v0 ,int16_t *v1 ,int16_t *v2 ,int16_t *v3 ,int16_t SPEED_GAIN);// 防止运动轨迹失真



/**====底盘任务====**/
void Chassis_Task(void *pvParameters)
{
	//加载时间
  vTaskDelay(CHASSIS_TASK_INIT_TIME);
	
	//功率限制参数初始化
  Power_Init();                                                        
	
	while(1)
	{
		chassis_heart =! chassis_heart;                                       //底盘任务心跳(仅用于debug观察是否进行任务)
//		ChassisTask_water = uxTaskGetStackHighWaterMark(ChassisTask_Handler);
		LEDE1 = chassis_heart; 
		

	
#ifdef board_chassis 

			if((RC_Ctl.rc.s2==3 && RC_Ctl.rc.s1==2) || Vision_flag==1)
			{
				Yaw_different_angle = Sec_chassis_angle; //副云台坐标系				
			}
			else
			{
		    Yaw_different_angle = Get_Yaw_Different_Angle(&Yaw_Motor_Data,3*19);  //计算底盘与云台差角								
			}
		
		  Chassis_control();	                                                  //底盘控制
			
	   	Chassis_to_Gimbal();                                                  //can2底盘发数据到云台
			Chassis_control();	                                                  //底盘控制

//		//冲放电（充电时不能放电，放电时不能充电）
    if(Remote_data.RC_ch1>600)
		{
		  GPIO_SetBits(GPIOA,GPIO_Pin_2);//拉高IO电平，开启放电开关
//			TIM3->CCR2=0;
		}	
		else
		{
		  GPIO_ResetBits(GPIOA,GPIO_Pin_2);
			//充电
//			TIM3->CCR2=abs_int16(limit_int16(Remote_data.RC_sw*100/660,30,0));   //电容充电控制0-100,限制在97%内		
		}
#endif	
			
#ifdef board_gimbal
		   Yaw_different_angle = Get_Yaw_Different_Angle(&Yaw_Motor_Data,3*19);  //计算底盘与云台差角								
#endif
    
		
		//系统延时
    vTaskDelay(CHASSIS_CONTROL_TIME_MS);
	}
}



/*底盘发送数据到云台（yaw轴电机换算码盘值，yaw轴电机速度值，遥控值）*/
static int send_sign = 0;              //顺序发送遥控值

static void Chassis_to_Gimbal(void)
{ 
  /*底盘发遥控数据到云台*/ 
        
	if(workStatus!=POWEROFF) //待机刷新数据 
	{  
		if(send_sign==0)
			{ 
			if(RC_Ctl.rc.s2 == MOUSE_CONTROL && RC_Ctl.rc.s1 == 3)
			  CAN2_Send_Data1_MK_To_Gimbal();
			else
				CAN2_Send_Data1_RC_To_Gimbal();
		  send_sign=1;
			}
		if(send_sign==1){   
			CAN2_Send_Data2_To_Gimbal();
		  send_sign=0;}
  }
 
} 
	



/**====底盘运动控制====**/
static void Chassis_control(void)//底盘模式控制
{
	/*不同底盘模式对底盘输入做不同处理*/
	if(INITIALIZE_flag == 1)
	{
		  #ifdef board_chassis
		  workStatus = WORKING;   //开始运行
		  #endif
		  
			if(chassis_workStatus == FOLLOWING)
			{
				Chassis_Follow();	//跟随
			}
			if(chassis_workStatus == TWIST_WAIST)
			{
				Chassis_Twist();  //扭腰
			}
			if(chassis_workStatus == ROTATION)
			{
				Chassis_Rotation();//小陀螺
			}	
			if(chassis_workStatus == INDEPENDENT)
			{
				Chassis_Independent();//底盘不跟随
			}	
	
	}
	if(chassis_workStatus == STOP || INITIALIZE_flag == 0)
	{
		Chassis_Input(0,0,0);//停止
	}
	
}



/*====底盘跟随状态控制====*/
static void Chassis_Follow(void)
{
	Chassis_ch2 = Yaw_different_angle;//Angle_Limiting_Int32(Yaw_Motor_Data.actual_Position, 1, 8192);//临角处理   
	
	CHASSIS_ROTATE_FOLLOW_PID.SetValue = Chassis_ch2;       
	
	Chassis_ch2 = Location_Pid_Int32(&CHASSIS_ROTATE_FOLLOW_PID,0);	  
	
#ifdef chassis_exclusive_use	
	Chassis_Input(Remote_data.RC_ch0, Remote_data.RC_ch1, -(RC_Ctl.rc.ch2-1024));//底盘裸机运行，遥控控制ch2转向
#else 
	Chassis_Input(Remote_data.RC_ch0, Remote_data.RC_ch1, Chassis_ch2);  //底盘跟随，云台差角控制ch2转向
#endif
}	



/*===底盘扭腰状态控制=====*/
static int16_t TWIST_FLAG=0;  //0,左转；1，右转
static int16_t Chassis_Twist_acc=0;//底盘扭腰加速度限制

static void Chassis_Twist(void)
{
	/*判断转向*/
	if(Yaw_different_angle>20 && TWIST_FLAG==1)
	{
		TWIST_FLAG = 0;
	}
	else if(Yaw_different_angle<-20 && TWIST_FLAG==0)
	{
		TWIST_FLAG = 1;		
	}
	
	/*转动控制*/
	 if(TWIST_FLAG == 0)//左角度大于30，向左转
	 {
			//*开环固定扭速
		  if(Chassis_Twist_acc<=CHASSIS_TWIST_SPEED)
				Chassis_Twist_acc+=20;
			Chassis_ch2 = Chassis_Twist_acc;      
//			Chassis_ch2 = CHASSIS_TWIST_SPEED;      
			
			//*扭腰移动算法：云台为主坐标轴，目标值分解到底盘坐标轴
			 if(Remote_data.RC_ch1||Remote_data.RC_ch0)
				{ 
					Chassis_ch1=cos(Yaw_different_angle/180*PI)*Remote_data.RC_ch1;//需要用弧度单位计算
					Chassis_ch0=-sin(Yaw_different_angle/180*PI)*Remote_data.RC_ch1;
					if(Remote_data.RC_ch0)
					{
						Chassis_ch1+=sin(Yaw_different_angle/180*PI)*Remote_data.RC_ch0;
						Chassis_ch0+=cos(Yaw_different_angle/180*PI)*Remote_data.RC_ch0;
					}
				}
				else
				{
					Chassis_ch0 = 0;
					Chassis_ch1 = 0;
				}
		}
	 else if(TWIST_FLAG == 1)//右角度小于负30，向右转
	 {
			//*开环固定扭速 
		  if(Chassis_Twist_acc>=-CHASSIS_TWIST_SPEED)
				Chassis_Twist_acc-=20;		 
			Chassis_ch2 = Chassis_Twist_acc;      
//			Chassis_ch2 = -CHASSIS_TWIST_SPEED;      	
			
			//*扭腰移动算法：云台为主坐标轴，目标值分解到底盘坐标轴
			 if(Remote_data.RC_ch1||Remote_data.RC_ch0)
				{ 
					Chassis_ch1=cos(Yaw_different_angle/180*PI)*Remote_data.RC_ch1;//需要用弧度单位计算
					Chassis_ch0=-sin(Yaw_different_angle/180*PI)*Remote_data.RC_ch1;
					if(Remote_data.RC_ch0)
					{
						Chassis_ch1+=sin(Yaw_different_angle/180*PI)*Remote_data.RC_ch0;
						Chassis_ch0+=cos(Yaw_different_angle/180*PI)*Remote_data.RC_ch0;
					}
				}	
				else
				{
					Chassis_ch0 = 0;
					Chassis_ch1 = 0;
				}				
	 }
   else
	 {
		 Chassis_ch0 = 0;
		 Chassis_ch1 = 0;
		 Chassis_ch2 = 0;
	 }		 
		
	/*传入值处理*/
	Chassis_Input(Chassis_ch0*2, Chassis_ch1*2, Chassis_ch2); 
}


/*=====底盘小陀螺状态控制======*/
static void Chassis_Rotation(void)
{  
	//*小陀螺旋转移动算法：云台为主坐标轴，目标值分解到底盘坐标轴
	 if(Remote_data.RC_ch1||Remote_data.RC_ch0)  //此时有移动
		{ 
			Chassis_ch2 = CHASSIS_ROTATION_MOVE_SPEED;//小陀螺移动时减速
			
			Chassis_ch1=cos(Yaw_different_angle/180*PI)*Remote_data.RC_ch1;  //需要用弧度单位计算
			Chassis_ch0=-sin(Yaw_different_angle/180*PI)*Remote_data.RC_ch1;
		  if(Remote_data.RC_ch0)
			{
				Chassis_ch1+=sin(Yaw_different_angle/180*PI)*Remote_data.RC_ch0;
				Chassis_ch0+=cos(Yaw_different_angle/180*PI)*Remote_data.RC_ch0;
			}
		}
		else  //原地转快速
		{
	//*开环固定转速
	    Chassis_ch2 = CHASSIS_ROTATION_SPEED;//（左正右负） 
			Chassis_ch0 = 0;
			Chassis_ch1 = 0;			
		}
		
	//*传入值处理	
	Chassis_Input(Chassis_ch0*2, Chassis_ch1*2, Chassis_ch2); 
} 


/*======底盘不跟随状态控制========*/
static void Chassis_Independent(void) 
{
	//*云底分离移动算法：云台为主坐标轴，目标值分解到底盘坐标轴（与小陀螺一样）
	 if(Remote_data.RC_ch1||Remote_data.RC_ch0)
		{ 
			Chassis_ch1=cos(Yaw_different_angle/180*PI)*Remote_data.RC_ch1;//需要用弧度单位计算
			Chassis_ch0=-sin(Yaw_different_angle/180*PI)*Remote_data.RC_ch1;
		  if(Remote_data.RC_ch0)
			{
				Chassis_ch1+=sin(Yaw_different_angle/180*PI)*Remote_data.RC_ch0;
				Chassis_ch0+=cos(Yaw_different_angle/180*PI)*Remote_data.RC_ch0;
			}
		}
		else
		{
			Chassis_ch0=0;
			Chassis_ch1=0;
		}
		
	//*传入值处理	
	Chassis_Input(Chassis_ch0, Chassis_ch1, 0);	
}


 

/*====底盘输入数据处理====*/
static first_order_low_filter_type_t  Chassis_ForwardBack_LowFilt_Data = {0.0f,0.0f,0.0f,Chassis_Fir_Ord_Low_Fil_Param}; //低通前后控制量
static first_order_low_filter_type_t  Chassis_LeftRight_LowFilt_Data   = {0.0f,0.0f,0.0f,Chassis_Fir_Ord_Low_Fil_Param}; //低通左右控制量
static int16_t filt_ch0=0,filt_ch1=0;
static int16_t SPEED_GAIN = 0;
int16_t  chassis_ch0_jscope = 0;
int16_t  chassis_ch1_jscope = 0;

static void Chassis_Input(int16_t ch0, int16_t ch1, int16_t ch2)
{	

/*输入量斜坡函数处理*/	
	Chassis_accelerated_Control(&ch0,&ch1,&ch2);
	
/*输入量一阶低通滤波函数处理*/
	filt_ch0 = First_Order_Low_Filter(&Chassis_LeftRight_LowFilt_Data   , ch0); //左右控制量低通滤波
	filt_ch1 = First_Order_Low_Filter(&Chassis_ForwardBack_LowFilt_Data , ch1); //前后控制量低通滤波

//	filt_ch0 = ch0;
//	filt_ch1 = ch1;
	
	chassis_ch0_jscope = filt_ch0;
	chassis_ch1_jscope = filt_ch1;
	
/*输入量根据底盘模式处理*/
	if(workStatus==POWEROFF || workStatus==INITIALIZE ) //关电或初始化阶段
	{
		chassis_input[0] = 0;
		chassis_input[1] = 0;
		chassis_input[2] = 0;
		chassis_input[3] = 0;
	}
	else
	{
		if(chassis_workStatus==FOLLOWING)  //跟随
		{
			//麦轮运动学合成
			//正常行驶速度增益
			chassis_input[0] = (-(filt_ch1  + ch2 - filt_ch0))*CHASSIS_SPEED_GAIN;
		  chassis_input[1] = (filt_ch1 - ch2 + filt_ch0)   *CHASSIS_SPEED_GAIN;
		  chassis_input[2] = (-(filt_ch1 + ch2 + filt_ch0))*CHASSIS_SPEED_GAIN;
		  chassis_input[3] = (filt_ch1 - ch2 - filt_ch0)   *CHASSIS_SPEED_GAIN;
			
			SPEED_GAIN = CHASSIS_SPEED_GAIN;
		}
		if(chassis_workStatus==ROTATION||chassis_workStatus==TWIST_WAIST||chassis_workStatus==INDEPENDENT)  //小陀螺,扭腰，底盘不跟随
		{
			//麦轮运动学合成
			//特殊模式行驶速度增益
			chassis_input[0] = (-(filt_ch1 + ch2 - filt_ch0))*TWIST_SPEED_GAIN;
		  chassis_input[1] = (filt_ch1 - ch2 + filt_ch0)   *TWIST_SPEED_GAIN;
		  chassis_input[2] = (-(filt_ch1 + ch2 + filt_ch0))*TWIST_SPEED_GAIN;
		  chassis_input[3] = (filt_ch1 - ch2 - filt_ch0)   *TWIST_SPEED_GAIN;
			
			SPEED_GAIN = TWIST_SPEED_GAIN;
		}
		if(chassis_workStatus==STOP)       //停止
		{
		  chassis_input[0] = 0;
		  chassis_input[1] = 0;
		  chassis_input[2] = 0;
		  chassis_input[3] = 0;			
		}
	}

	
/*电机PID速度闭环处理*/
	chassis_output[0]=Rmmotor_Speed_control(&CHASSIS_1_S_PID,chassis_input[0],Chassis_Motor[0].speed,M3508_MAX_OUTPUT_CURRENT);
	chassis_output[1]=Rmmotor_Speed_control(&CHASSIS_2_S_PID,chassis_input[1],Chassis_Motor[1].speed,M3508_MAX_OUTPUT_CURRENT);
	chassis_output[2]=Rmmotor_Speed_control(&CHASSIS_3_S_PID,chassis_input[2],Chassis_Motor[2].speed,M3508_MAX_OUTPUT_CURRENT);
	chassis_output[3]=Rmmotor_Speed_control(&CHASSIS_4_S_PID,chassis_input[3],Chassis_Motor[3].speed,M3508_MAX_OUTPUT_CURRENT);

	
/*纯电流环控制电机(test)*/
//	if(Chassis_Motor[0].speed>=0)
//	  Motor_power[2] = POWER_Data.Motor_3;
//  else
//	  Motor_power[2] = -POWER_Data.Motor_3;
//	
//	if(Chassis_Motor[1].speed>=0)
//	  Motor_power[0] = POWER_Data.Motor_1;
//  else
//	  Motor_power[0] = -POWER_Data.Motor_1;	
//	
//	if(Chassis_Motor[2].speed>=0)
//	  Motor_power[3] = POWER_Data.Motor_4;
//  else
//	  Motor_power[3] = -POWER_Data.Motor_4;	
//	
//	if(Chassis_Motor[3].speed>=0)
//	  Motor_power[1] = POWER_Data.Motor_2;
//  else
//	  Motor_power[1] = -POWER_Data.Motor_2;	
//	
//	chassis_output[0]=Rmmotor_Speed_control(&CHASSIS_1_P_PID,chassis_input[0], Motor_power[2]*1000,900);
//	chassis_output[1]=Rmmotor_Speed_control(&CHASSIS_2_P_PID,chassis_input[1], Motor_power[0]*1000,900);
//	chassis_output[2]=Rmmotor_Speed_control(&CHASSIS_3_P_PID,chassis_input[2], Motor_power[3]*1000,900);
//	chassis_output[3]=Rmmotor_Speed_control(&CHASSIS_4_P_PID,chassis_input[3], Motor_power[1]*1000,900);

	
/*更新裁判系统功率数据（电容和功限都要用到）*/	
	Power_Real();         
	 
/*超级电容控制逻辑*/
#ifdef super_capacitor
  Capacitor_control_chassis(&Chassis_PowerLimit , 0 , chassis_output[0], chassis_output[1], chassis_output[2], chassis_output[3]);
#endif

/*底盘功率限制处理*/
#ifdef power_limit
{
	if(Chassis_PowerLimit.RemainPower[2]<PowerLimit_Thres && Chassis_PowerLimit.RemainPower[2]>5)  //不等0防止裁判系统数据传输空
	{
    Chassis_PowerLimit.SumOutValue   = abs(chassis_output[0])+ abs(chassis_output[1])+ abs(chassis_output[2])+ abs(chassis_output[3]); //输入数据求绝对和
    Chassis_PowerLimit.LimitOutValue = Chassis_PowerLimit.RemainPower[2] * Chassis_PowerLimit.RemainPower[2] * PowerLimit_Param ;      //缓冲功率平方和乘功率限制系数
		//将输出值进行比例分配
		chassis_output[0] = Chassis_PowerLimit.LimitOutValue * chassis_output[0] / Chassis_PowerLimit.SumOutValue;
		chassis_output[1] = Chassis_PowerLimit.LimitOutValue * chassis_output[1] / Chassis_PowerLimit.SumOutValue;
		chassis_output[2] = Chassis_PowerLimit.LimitOutValue * chassis_output[2] / Chassis_PowerLimit.SumOutValue;
		chassis_output[3] = Chassis_PowerLimit.LimitOutValue * chassis_output[3] / Chassis_PowerLimit.SumOutValue;
	}
	else if(Chassis_PowerLimit.RemainPower[2] <=5)// && Chassis_PowerLimit.RemainPower[2]!=0
  {
		chassis_output[0] = 0;
		chassis_output[1] = 0;
		chassis_output[2] = 0;
		chassis_output[3] = 0;		
	} 
}

/*防止底盘运动轨迹失真处理*/	
	Keep_Driv_Track(&chassis_output[0] ,&chassis_output[1] ,&chassis_output[2] ,&chassis_output[3] ,14);

//{
//  PowerLimitLoop();
//	chassis_output[0]*=Chassis_PowerLimit.scale;
//	chassis_output[1]*=Chassis_PowerLimit.scale;
//	chassis_output[2]*=Chassis_PowerLimit.scale;
//	chassis_output[3]*=Chassis_PowerLimit.scale;
//}

#endif
 
	
/*输出量发送到电机*/
#ifdef board_chassis  //只有底盘板启动底盘工作才能发送can

 #ifdef chassis_work
    if(INITIALIZE_flag==1)  //没搞懂为啥初始化时如果给底盘的四个电机发数据，再给y轴单独发时会发不出去，导致初始化时y轴不转。所以这里加了判断但还没初始化完成时，不给底盘电机发数据
      CAN1_SendCommand_Chassis(chassis_output[0],chassis_output[1],chassis_output[2],chassis_output[3]);
 #endif

#endif
	
}


/*==底盘加速度限制斜坡函数==*/
static void Chassis_accelerated_Control(int16_t *ch0,int16_t *ch1,int16_t *ch2)				
{
	static int16_t last_ch[3] = {0,0,0};
	int16_t temp[3];
	
	temp[0] = *ch0 - last_ch[0];
	temp[1] = *ch1 - last_ch[1];
	temp[2] = *ch2 - last_ch[2];
	
	if(RC_Ctl.rc.s2 == RC_CONTROL)													//遥控模式
	{
		if(abs_float(temp[0]) > TRANSLATION_ACCELERAD)
			*ch0 = last_ch[0] + temp[0]/abs_float(temp[0])*TRANSLATION_ACCELERAD;
		if(abs_float(temp[1]) > STRAIGHT_ACCELERAD)
			*ch1 = last_ch[1] + temp[1]/abs_float(temp[1])*STRAIGHT_ACCELERAD;
		
		if(chassis_workStatus==TWIST_WAIST||chassis_workStatus==ROTATION)         //扭腰模式下才用旋转加速度限制
		{
			if(abs_float(temp[2]) > ROTATING_ACCELERAD)
				*ch2 = last_ch[2] + temp[2]/abs_float(temp[2])*ROTATING_ACCELERAD;
	  }
	}
	if(RC_Ctl.rc.s2 == MOUSE_CONTROL)												//键盘模式
	{
		if(abs_float(temp[0]) > TRANSLATION_ACCELERAD)
			*ch0 = last_ch[0] + temp[0]/abs_float(temp[0])*TRANSLATION_ACCELERAD;
		if(abs_float(temp[1]) > STRAIGHT_ACCELERAD)
			*ch1 = last_ch[1] + temp[1]/abs_float(temp[1])*STRAIGHT_ACCELERAD;
		
		if(chassis_workStatus==TWIST_WAIST)                   //扭腰模式下才用旋转加速度限制
		{
			if(abs_float(temp[2]) > ROTATING_ACCELERAD)
				*ch2 = last_ch[2] + temp[2]/abs_float(temp[2])*ROTATING_ACCELERAD;
	  }
	}
	last_ch[0] = *ch0;	
	last_ch[1] = *ch1;
	last_ch[2] = *ch2;
	
}



/*
*功能：底盘防止运动轨迹失真
*传入：底盘四个轮子输出指针
*传出：处理后的四个输出
*/
static void Keep_Driv_Track(int16_t *v0 ,int16_t *v1 ,int16_t *v2 ,int16_t *v3 ,int16_t SPEED_GAIN)						
{
	static int16_t max_v=0;
	static float scale=1.0f;
	
	max_v = max_abs(*v0,max_abs(*v1,max_abs(*v2,*v3)));		// 取速度数值最大的轮子
	if(SPEED_GAIN == 0)
	{ *v0 = *v1 = *v2 = *v3 = 0;}
	else if(max_v > (SPEED_GAIN * 660) )
	{
		scale = max_v/(SPEED_GAIN * 660);
		*v0 = (int16_t)((*v0)/scale);
		*v1 = (int16_t)((*v1)/scale);
		*v2 = (int16_t)((*v2)/scale);
		*v3 = (int16_t)((*v3)/scale);
	}
}



/*****************************************************************底盘功率限制**************************************************************************************/
/*======功率限制相关参数初始化=======*/
static void Power_Init(void)
{
	Chassis_PowerLimit.Real_Power[2]=REFEREE.PowerHeat.chassis_power;    //实时功率
	Chassis_PowerLimit.RemainPower[2]=REFEREE.PowerHeat.chassis_power_buffer; //功率缓冲
	
	Chassis_PowerLimit.Real_Power[1]=Chassis_PowerLimit.Real_Power[2];
	Chassis_PowerLimit.Real_Power[0]=Chassis_PowerLimit.Real_Power[1];
	
	Chassis_PowerLimit.RemainPower[1]=Chassis_PowerLimit.RemainPower[2];
	Chassis_PowerLimit.RemainPower[0]=Chassis_PowerLimit.RemainPower[1];
	
	REFEREE.PowerHeat.error=0;
	Chassis_PowerLimit.RemainPower[2] = Chassis_PowerLimit.RemainPower[1] = Chassis_PowerLimit.RemainPower[0] = 60; //缓冲功率
}

/*======防止裁判系统失真=======*/
static void Power_Real(void)
{
	if(REFEREE.PowerHeat.error!=1||Chassis_PowerLimit.RemainPower[2]>20)//保证频率为50Hz
	{
		Chassis_PowerLimit.Real_Power[2]=REFEREE.PowerHeat.chassis_power;
		Chassis_PowerLimit.RemainPower[2]=REFEREE.PowerHeat.chassis_power_buffer;
	}
	else 
	{
		Chassis_PowerLimit.Real_Power[2]=(2*Chassis_PowerLimit.Real_Power[1]-Chassis_PowerLimit.Real_Power[0])*1.3f;//数据帧丢失，通过微分关系预测此时功率，为了保证预测功率小于实际功率，取安全系数1.3
		Chassis_PowerLimit.RemainPower[2]=(2*Chassis_PowerLimit.RemainPower[1]-Chassis_PowerLimit.RemainPower[0])/1.3f;
	}
	//数据迭代
	Chassis_PowerLimit.Real_Power[1]=Chassis_PowerLimit.Real_Power[2];
	Chassis_PowerLimit.Real_Power[0]=Chassis_PowerLimit.Real_Power[1];
	
	Chassis_PowerLimit.RemainPower[1]=Chassis_PowerLimit.RemainPower[2];
	Chassis_PowerLimit.RemainPower[0]=Chassis_PowerLimit.RemainPower[1];
}

/*============================2019赛季旧功率限制（test）===============*/
static void PowerLimitLoop(void)
{
	float Power_p_error=0;
	
	Power_Real();//防止裁判数据失真处理
	
	if(Chassis_PowerLimit.RemainPower[2]>=50)  //缓冲功率足够
	{
		Chassis_PowerLimit.scale = 1;
	}

	else if(Chassis_PowerLimit.RemainPower[2]>10&&Chassis_PowerLimit.RemainPower[2]<50)  //缓冲功率减少，速度降低
	{
		Power_p_error = 50 - Chassis_PowerLimit.RemainPower[2];
		Chassis_PowerLimit.scale = 1 - (float)(Power_p_error/40);	
	}
	else if(Chassis_PowerLimit.RemainPower[2]<=10||Chassis_PowerLimit.RemainPower[2]>0)  //缓冲功率不够，直接底盘电机输出为0
	{
		Chassis_PowerLimit.scale=0; 
	}
	else if(Chassis_PowerLimit.RemainPower[2]==0)//检测不到裁判系统还是可以低速走
	{
	  Chassis_PowerLimit.scale=0.7;       
	}

}


