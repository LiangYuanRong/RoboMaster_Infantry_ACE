/*==================================================*
*文件功能：1.云台控制任务
*         2.各种云台模式的控制算法
*             
*备注：該文件还没加入副云台控制
*==================================================*/

//===============头文件
#include "main.h"
#include "Gimbal_Task.h"
#include "Chassis_Task.h"
#include "Remote_Task.h"
#include "Fire_Task.h"
#include "Speed_Def.h"
#include "PID_Def.h"
#include "Mode_Def.h"
#include "Gyro_Task.h"
#include "kalman_filter.h"

//================参数定义
int gimbal_heart=0;                     //云台任务心跳
int GimbalTask_water=0;                 //堆栈剩余

int8_t INITIALIZE_flag=0;               //发送给底盘的初始化成功标志
int8_t YAW_ZERO_FLAG = 0;               //yaw轴中值光电标志
int8_t SEC_YAW_ZERO_FLAG = 0;           //副yaw轴中值光电标志

int8_t gimbal_work_mode=0;              //云台状态切换控制量处理标志位
int16_t gimbal_output[4] = {0,0,0,0};   //云台电机处理输出    0:P轴电机   1：Y轴电机   2:副yaw轴电机   3：副pitch轴电机

int16_t yaw_different_angle=0;          //云台底盘差角
float Gimbal_yaw_angle=0.0f;            //云台当前角度
static float Last_Gimbal_yaw_angle=0.0f;//云台保存角度
int8_t GIMBAL_SUPPLY_FLAG = 0;          //补给状态标志位(0:归中状态，进入补给状态  1:云台90度转动中  2:到达指定位置  3:归中中   4：补给模式结束)

int16_t second_yaw=0;
extern float Yaw_different_angle;       //主云台相对底盘的差角
float  Sec_Yaw_different_angle=0.0f;    //副云台相对主云台的差角
int16_t  Sec_chassis_angle=0;          //副云台相对底盘的差角

//================结构体定义
//extKalman gimbal_buff_yaw;             //定义一个打符yaw轴卡尔曼结构体            

//================内部函数定义
  //云台控制
static void Gimbal_Control(void);       //云台状态处理          
  //云台行为
static void Gimbal_Work(void);          //云台正常工作模式
static void Gimbal_Init(void);          //云台初始化
static  void Yaw_Init(void);            //Yaw轴初始化
static void Sec_Yaw_Init(void);         //副Yaw轴初始化
static	void Pitch_Init(void);          //Pitch轴初始化
static void Gimbal_Auto(void);          //云台自瞄模式
static void Gimbal_Buff(void);          //云台打符模式
static void Gimbal_Independent(void);   //云台独立模式
static void Gimbal_Supply(void);        //云台补弹模式
static void Gimbal_Stop(void);          //云台无力
 //控制算法
static void Gimbal_Angle_Limit(void);   //云台角度限制
static void Correct_Yaw_Zero(void);     //yaw光电校正
static void Correct_Sec_Yaw_Zero(void); //副yaw光电校正
static void Change_camera(int camera_num);//实时参考小电脑数据改变相机参数






/**====云台任务====**/
void Gimbal_Task(void *pvParameters)
{
  vTaskDelay(GIMBAL_TASK_INIT_TIME);	
	
	BMI160_Zero_Correct();            //发送校准信号给陀螺仪模块
//	BMI160_AngleZ_Zero();           //陀螺仪模块yaw轴归零	
	MiniPC_Kalman_Data_Init();        //视觉数据卡尔曼参数初始化
	
	while(1)
	{
		gimbal_heart = !gimbal_heart;    //云台任务心跳
		LEDE2 = gimbal_heart;

    Gimbal_Control();                //云台工作
		
#ifdef chassis_exclusive_use
		INITIALIZE_flag = 1;             //底盘单独使用时，初始化标志为1
#endif		
		//系统延时
    vTaskDelay(GIMBAL_CONTROL_TIME_MS);
	}
}



/**====云台状态控制====**/
static void Gimbal_Control(void)
{
  Correct_Yaw_Zero();                        //yaw轴码盘修正
//  Correct_Sec_Yaw_Zero();                    //副yaw轴码盘修正

#ifdef board_gimbal
//==============预处理============
	YAW_ZERO_FLAG = Yaw_Zero_Value();         //实时监测Yaw校准光电  (到中值返回1)
	SEC_YAW_ZERO_FLAG = Sec_Yaw_Zero_Value(); //实时监测副Yaw校准光电(到中值返回1)	
	MiniPC_Data_Deal();                       //小电脑传回数据处理
	CAN2_Send_Yaw_Data_To_Chassis(INITIALIZE_flag , gimbal_output[1] , YAW_ZERO_FLAG , GIMBAL_SUPPLY_FLAG , Sec_chassis_angle);          //云台数据发送给底盘
	BMI_Data_Deal();                          //外接陀螺仪 
	Sec_Yaw_different_angle = Get_Yaw_Different_Angle(&Second_Yaw_Motor_Data , 1*3);                             //码盘解算出副云台与主云台之间的差角 
	Sec_chassis_angle       = loop_fp32_constrain((Yaw_different_angle - Sec_Yaw_different_angle) , -180 , 180); //码盘解算出副云台与底盘之间的差角
	if(workStatus != INITIALIZE)
	{Gyro_usart_iwdg();}                      //给陀螺仪喂狗粮

	
//==============状态切换===========
	if(workStatus == INITIALIZE)
	{ 
		gimbal_work_mode=0;
		Gimbal_Init();         //云台初始化
	}
	else if(workStatus == AUTOATTACK)
	{
		MiniPC_Send_Data(1 , 0 , 12); //发送到小电脑（  0：蓝色 ，1：红色  ; 0：自瞄，1：打符    ；  当前机器人射速）		
		Gimbal_Auto();                //云台自动打击
	}
	else if(workStatus == AUTOBUFF)  
	{ 
		MiniPC_Send_Data(1 , 1 , 12); //发送到小电脑（  0：蓝色 ，1：红色  ; 0：自瞄，1：打符    ；  当前机器人射速）				
		Gimbal_Buff();                //云台大风车模式
	}
	else if(workStatus == REPLENISHMEN)
	{
		gimbal_work_mode=0;		
		Gimbal_Supply();              //云台补给模式
	}
	else if(workStatus == WORKING)
	{
		if(chassis_workStatus==TWIST_WAIST||chassis_workStatus==ROTATION/*INDEPENDENT*/)//扭腰或陀螺时换另一套参数
		{
			Gimbal_Work();              //云台正常工作			
//			Gimbal_Independent();
		}
		else
		{			
			Gimbal_Work();              //云台正常工作
		}
	}
	else
	{
		gimbal_work_mode=0;		
		Gimbal_Stop();                //停止
	}
#endif

	
/*输出云台电机*/
	
#ifdef gimbal_work
	
  #ifdef board_gimbal      //只有云台板和启动云台工作时才能控制PY轴
	    #ifdef double_gimbal
   	   CAN1_SendCommand_Gimbal_Pitch(gimbal_output[0],gimbal_output[2]);  //云台板can1控制P轴和副云台yaw	    
      #else	     
   	   CAN1_SendCommand_Gimbal_Pitch(gimbal_output[0],0);                 //云台板can1控制P轴和副云台yaw	 
      #endif	
	#endif 

	#ifdef board_chassis
	   #ifdef fire_work
     CAN1_SendCommand_Gimbal_Fire(gimbal_output[1],0,Fire_param.GD_output);//底盘板can1控制Y轴供弹	
	   #else
     CAN1_SendCommand_Gimbal_Fire(gimbal_output[1],0,0);        //底盘板can1控制Y轴供弹	
	   #endif
	#endif
	
#else	   
     gimbal_output[1]=0;
	   CAN1_SendCommand_Gimbal_Fire(0,0,0);
#endif
	
}	





/*===================================云台初始化==============================*/
int16_t yaw_init_success=0;           //Y轴初始化成功标志
static int16_t sec_yaw_init_success=0;//副Y轴初始化成功标志
static int16_t pitch_init_success=0;  //P轴初始化成功标志
static int16_t pitch_init_state=0;    //P轴初始化状态

//云台初始程序
static void Gimbal_Init(void)
{		
	//直接初始化，看需要加，不用直接去注释即可
		yaw_init_success   = 1;
		pitch_init_success = 1;
		sec_yaw_init_success = 1;
		
		
		Yaw_Init();    //Y轴初始化
		Sec_Yaw_Init();//副Y轴初始化
		Pitch_Init();  //P轴初始化
	
	if(yaw_init_success==1 && pitch_init_success==1 && sec_yaw_init_success==1)
	{
		yaw_init_success     = 0;
		sec_yaw_init_success = 0;
		pitch_init_success   = 0;
		pitch_init_state     = 0;
		INITIALIZE_flag      = 1;
		workStatus           = WORKING;   //开始运行
	}
}

//Yaw轴初始化
static void Yaw_Init(void)
{ 
	if(yaw_init_success == 0)  //Y轴初始化未成功
	{
    gimbal_output[1] = Rmmotor_Speed_control(&GIMBAL_S_YAW_PID, 900, Yaw_Motor_Data.speed, 1500);		//速度环给900目标值	
		if(YAW_ZERO_FLAG == 1)  //码盘接近中值
		{
			 yaw_init_success = 1;  		
		}
	} 	
  else
	{
		gimbal_output[1] = 0;
	}		
}

//Sec_Yaw轴初始化
static void Sec_Yaw_Init(void)
{ 
	if(sec_yaw_init_success == 0)  //副Y轴初始化未成功
	{
    gimbal_output[2] = Rmmotor_Speed_control(&GIMBAL_DOUBLE_S_YAW_PID, 850, Second_Yaw_Motor_Data.speed, 1000);	//速度环给850目标值		
		if(SEC_YAW_ZERO_FLAG==1)  //码盘接近中值
		{
			 sec_yaw_init_success = 1;  		
		}
	}
  else
	{
		gimbal_output[2] = 0; 
	}		
}
 	

//Pitch轴初始化  
static int16_t pitch_init_error = 0;   //P轴初始化累失败次数
static int16_t pitch_init_count = 0;   //P轴初始化云台归中消抖计数

static void Pitch_Init(void)
{
	if(pitch_init_success == 0) //P轴未初始化完成
	{
			if(pitch_init_state==0)   //P轴将副云台卡入卡扣
			{
					gimbal_output[0]=-5600; //速度开环
					if(Pitch_Motor_Data.actual_Position>=100) //副云台已卡入卡扣
					{
						pitch_init_state=1;
					}
			}
			
//			pitch_init_state=1;    //直接普通归中，不用副云台p轴

			if(pitch_init_state==1)//P轴回中
			{
				//位置环目标值为0
				 gimbal_output[0] = Motor_Position_Speed_Control(&GIMBAL_S_PITCH_PID , &GIMBAL_P_PITCH_PID , 0 , Pitch_Motor_Data.speed , (Pitch_Motor_Data.actual_Position*360/1024) , PITCH_OUTPUT_LIMIT); 
				
				if(abs(Pitch_Middle_Angle - Pitch_Motor_Data.position) < 20)  //码盘接近中值
				{
					if(pitch_init_count >= 50) //初始化消抖
					 { 
						 pitch_init_success = 1;  
						 pitch_init_count   = 0; //复位
						 pitch_init_error   = 0;
						 pitch_init_state   = 0;			
					 }
					 else
					 {
						 pitch_init_count++;     //成功次数累加
					 }
				}
				else
				{
					pitch_init_error++;        //初始化失败记录
				}
			 }
		
			if(pitch_init_error == 1500) //多次初始化失败，强制初始化成功
			{
				 pitch_init_success = 1;  
				 pitch_init_count   = 0;   //复位
				 pitch_init_error   = 0;	
				 pitch_init_state   = 0;			
			}    
  }
	else  //P轴初始化完成
	{
	  gimbal_output[0] = Motor_Position_Speed_Control(&GIMBAL_S_PITCH_PID , &GIMBAL_P_PITCH_PID , 0 , Pitch_Motor_Data.speed , (Pitch_Motor_Data.actual_Position*360/1024) , PITCH_OUTPUT_LIMIT); 
	}
}


/*===============================云台正常控制========================*/
first_order_low_filter_type_t  Gimbal_Updown_LowFilt_Pitch_Data = {0.0f,0.0f,0.0f,Gimbal_Pitch_Fir_Ord_Low_Fil_Param}; //低通俯仰控制量
//first_order_low_filter_type_t  Gimbal_Updown_LowFilt_Yaw_Data = {0.0f,0.0f,0.0f,Gimbal_Yaw_Fir_Ord_Low_Fil_Param};     //低通偏航控制量
//first_order_low_filter_type_t  Gimbal_Updown_LowFilt_Remote_data = {0.0f,0.0f,0.0f,Gimbal_Remote_Fir_Ord_Low_Fil_Param};//低通遥控量
//acceleration_control_type_t    Gimbal_Pitch_acc_limit_Data  = {0,0,0,0,GIMBAL_PITCH_ACCELERAD};  //P轴加速度限制                        
//acceleration_control_type_t    Gimbal_Yaw_acc_limit_Data  = {0,0,0,0,GIMBAL_YAW_ACCELERAD};    //Y轴加速度限制   
sliding_mean_filter_type_t    Gimbal_Updown_Slidmean_Pitch_Data ;                                //P轴滑动滤波器

int16_t pitch_filt_output=0;          //P轴滤波值
int16_t yaw_filt_output=0;            //Y轴滤波值
//int16_t remote_ch3_filt_output=0;     //遥控滤波值
int16_t pitch_real_jscope=0;   //P轴打印实际曲线
int16_t pitch_set_jscope=0;    //P轴打印设定曲线
float yaw_real_jscope=0;       //Y轴打印实际曲线
float yaw_set_jscope=0;        //Y轴打印设定曲线
float sec_yaw_set_jscope=0;    //副Y轴打印设定曲线
float sec_yaw_real_jscope=0;   //副Y轴打印实际曲线
float Sec_yaw_set_angle=0;     //副yaw轴设定转向角度
float Sec_yaw_angle=0;         //副yaw轴绝对角度
float Gimbal_set_position=0;   //主云台pid设定位置
static int8_t chassis_dance_flag=0;   //用于底盘陀螺时，副云台控制方式切换导致的绝对角度零点转换

static void Gimbal_Work(void)
{
	if(gimbal_work_mode != 1) //使用云台控制时刷新出来一个角度（防止切换模式角度变化）
	{
		Gimbal_yaw_angle=0;
		Last_Gimbal_yaw_angle = BMI160_Data.yaw_angle;
		gimbal_work_mode = 1;
		Sliding_Mean_Filter_Init(&Gimbal_Updown_Slidmean_Pitch_Data);  //初始化P轴滑动滤波器
		
		Remote_reload();
	}
  Gimbal_yaw_angle = BMI160_Data.yaw_angle-Last_Gimbal_yaw_angle;         //获取云台正常工作时的陀螺仪Y轴角度
 	Gimbal_Angle_Limit();                                                   //云台角度限制

  /*主云台控制*/
	Gimbal_set_position = loop_fp32_constrain((-Remote_data.RC_ch2-(YAW_ANGLE_FLAG*Gimbal_yaw_angle))  ,  -180.0f  ,  180.0f);  //云台设定位置循环限幅
 
	gimbal_output[0] = Sliding_Mean_Filter(&Gimbal_Updown_Slidmean_Pitch_Data , gimbal_output[0] , 55);     //均值滑窗滤波（有滞后）	
	gimbal_output[0] = First_Order_Low_Filter(&Gimbal_Updown_LowFilt_Pitch_Data , gimbal_output[0]); 

  gimbal_output[0] = Motor_Position_Speed_Control(&GIMBAL_S_PITCH_PID , &GIMBAL_P_PITCH_PID , 0 , /*Pitch_Motor_Data.speed*/BMI160_Data.Gyro_X , -((Remote_data.RC_ch3)-(Pitch_Motor_Data.actual_Position*360/1024)) , PITCH_OUTPUT_LIMIT); //-(Remote_data.RC_ch3-(Pitch_Motor_Data.actual_Position*360/1024))
	gimbal_output[1] = Motor_Position_Speed_Control(&GIMBAL_S_YAW_PID , &GIMBAL_P_YAW_PID , 0 , /*(YAW_GYRO_ANGULAR_SPEED)*/Yaw_Motor_Data.speed, Gimbal_set_position , YAW_OUTPUT_LIMIT);//Yaw位置控制
  

  /*副云台控制*/	
  #ifdef double_gimbal 
	Sec_yaw_set_angle = loop_fp32_constrain((Yaw_different_angle - Sec_Yaw_different_angle) , -180 , 180);
	Sec_yaw_angle     = loop_fp32_constrain((Gimbal_yaw_angle - Sec_Yaw_different_angle)    , -180 , 180); //副yaw轴绝对角度解算
	
	//Yaw控制
	  gimbal_output[2] = Motor_Position_Speed_Control(&GIMBAL_DOUBLE_S_YAW_PID , &GIMBAL_DOUBLE_P_YAW_PID , 0 , Second_Yaw_Motor_Data.speed, loop_fp32_constrain(-(Sec_Yaw_different_angle),-180,180) , 2000);  //副云台·闭环控制
	
	//Pitch控制
    //TIM3->CCR1=limit_int16(-((Pitch_Motor_Data.position-195)*30/130)+140 , 169 , 111);     //跟随主p轴                        
//	TIM3->CCR1=limit_int16((-(Remote_data.RC_sw/660)*30)+140 , 169 , 111);   //拨轮调试 (170-110)                           
     TIM3->CCR1=limit_int16((-(Remote_data.RC_ch3))+140 , 169 , 111);   //遥控ch3控制                           

	#endif 
	

	/*打印曲线*/
	pitch_real_jscope = gimbal_output[0];    //jscope观察曲线变化
//  pitch_set_jscope = (Pitch_Motor_Data.actual_Position*360/1024); 
//	yaw_real_jscope = YAW_ANGLE_FLAG*Gimbal_yaw_angle;    //jscope观察曲线变化
//  yaw_set_jscope = -Remote_data.RC_ch2;

}



/*==============================炮台模式云台==========================*/
static void Gimbal_Independent(void)
{
	if(gimbal_work_mode != 1) //使用云台控制时刷新出来一个角度（防止切换模式角度变化）
	{
		Gimbal_yaw_angle=0;
		Last_Gimbal_yaw_angle = BMI160_Data.yaw_angle;
		gimbal_work_mode = 1;
	}
  Gimbal_yaw_angle = BMI160_Data.yaw_angle-Last_Gimbal_yaw_angle;//获取云台正常工作时的陀螺仪Y轴角度
	
 	Gimbal_Angle_Limit();                                                   //云台角度限制


  gimbal_output[0] = Motor_Position_Speed_Control(&GIMBAL_S_PITCH_PID , &GIMBAL_P_PITCH_PID , 0 , Pitch_Motor_Data.speed , -((Remote_data.RC_ch3)-(Pitch_Motor_Data.actual_Position*360/1024)) , PITCH_OUTPUT_LIMIT); //-(Remote_data.RC_ch3-(Pitch_Motor_Data.actual_Position*360/1024))
	gimbal_output[1] = Motor_Position_Speed_Control(&GIMBAL_INDEPENDENT_S_YAW_PID , &GIMBAL_INDEPENDENT_P_YAW_PID , 0 ,Yaw_Motor_Data.speed, (-Remote_data.RC_ch2-(YAW_ANGLE_FLAG*Gimbal_yaw_angle)) , YAW_OUTPUT_LIMIT);//Yaw位置控制
 
	/*低通滤波*/
//	remote_ch3_filt_output = First_Order_Low_Filter(&Gimgimbal_outputbal_Updown_LowFilt_Remote_data , -(Remote_data.RC_ch3-(Pitch_Motor_Data.actual_Position*360/1024))); //Y轴输出量低通滤波  
//	yaw_filt_output = First_Order_Low_Filter(&Gimbal_Updown_LowFilt_Yaw_Data , gimbal_output[1]); //Y轴输出量低通滤波
//	pitch_filt_output = First_Order_Low_Filter(&Gimbal_Updown_LowFilt_Pitch_Data , [0]);  //P轴输出量低通滤波
//  pitch_filt_output = Motion_acceleration_control(&Gimbal_Pitch_acc_limit_Data , gimbal_output[0]); //P轴加速度限制
	
	/*打印曲线*/
	pitch_real_jscope = gimbal_output[0];    //jscope观察曲线变化
  pitch_set_jscope = (Pitch_Motor_Data.actual_Position*360/1024); 
	
//  gimbal_output[1] = yaw_filt_output;	
//  gimbal_output[0] = pitch_filt_output;	
}


/*==============================云台补给============================*/
static float Gimbal_supply_angle = 0;         //补给时角度
static float Gimbal_supply_angle_back = 0.0f; //补给返回角度

static void Gimbal_Supply(void)
{
	/*进入补给模式*/
	if(GIMBAL_SUPPLY_FLAG==0)
	{
		Gimbal_supply_angle = BMI160_Data.yaw_angle + 70.0f;  //补给时目标角度为60度
	  GIMBAL_SUPPLY_FLAG = 1;                               //进入补给状态
	}
	/*到补给角度*/
	else if(GIMBAL_SUPPLY_FLAG==1)  
	{
	  if(abs_float(BMI160_Data.yaw_angle - Gimbal_supply_angle) <= 3.0f) //到达指定位置
		{
			GIMBAL_SUPPLY_FLAG = 2;
			gimbal_output[1] = 0;
		  Gimbal_supply_angle_back = BMI160_Data.yaw_angle - 70.0f;  //归中时目标角度为-90度			
		}
		else
		{
			gimbal_output[1] = Motor_Position_Speed_Control(&GIMBAL_S_YAW_PID , &GIMBAL_P_YAW_PID , 0 , Yaw_Motor_Data.speed, (Gimbal_supply_angle-(YAW_ANGLE_FLAG*BMI160_Data.yaw_angle)) , 600);//Yaw位置控制 	
		}
	}
	/*返回中间角*/
	else if(GIMBAL_SUPPLY_FLAG==3)//（到位前提按下按键）
	{
	  gimbal_output[1] = Motor_Position_Speed_Control(&GIMBAL_S_YAW_PID , &GIMBAL_P_YAW_PID , 0 , Yaw_Motor_Data.speed, (Gimbal_supply_angle_back-(YAW_ANGLE_FLAG*BMI160_Data.yaw_angle)) , 600);//Yaw位置控制 			
	  if(abs_float(BMI160_Data.yaw_angle - Gimbal_supply_angle_back) <= 3.0f) //到达指定位置
		{
			GIMBAL_SUPPLY_FLAG = 4;
			Gimbal_supply_angle = 0;
			Gimbal_supply_angle_back = 0;
		}	  
	}
	
	 /*副云台控制*/
	 #ifdef double_gimbal
		Sec_yaw_set_angle = loop_fp32_constrain((Yaw_different_angle - Sec_Yaw_different_angle) , -180 , 180);
		Sec_yaw_angle     = loop_fp32_constrain((Remote_data.RC_ch2+(Gimbal_yaw_angle - Sec_Yaw_different_angle)) , -180 , 180); //副yaw轴绝对角度解算
	  gimbal_output[2] = Motor_Position_Speed_Control(&GIMBAL_DOUBLE_S_YAW_PID , &GIMBAL_DOUBLE_P_YAW_PID , 0 , Second_Yaw_Motor_Data.speed, (Sec_yaw_set_angle) , 2000);  //副云台·闭环控制

	//Pitch控制
   	TIM3->CCR1=limit_int16((-(Remote_data.RC_ch3))+140 , 169 , 111);                            
	 #endif
}



/*===============================云台自动瞄准==================================*/
static void Gimbal_Auto(void)
{
	if(gimbal_work_mode != 2) //使用云台控制时刷新出来一个角度（防止切换模式角度变化）
	{ 
		Gimbal_yaw_angle=0;
		Last_Gimbal_yaw_angle = BMI160_Data.yaw_angle;		
		gimbal_work_mode = 2;
		Change_camera(1);      //自瞄模式下切换摄像头参数为短焦相机
		Vision_Auto_Data.pitch_control_data = 0;
		Vision_Auto_Data.yaw_control_data   = 0;		
	}
	 /*预处理*/
   Gimbal_yaw_angle = BMI160_Data.yaw_angle-Last_Gimbal_yaw_angle;                                                             //获取云台自动打击时的陀螺仪Y轴角度
//	 Kalman_Filter_Calc(&kalman_Filter, Vision_Auto_Data.auto_yaw_angle+Gimbal_yaw_angle, Vision_Auto_Data.auto_pitch_angle);  //对小电脑传输角度进行卡尔曼滤波

   /*云台视觉控制量*/	//(y轴，p轴还没用卡尔曼)
	 Vision_Auto_Data.pitch_control_data = Vision_Auto_Data.auto_pitch_angle*1.7f; //    Vision_Auto_Data.auto_pitch_sum - (Pitch_Motor_Data.actual_Position*360/1024)
	 Vision_Auto_Data.yaw_control_data   = Vision_Auto_Data.auto_yaw_angle;        // - Vision_Auto_Data.auto_yaw_speed*9;    //14  auto_yaw_angle_kf第一个参数是kalman出来的yaw的角度，因为电机的控制，所以不用减去陀螺仪的角度，而是直接加上增益
	
   /*输出量闭环处理*/
//		if( Vision_Auto_Data.yaw_control_data==0 && Vision_Auto_Data.pitch_control_data==0) //视觉无数据返回时，y轴自旋
//		{
//			gimbal_output[1] = Rmmotor_Speed_control(&GIMBAL_S_YAW_PID, -900, Yaw_Motor_Data.speed, 1500);		
//		}
//		else  //识别到目标，自瞄
//		{
			gimbal_output[1] = Motor_Position_Speed_Control(&GIMBAL_AUTO_S_YAW_PID , &GIMBAL_AUTO_P_YAW_PID , 0 , /*(-YAW_GYRO_ANGULAR_SPEED)*/Yaw_Motor_Data.speed , (-Vision_Auto_Data.yaw_control_data) , YAW_OUTPUT_LIMIT);//Yaw位置控制
//		}
		
	 gimbal_output[0] = Motor_Position_Speed_Control(&GIMBAL_AUTO_S_PITCH_PID , &GIMBAL_AUTO_P_PITCH_PID , 0 , /*Pitch_Motor_Data.speed*/ BMI160_Data.Gyro_X , /*-*/(Vision_Auto_Data.pitch_control_data) , PITCH_OUTPUT_LIMIT); 

	 /*副云台控制*/
	 #ifdef double_gimbal
		Sec_yaw_set_angle = loop_fp32_constrain((Yaw_different_angle - Sec_Yaw_different_angle) , -180 , 180);
		Sec_yaw_angle     = loop_fp32_constrain((Remote_data.RC_ch2+(Gimbal_yaw_angle - Sec_Yaw_different_angle)) , -180 , 180); //副yaw轴绝对角度解算
	  gimbal_output[2] = Motor_Position_Speed_Control(&GIMBAL_DOUBLE_S_YAW_PID , &GIMBAL_DOUBLE_P_YAW_PID , 0 , Second_Yaw_Motor_Data.speed, (Sec_yaw_angle) , 2000);  //副云台·闭环控制

	//Pitch控制
   	TIM3->CCR1=limit_int16((-(Remote_data.RC_ch3))+140 , 169 , 111);                            
	
	 #endif


		/*打印曲线*/
		pitch_real_jscope = -gimbal_output[1];    //jscope观察曲线变化
		pitch_set_jscope  = -GIMBAL_AUTO_P_YAW_PID.SetValue; 
}


/*==================================云台打符模式==============================*/
float pitch_data_filt=0.0f;                                                //视觉p轴数据滤波值
float yaw_data_filt=0.0f;                                                //视觉y轴数据滤波值
float pitch_Kalman_filt=0.0f;                                                //视觉p轴卡尔曼测试值
first_order_low_filter_type_t pitch_low_filt  = {0.0f,0.0f,0.0f, 0.02642}; //一阶低通滤波器 0.01642
first_order_low_filter_type_t yaw_low_filt    = {0.0f,0.0f,0.0f, 0.05642}; //一阶低通滤波器 0.01642

sliding_mean_filter_type_t    pitch_mean_filt ;                            //滑动滤波器
extKalman_t                   pitch_kalman_filt;                           //卡尔曼滤波器

static void Gimbal_Buff(void) 
{
	if(gimbal_work_mode != 2) //模式切换处理
	{ 
		Gimbal_yaw_angle = 0;
		Last_Gimbal_yaw_angle = BMI160_Data.yaw_angle;//使用云台控制时刷新出来一个角度（防止切换模式角度变化）
		Vision_Auto_Data.pitch_control_data = 0;
		Vision_Auto_Data.yaw_control_data   = 0;
		gimbal_work_mode = 2;
		Change_camera(3);                            //打符模式下，切换摄像头为工业相机参数		
		Sliding_Mean_Filter_Init(&pitch_mean_filt);  //初始化P轴滑动滤波器
		KalmanCreate(&pitch_kalman_filt,0.001,0.02);     //初始化该滤波器的Q=20 R=200参数    //* @attention R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
                                                                                   //*		       	反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
		
	}
	 /*预处理*/
   Gimbal_yaw_angle = BMI160_Data.yaw_angle-Last_Gimbal_yaw_angle;                                                               //获取云台自动打击时的陀螺仪Y轴角度

   /*云台视觉控制量*/	//(y轴用了卡尔曼，p轴还没用)   //4.3
	 Vision_Auto_Data.pitch_control_data = Vision_Auto_Data.auto_pitch_angle*1.0f;  //1.93    Vision_Auto_Data.auto_pitch_sum - (Pitch_Motor_Data.actual_Position*360/1024)
	 Vision_Auto_Data.yaw_control_data   = Vision_Auto_Data.auto_yaw_angle*1.5f;    // - Vision_Auto_Data.auto_yaw_speed*14;    //auto_yaw_angle_kf第一个参数是kalman出来的yaw的角度，因为电机的控制，所以不用减去陀螺仪的角度，而是直接加上增益

	 //滤波p轴
//   Data_Accelerated_Control(&Vision_Auto_Data.pitch_control_data , 3.6f);	//2.6                                                //斜坡加速度限制函数
//   Vision_Auto_Data.pitch_control_data = Sliding_Mean_Filter(&pitch_mean_filt , Vision_Auto_Data.pitch_control_data , 35);     //均值滑窗滤波（有滞后）	
//  	 pitch_data_filt = First_Order_Low_Filter(&pitch_low_filt , Vision_Auto_Data.pitch_control_data);                            //一阶低通滤波

	//	 pitch_Kalman_filt = pitch_data_filt;
//	 pitch_data_filt = KalmanFilter(&pitch_kalman_filt,pitch_data_filt);                                                         //卡尔曼滤波
//	 Kalman_Filter_Calc(&kalman_Filter, Vision_Auto_Data.auto_yaw_angle+Gimbal_yaw_angle, Vision_Auto_Data.auto_pitch_angle);    //二阶卡尔曼滤波

	  //滤波y轴
	 yaw_data_filt = First_Order_Low_Filter(&yaw_low_filt , Vision_Auto_Data.yaw_control_data);                            //一阶低通滤波

	
   /*输出量闭环处理*/
	 gimbal_output[1] = Motor_Position_Speed_Control(&GIMBAL_AUTO_BUFF_S_YAW_PID ,
                                                   &GIMBAL_AUTO_BUFF_P_YAW_PID ,
                                                   0 , 
	                                                 Yaw_Motor_Data.speed , //BMI160_Data.Gyro_Z   Yaw_Motor_Data.speed
                                                	 (-yaw_data_filt) , 
	                                                 YAW_OUTPUT_LIMIT);//Yaw位置控制
	
   gimbal_output[0] = Motor_Position_Speed_Control(&GIMBAL_AUTO_BUFF_S_PITCH_PID , 
	                                                 &GIMBAL_AUTO_BUFF_P_PITCH_PID ,
                                                   0 ,
                                                   Pitch_Motor_Data.speed ,
                         	                         Vision_Auto_Data.pitch_control_data,//Vision_Auto_Data.pitch_control_data,/*-(Pitch_Motor_Data.actual_Position*360/1024)*/ //(Remote_data.RC_ch3)//没有检测到目标时p轴归中
                                                   PITCH_OUTPUT_LIMIT); 
									 		 														 
	
	 /*副云台控制*/
		#ifdef double_gimbal
		Sec_yaw_set_angle = loop_fp32_constrain((Yaw_different_angle - Sec_Yaw_different_angle) , -180 , 180);
	  gimbal_output[2] = Motor_Position_Speed_Control(&GIMBAL_DOUBLE_S_YAW_PID , &GIMBAL_DOUBLE_P_YAW_PID , 0 , Second_Yaw_Motor_Data.speed, (Sec_yaw_set_angle) , 2000);  //副云台·闭环控制
    #endif
	 
		/*打印曲线*/
		pitch_real_jscope = -gimbal_output[1];    //jscope观察曲线变化
		pitch_set_jscope = -GIMBAL_AUTO_P_YAW_PID.SetValue; 	
}


/*==================================云台无力==================================*/
static void Gimbal_Stop(void)
{
   gimbal_output[0] = 0;  //P
	 gimbal_output[1] = 0;  //Y
	 gimbal_output[2] = 0;
	 gimbal_output[3] = 0;
}


/******************控制算法************************/

/*
* 功能：云台的角度限制
* 输入：遥控值结构体
* 输出：无
* 描述：限制处理后的控制量
*/
static void Gimbal_Angle_Limit(void)
{
	//pitch角度限制
	Remote_data.RC_ch3 = limit_float(Remote_data.RC_ch3, PITCH_ANGLE_LIMIT_UP, PITCH_ANGLE_LIMIT_DOWN);  
  //yaw角度限制 
//	Remote_data.RC_ch2 = limit_float(Remote_data.RC_ch2, YAW_ANGLE_LIMIT, -YAW_ANGLE_LIMIT);
}

/******************************************/



/*
* 功能：Yaw轴电机码盘位置的光电校正
* 输入：无
* 输出：无
* 描述：云台工作过程中运行次程序会调用检测Y中值光电程序，修正Y电机码盘真实值，中间为0
*/
static int16_t correctYawValue[5]={0};
static void Correct_Yaw_Zero(void)
{
	u8 i=0;
	u16 correctSum=0;
	
	for(i=0;i<5;i++)
	{
		correctYawValue[i-1]=correctYawValue[i];
	}
	correctYawValue[4] = !YAW_ZERO_FLAG;//Yaw_Zero_Value();
	
	for(i=0;i<5;i++)
	{
		correctSum+=correctYawValue[i];
	}
	
	correctSum/=5;
	
	if(correctSum==0)
	{
		Yaw_Motor_Data.actual_Position=0;
	}
}


/*
* 功能：副Yaw轴电机码盘位置的光电校正
* 输入：无
* 输出：无
* 描述：云台工作过程中运行次程序会调用检测Y中值光电程序，修正Y电机码盘真实值，中间为0
*/
static int16_t correctSecYawValue[5]={0};
static void Correct_Sec_Yaw_Zero(void)
{
	u8 i=0;
	u16 correctSum=0;
	
	for(i=0;i<5;i++)
	{
		correctSecYawValue[i-1]=correctSecYawValue[i];
	}
	correctSecYawValue[4] = !SEC_YAW_ZERO_FLAG;//Yaw_Zero_Value();
	
	for(i=0;i<5;i++)
	{
		correctSum+=correctSecYawValue[i];
	}
	
	correctSum/=5;
	
	if(correctSum==0)
	{
		Second_Yaw_Motor_Data.actual_Position=0;
	}
}




/*
*功能：通过接收的小电脑相机代号改变pid的参数值
*输入：相机代号（1：短焦  2：长焦  3：工业）
*输出：无
*/

static void Change_camera(int camera_num)
{
 if(camera_num==1)
 {
  //Auto_Pitch
  GIMBAL_AUTO_P_PITCH_PID.kp=GIMBAL_AUTO_SHORT_P_PITCH_P;
	GIMBAL_AUTO_P_PITCH_PID.ki=GIMBAL_AUTO_SHORT_P_PITCH_I;
	GIMBAL_AUTO_P_PITCH_PID.kd=GIMBAL_AUTO_SHORT_P_PITCH_D;
	GIMBAL_AUTO_P_PITCH_PID.Error=0;
	GIMBAL_AUTO_P_PITCH_PID.iError=0;
	GIMBAL_AUTO_P_PITCH_PID.LastError=0;
	GIMBAL_AUTO_P_PITCH_PID.OutputValue=0;
	GIMBAL_AUTO_P_PITCH_PID.outputValue_Int32=0;
	GIMBAL_AUTO_P_PITCH_PID.LastOutput=0;
	GIMBAL_AUTO_P_PITCH_PID.SetValue=0;
	
	GIMBAL_AUTO_S_PITCH_PID.kp=GIMBAL_AUTO_SHORT_S_PITCH_P;
	GIMBAL_AUTO_S_PITCH_PID.ki=GIMBAL_AUTO_SHORT_S_PITCH_I;
	GIMBAL_AUTO_S_PITCH_PID.kd=GIMBAL_AUTO_SHORT_S_PITCH_D;
	GIMBAL_AUTO_S_PITCH_PID.Error=0;
	GIMBAL_AUTO_S_PITCH_PID.iError=0;	
	GIMBAL_AUTO_S_PITCH_PID.LastError=0;
	GIMBAL_AUTO_S_PITCH_PID.PrevError=0;
	GIMBAL_AUTO_S_PITCH_PID.OutputValue=0;
	GIMBAL_AUTO_S_PITCH_PID.outputValue_Int32=0;
	GIMBAL_AUTO_S_PITCH_PID.LastOutput=0;
	GIMBAL_AUTO_S_PITCH_PID.SetValue=0;
	
	//Auto_Yaw
	GIMBAL_AUTO_P_YAW_PID.kp=GIMBAL_AUTO_SHORT_P_YAW_P;
	GIMBAL_AUTO_P_YAW_PID.ki=GIMBAL_AUTO_SHORT_P_YAW_I;
	GIMBAL_AUTO_P_YAW_PID.kd=GIMBAL_AUTO_SHORT_P_YAW_D;
	GIMBAL_AUTO_P_YAW_PID.Error=0;
	GIMBAL_AUTO_P_YAW_PID.iError=0;	
	GIMBAL_AUTO_P_YAW_PID.LastError=0;
	GIMBAL_AUTO_P_YAW_PID.OutputValue=0;
	GIMBAL_AUTO_P_YAW_PID.outputValue_Int32=0;
	GIMBAL_AUTO_P_YAW_PID.LastOutput=0;
	GIMBAL_AUTO_P_YAW_PID.SetValue=0;
	
	GIMBAL_AUTO_S_YAW_PID.kp=GIMBAL_AUTO_SHORT_S_YAW_P;
	GIMBAL_AUTO_S_YAW_PID.ki=GIMBAL_AUTO_SHORT_S_YAW_I;
	GIMBAL_AUTO_S_YAW_PID.kd=GIMBAL_AUTO_SHORT_S_YAW_D;
	GIMBAL_AUTO_S_YAW_PID.Error=0;
	GIMBAL_AUTO_S_YAW_PID.iError=0;
	GIMBAL_AUTO_S_YAW_PID.LastError=0;
	GIMBAL_AUTO_S_YAW_PID.PrevError=0;
	GIMBAL_AUTO_S_YAW_PID.OutputValue=0;
	GIMBAL_AUTO_S_YAW_PID.outputValue_Int32=0;
	GIMBAL_AUTO_S_YAW_PID.LastOutput=0;
	GIMBAL_AUTO_S_YAW_PID.SetValue=0;
  }
  else if(camera_num==2)
	{
  //Auto_Pitch
  GIMBAL_AUTO_P_PITCH_PID.kp=GIMBAL_AUTO_LONG_P_PITCH_P;
	GIMBAL_AUTO_P_PITCH_PID.ki=GIMBAL_AUTO_LONG_P_PITCH_I;
	GIMBAL_AUTO_P_PITCH_PID.kd=GIMBAL_AUTO_LONG_P_PITCH_D;
	GIMBAL_AUTO_P_PITCH_PID.Error=0;
	GIMBAL_AUTO_P_PITCH_PID.iError=0;
	GIMBAL_AUTO_P_PITCH_PID.LastError=0;
	GIMBAL_AUTO_P_PITCH_PID.OutputValue=0;
	GIMBAL_AUTO_P_PITCH_PID.outputValue_Int32=0;
	GIMBAL_AUTO_P_PITCH_PID.LastOutput=0;
	GIMBAL_AUTO_P_PITCH_PID.SetValue=0;
	
	GIMBAL_AUTO_S_PITCH_PID.kp=GIMBAL_AUTO_LONG_S_PITCH_P;
	GIMBAL_AUTO_S_PITCH_PID.ki=GIMBAL_AUTO_LONG_S_PITCH_I;
	GIMBAL_AUTO_S_PITCH_PID.kd=GIMBAL_AUTO_LONG_S_PITCH_D;
	GIMBAL_AUTO_S_PITCH_PID.Error=0;
	GIMBAL_AUTO_S_PITCH_PID.iError=0;	
	GIMBAL_AUTO_S_PITCH_PID.LastError=0;
	GIMBAL_AUTO_S_PITCH_PID.PrevError=0;
	GIMBAL_AUTO_S_PITCH_PID.OutputValue=0;
	GIMBAL_AUTO_S_PITCH_PID.outputValue_Int32=0;
	GIMBAL_AUTO_S_PITCH_PID.LastOutput=0;
	GIMBAL_AUTO_S_PITCH_PID.SetValue=0;
	
	//Auto_Yaw
	GIMBAL_AUTO_P_YAW_PID.kp=GIMBAL_AUTO_LONG_P_YAW_P;
	GIMBAL_AUTO_P_YAW_PID.ki=GIMBAL_AUTO_LONG_P_YAW_I;
	GIMBAL_AUTO_P_YAW_PID.kd=GIMBAL_AUTO_LONG_P_YAW_D;
	GIMBAL_AUTO_P_YAW_PID.Error=0;
	GIMBAL_AUTO_P_YAW_PID.iError=0;	
	GIMBAL_AUTO_P_YAW_PID.LastError=0;
	GIMBAL_AUTO_P_YAW_PID.OutputValue=0;
	GIMBAL_AUTO_P_YAW_PID.outputValue_Int32=0;
	GIMBAL_AUTO_P_YAW_PID.LastOutput=0;
	GIMBAL_AUTO_P_YAW_PID.SetValue=0;
	
	GIMBAL_AUTO_S_YAW_PID.kp=GIMBAL_AUTO_LONG_S_YAW_P;
	GIMBAL_AUTO_S_YAW_PID.ki=GIMBAL_AUTO_LONG_S_YAW_I;
	GIMBAL_AUTO_S_YAW_PID.kd=GIMBAL_AUTO_LONG_S_YAW_D;
	GIMBAL_AUTO_S_YAW_PID.Error=0;
	GIMBAL_AUTO_S_YAW_PID.iError=0;
	GIMBAL_AUTO_S_YAW_PID.LastError=0;
	GIMBAL_AUTO_S_YAW_PID.PrevError=0;
	GIMBAL_AUTO_S_YAW_PID.OutputValue=0;
	GIMBAL_AUTO_S_YAW_PID.outputValue_Int32=0;
	GIMBAL_AUTO_S_YAW_PID.LastOutput=0;
	GIMBAL_AUTO_S_YAW_PID.SetValue=0;
  }
	else if(camera_num==3)
	{
  //Auto_Pitch
  GIMBAL_AUTO_P_PITCH_PID.kp=GIMBAL_AUTO_INDUSTRY_P_PITCH_P;
	GIMBAL_AUTO_P_PITCH_PID.ki=GIMBAL_AUTO_INDUSTRY_P_PITCH_I;
	GIMBAL_AUTO_P_PITCH_PID.kd=GIMBAL_AUTO_INDUSTRY_P_PITCH_D;
	GIMBAL_AUTO_P_PITCH_PID.Error=0;
	GIMBAL_AUTO_P_PITCH_PID.iError=0;
	GIMBAL_AUTO_P_PITCH_PID.LastError=0;
	GIMBAL_AUTO_P_PITCH_PID.OutputValue=0;
	GIMBAL_AUTO_P_PITCH_PID.outputValue_Int32=0;
	GIMBAL_AUTO_P_PITCH_PID.LastOutput=0;
	GIMBAL_AUTO_P_PITCH_PID.SetValue=0;
	
	GIMBAL_AUTO_S_PITCH_PID.kp=GIMBAL_AUTO_INDUSTRY_S_PITCH_P;
	GIMBAL_AUTO_S_PITCH_PID.ki=GIMBAL_AUTO_INDUSTRY_S_PITCH_I;
	GIMBAL_AUTO_S_PITCH_PID.kd=GIMBAL_AUTO_INDUSTRY_S_PITCH_D;
	GIMBAL_AUTO_S_PITCH_PID.Error=0;
	GIMBAL_AUTO_S_PITCH_PID.iError=0;	
	GIMBAL_AUTO_S_PITCH_PID.LastError=0;
	GIMBAL_AUTO_S_PITCH_PID.PrevError=0;
	GIMBAL_AUTO_S_PITCH_PID.OutputValue=0;
	GIMBAL_AUTO_S_PITCH_PID.outputValue_Int32=0;
	GIMBAL_AUTO_S_PITCH_PID.LastOutput=0;
	GIMBAL_AUTO_S_PITCH_PID.SetValue=0;
	
	//Auto_Yaw
	GIMBAL_AUTO_P_YAW_PID.kp=GIMBAL_AUTO_INDUSTRY_P_YAW_P;
	GIMBAL_AUTO_P_YAW_PID.ki=GIMBAL_AUTO_INDUSTRY_P_YAW_I;
	GIMBAL_AUTO_P_YAW_PID.kd=GIMBAL_AUTO_INDUSTRY_P_YAW_D;
	GIMBAL_AUTO_P_YAW_PID.Error=0;
	GIMBAL_AUTO_P_YAW_PID.iError=0;	
	GIMBAL_AUTO_P_YAW_PID.LastError=0;
	GIMBAL_AUTO_P_YAW_PID.OutputValue=0;
	GIMBAL_AUTO_P_YAW_PID.outputValue_Int32=0;
	GIMBAL_AUTO_P_YAW_PID.LastOutput=0;
	GIMBAL_AUTO_P_YAW_PID.SetValue=0;
	
	GIMBAL_AUTO_S_YAW_PID.kp=GIMBAL_AUTO_INDUSTRY_S_YAW_P;
	GIMBAL_AUTO_S_YAW_PID.ki=GIMBAL_AUTO_INDUSTRY_S_YAW_I;
	GIMBAL_AUTO_S_YAW_PID.kd=GIMBAL_AUTO_INDUSTRY_S_YAW_D;
	GIMBAL_AUTO_S_YAW_PID.Error=0;
	GIMBAL_AUTO_S_YAW_PID.iError=0;
	GIMBAL_AUTO_S_YAW_PID.LastError=0;
	GIMBAL_AUTO_S_YAW_PID.PrevError=0;
	GIMBAL_AUTO_S_YAW_PID.OutputValue=0;
	GIMBAL_AUTO_S_YAW_PID.outputValue_Int32=0;
	GIMBAL_AUTO_S_YAW_PID.LastOutput=0;
	GIMBAL_AUTO_S_YAW_PID.SetValue=0;		
	}
 }

 
 
 
 /*
*功能：解算出双云台以及底盘的相对角度
*输入：1 主Yaw轴绝对角度()   2 副yaw轴绝对角度()  3 主yaw轴相对底盘差角（+-180）   4 副yaw轴相对主yaw轴差角（+-180）
*输出：无
*/
 
 