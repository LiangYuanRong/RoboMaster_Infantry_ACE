#include "main.h"
#include "maths.h"
#include "Remote_Task.h"
#include "remote.h"
#include "Speed_Def.h"
#include "Mode_Def.h"
#include "Gimbal_Task.h"
#include "Fire_Task.h"
#include "Chassis_Task.h"
#include "Capacitor_control.h"

/*****************************操作键位说明书**********************************************************/
/* 一、遥控模式：
         1.底盘跟随  ：左中右上
				 2.扭腰模式  ：左下右上
				 3.底盘小陀螺：左上右上
				 4.打符模式  ：左上右中（底盘以云台坐标轴移动，不跟随）
				 5.自瞄模式  ：左下右中
				 6.键盘模式  ：左中右中
		     7.发弹      ：在初始化完成且右开关打上或中情况下，左上角波轮拉最上发弹，回中停止射弹
				 8.关机      ：右下				 
	 二、键鼠模式：
	       1.基本运动：WASD
				 2.云台运动：鼠标
				 3.发射：    鼠标左键单点单发，按住连发
				 4.加速（电容放电）：    按住shift（逮虾户）
				 5.扭腰模式： F  （按一下进入，再按一下返回）
				 6.自瞄模式： 鼠标右键  （按住）
				 7.补给模式：  G  （按一下云台往一边转，补完弹再按一下回来）
				 8.高射速模式：C （按一次进入）（这个模式没用了，之前全部最大射速打）
				 9.低射速模式：V （按一次进入）
				 10.退弹模式： Z  （按住）
				 11.炮台模式： Ctrl （按住，只能控制云台，底盘不动）
				 12.打符模式： X （一次进入）
				 13.小陀螺模式：R（按一次进入）
                                                                                                    */
/****************************************************************************************************/
/*
   部分简写：RC：remote contral   遥控器
            MK: mouse key        键鼠
*/


/*遥控器或键鼠的处理值*/
RC_Deal_data        Remote_data;         
/*堆栈剩余*/
extern TaskHandle_t RemoteTask_Handler; // 栈大小


/*
*功能：遥控任务
*/
int mode_heart=0;    //遥控任务心跳
//int RemoteTask_water=0;

void Remote_Task(void *pvParameters)
{
	//加载时间
    vTaskDelay(REMOTE_TASK_INIT_TIME);
	
	while(1)
	{
		mode_heart=!mode_heart;       //遥控任务心跳
		LEDE0 = mode_heart;           //A板测试用
//		RemoteTask_water = uxTaskGetStackHighWaterMark(RemoteTask_Handler);//检测剩余任务堆栈
		
		Select_Ctl_Mode();            //遥控数据处理
		//任务周期
    vTaskDelay(REMOTE_CONTROL_TIME_MS);
	}
}


	
		


/*
*功能：遥控选择模式函数
*/

static u8 control_mode=0;                  //该变量为记录该次遥控模式状态（s2）
extern int8_t INITIALIZE_flag;             //发送给底盘的初始化成功标志
extern int gimbal_work_mode;               //云台状态切换控制量处理标志位

void Select_Ctl_Mode()
{
	if(RC_Ctl.rc.s2 == RC_CONTROL)             //右开关打上，遥控控制
	{
				 if(control_mode==2)                     //上一次状态为s2打中，更新机器人状态
				{ 
					chassis_workStatus = FOLLOWING;        //底盘默认工作模式为跟随
					Remote_reload();                       //摇杆量清零			
				}	
				
				if(workStatus == POWEROFF)//之前的状态为待机
				{
					workStatus = INITIALIZE;               //状态设置为初始化				
					Remote_reload();                       //摇杆量清零
				}
				else if(workStatus == AUTOATTACK || workStatus == AUTOBUFF)//之前状态为自动打击或打符模式
				{
					workStatus=WORKING;     //状态设置为工作
				}
				else
				{
						if(workStatus != INITIALIZE)//状态不为初始化
						{
							RC_Data_Process();     //遥控器数据处理
                  
						 /*开关切换模式*/
									if(RC_Ctl.rc.s1 == RC_SW_UP)//s1（左上）打到上
									{
										chassis_workStatus=ROTATION;//底盘小陀螺模式   ROTATION
									}
									if(RC_Ctl.rc.s1 == RC_SW_MID)//s1（左上）打到中间       //平常遥控状态
									{
										chassis_workStatus=FOLLOWING;//底盘跟随模式					
									}
									if(RC_Ctl.rc.s1 == RC_SW_DOWN)//s1（左上）打到最下
									{
										chassis_workStatus=TWIST_WAIST;//底盘扭腰模式   TWIST_WAIST
									}
									
						/*发射状态*/
									/*拨轮控制射弹（上是高射速，下是低射速，中间是停止开火）*/
									if(Remote_data.RC_sw >= 500)
									{
										fire_workstatus = FIRE;              //拨轮上  开火
										shoot_workstatus = HIGH_SPEED;       //摩擦轮模式高速模式 			
									} 
									if(Remote_data.RC_sw <= -500)
									{
										fire_workstatus = FIRE;              //拨轮上  开火	
										shoot_workstatus = LOW_SPEED;        //摩擦轮模式低速模式 				
									}
									else
									{
										fire_workstatus = STOP_FIRE;         //拨轮中间  停止开火
										shoot_workstatus = LOW_SPEED;        //摩擦轮模式低速模式 							
									}	
									
							
						}
				}
				control_mode=1;         //s2归1		
	}
	if(RC_Ctl.rc.s2 == MOUSE_CONTROL)  //右开关打中，键盘控制
	{
				 if(control_mode==1||control_mode==0)  //上一次状态为s2打上,更新机器人状态
				{
					if(control_mode==0)
							workStatus = POWEROFF;               //默认工作模式为待机模式
					chassis_workStatus = FOLLOWING;          //底盘默认工作模式为跟随
					gimbal_work_mode=0;                  
					
					Remote_reload();                         //摇杆量清零
				}
				
				if(workStatus == POWEROFF)    //之前的状态为关电
				{
					workStatus = INITIALIZE;    //状态设置为初始化
					Remote_reload();            //摇杆量清零					 
				}
				else
				{
							if(workStatus != INITIALIZE)//状态不为初始化
							{
									if(RC_Ctl.rc.s1 == 1)      //左上右中  打符模式
									{ 
										RC_Data_Process();       //遥控器数据处理
										workStatus = AUTOBUFF;   //打符模式 
										chassis_workStatus = INDEPENDENT; //底盘不跟随	
									
									}
									if(RC_Ctl.rc.s1 == 2)     //左下右中  自动打击
									{
										RC_Data_Process();       //遥控器数据处理					
										workStatus = AUTOATTACK; //自动打击模式
									
										/*发射状态*/
												/*拨轮控制射弹（上是高射速，下是低射速，中间是停止开火）*/
												if(Remote_data.RC_sw >= 500)
												{
													fire_workstatus = FIRE;              //拨轮上  开火
													shoot_workstatus = HIGH_SPEED;       //摩擦轮模式高速模式 			
												} 
												if(Remote_data.RC_sw <= -500)
												{
													fire_workstatus = FIRE;              //拨轮上  开火	
													shoot_workstatus = LOW_SPEED;        //摩擦轮模式低速模式 				
												}
												else
												{
													fire_workstatus = STOP_FIRE;         //拨轮中间  停止开火
													shoot_workstatus = LOW_SPEED;        //摩擦轮模式低速模式 							
												}	
									
									}
									if(RC_Ctl.rc.s1 == 3)      //左中右中   键鼠模式
									{
										MK_Data_Process();       //键鼠控制
									}
												
							}
				}
				control_mode=2;          //s2归2
	}
	if(RC_Ctl.rc.s2 == ALL_STOP)       //右开关打下，停止工作
	{
		workStatus = POWEROFF;   //进入待机模式
		INITIALIZE_flag = 0;     //初始化标志位清零
		control_mode=0;          //s2归0
		Remote_reload();         //摇杆量清零
		
	}
	if(RC_Ctl.rc.s2 == 0)
	{
		workStatus = POWEROFF;   //进入待机模式
		INITIALIZE_flag = 0;     //初始化标志位清零
		Remote_reload();         //摇杆量清零
		
	}
	
}


/*
*功能：遥控器数据处理
*取值：来源于remote文件里的遥控值获取
*/
void RC_Data_Process(void)    
{
	/*遥控器滤波消抖*/
	
	//遥控器获取的值在364和1684之间，错误宽容为350到1700
	if((RC_Ctl.rc.ch0<350||RC_Ctl.rc.ch0>1700)||(RC_Ctl.rc.ch1<350||RC_Ctl.rc.ch1>1700)||(RC_Ctl.rc.ch2<350||RC_Ctl.rc.ch2>1700)||(RC_Ctl.rc.ch3<350||RC_Ctl.rc.ch3>1700))
	{
		RC_Ctl.rc.ch0 = RC_Ctl.rc.ch1 = RC_Ctl.rc.ch2 = RC_Ctl.rc.ch3 =1024;
	}
	//消抖值为10
		if(abs(RC_Ctl.rc.ch0 - 1024)<10)
			RC_Ctl.rc.ch0=1024;
		if(abs(RC_Ctl.rc.ch1 - 1024)<10)
			RC_Ctl.rc.ch1=1024;
		if(abs(RC_Ctl.rc.ch2 - 1024)<10)
			RC_Ctl.rc.ch2=1024;
		if(abs(RC_Ctl.rc.ch3 - 1024)<10)
			RC_Ctl.rc.ch3=1024;
	
	/*将遥控器值赋给遥控量*/
		if(workStatus == WORKING || workStatus == AUTOATTACK || workStatus == AUTOBUFF)  //手动控制
		{
			Remote_data.RC_ch0 = (RC_Ctl.rc.ch0 - 1024);
			Remote_data.RC_ch1 = (RC_Ctl.rc.ch1 - 1024); 
			Remote_data.RC_ch2 += (RC_Ctl.rc.ch2 - 1024)*RC_YAW_SPEED;   //Y轴位置环量累加
			   Remote_data.RC_ch2 = loop_fp32_constrain(Remote_data.RC_ch2 , -180.0f , 180.0f);//循环限幅
			Remote_data.RC_ch3 +=(RC_Ctl.rc.ch3 - 1024)*RC_PITCH_SPEED;   //P轴位置环量累加
			Remote_data.RC_sw  = -(RC_Ctl.rc.sw - 1024);     //遥控器拨轮值，中值为1024，上下限两万多。处理后中值为0
		}

		else  //待机模式，初始化模式，补给模式  均不可手动控制
		{
			Remote_reload();                       //摇杆量清零  
		}
	
}


/*
*功能：键盘数据处理
*取值：来源于remote文件里的遥控值获取
*       &RC_Ctl.mouse    鼠标
*       &RC_Ctl.key      键盘
*/
Key_Press_Sigh Key_Press = {0};       //键盘按键按下记录

extern int8_t GIMBAL_SUPPLY_FLAG;     //补给状态标志位(0:归中状态，进入补给状态  1:云台90度转动中  2:到达指定位置，归中中)
		   int8_t CHASSIS_SUPPLY_FLAG = 0;//用于底盘补给状态模式切换
int8_t  Vision_flag = 0;              //视觉模式切换标志（0：关闭，1：自瞄，2：能量机关）
static int8_t  Shoot_flag = 0;        //拨弹轮状态切换标志
static int8_t  Independent_flag=0;    //底盘独立切换标志

static int16_t Speed_Straight_ACC = 0;       //前后移动键盘模拟斜坡加速
static int16_t Speed_Across_ACC = 0;         //左右平移键盘模拟斜坡加速

void MK_Data_Process(void)
{
/**********键盘模式选择**********/
	
	//**F  扭腰模式
		if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_F)     //F  扭腰模式
		{
				if(Key_Press.KEY_F == 0)
				{
					if(chassis_workStatus == FOLLOWING)
					{	
					  chassis_workStatus=TWIST_WAIST;      //底盘扭腰模式
					}
					else
					{
						chassis_workStatus = FOLLOWING;
					}
					Key_Press.KEY_F = 1;
				}
		}
		else
		{
			Key_Press.KEY_F = 0;			
		}
		
		


		//**R  小陀螺模式
		if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_R)     //R  小陀螺模式
		{
				if(Key_Press.KEY_R == 0)
				{
					if(chassis_workStatus == FOLLOWING)
					{	
						chassis_workStatus = ROTATION;
					}
					else
					{
						chassis_workStatus = FOLLOWING;
					}
					Key_Press.KEY_R = 1;
				}
		} 
		else
		{
			Key_Press.KEY_R = 0;			
		}



		//**Ctrl（唱跳rap篮球）   炮台模式
		if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_CTRL) //Ctrl  炮台模式（按住）
	  {
			Independent_flag = 1;
			chassis_workStatus = INDEPENDENT;
	  }
		else if(Independent_flag==1)
		{
			Independent_flag=0;
			chassis_workStatus = FOLLOWING;
		}
	 
		

		//**G  补给模式
		
#ifdef board_gimbal//云台板补给逻辑		
		if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_G)    //G  补给模式
		{
				if(Key_Press.KEY_G == 0)            //按下
				{
					if(workStatus != REPLENISHMEN)  //开启补给
					{
						workStatus=REPLENISHMEN;
						chassis_workStatus=STOP; 
					}
					else if(workStatus == REPLENISHMEN && GIMBAL_SUPPLY_FLAG==2) //退出补给（正在补给模式中且已到达目标角度）
					{
             GIMBAL_SUPPLY_FLAG = 3;
					}
					Key_Press.KEY_G = 1;
				}
		}
		else
		{
			if(GIMBAL_SUPPLY_FLAG == 4)
			{
				GIMBAL_SUPPLY_FLAG = 0;   //松开按键回复原状态
				workStatus = WORKING;
				chassis_workStatus = FOLLOWING;
			}
			Key_Press.KEY_G = 0;			
		}
		
#else //底盘板补给逻辑
   	if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_G)    //G  补给模式
		{
				if(Key_Press.KEY_G == 0)            //按下
				{
						chassis_workStatus=STOP; 
						CHASSIS_SUPPLY_FLAG = 1;
						GIMBAL_SUPPLY_FLAG = 1;
			  		Key_Press.KEY_G = 1;
				}
		}
		else
		{
			if(CHASSIS_SUPPLY_FLAG==1 && (GIMBAL_SUPPLY_FLAG == 4 || GIMBAL_SUPPLY_FLAG == 0))  //因为发送频率原因，有时云台未来得及发送4的标志位就变成了0，所以两个都以及加判断
			{
				GIMBAL_SUPPLY_FLAG = 0;   //云台补给动作完成
				CHASSIS_SUPPLY_FLAG = 0;  //底盘解除补给状态
				chassis_workStatus = FOLLOWING;
			}
			Key_Press.KEY_G = 0;			
		}	
#endif
		
	
		
	 //**C  高射速模式
		if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_C)    //C		高射速模式
		{
				if(Key_Press.KEY_C == 0)
			{
          if(shoot_workstatus == LOW_SPEED)
					{
						shoot_workstatus=HIGH_SPEED;
					}
					Key_Press.KEY_C = 1;
				}
		}
		else
		{
			Key_Press.KEY_C  = 0;
		}		
	
		
	 //**V  低射速模式
		if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_V)    //V		低射速模式
		{
				if(Key_Press.KEY_V == 0)
			{
          if(shoot_workstatus == HIGH_SPEED)
					{
						shoot_workstatus=LOW_SPEED;
					}
					Key_Press.KEY_V = 1;
				}
		}
		else
		{
			Key_Press.KEY_V  = 0;
		}	
	
		

		
	//**X  打符模式
#ifdef board_gimbal	//云台板打符模式
		if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_X)    
		{
				if(Key_Press.KEY_X == 0)
				{
					if(workStatus != AUTOBUFF)
					{
						workStatus = AUTOBUFF;
					}
					else
					{
						workStatus = WORKING;  //打符模式结束后进入正常控制
					}
					Key_Press.KEY_X = 1;
				}
		}
		else
		{
			Key_Press.KEY_X = 0;			
		}
#else  //底盘板打符模式
   	if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_X)    
		{        
			if(Key_Press.KEY_X == 0)
			{
				 if(Vision_flag==0)
				 {
						chassis_workStatus = STOP; 
						Vision_flag = 2;
				 }
				 else if(Vision_flag==2)
				 {
					 	Vision_flag = 0;
	     			chassis_workStatus = FOLLOWING;
				 }					 
			}
			Key_Press.KEY_X = 1;
		}
		else
		{
			Key_Press.KEY_X = 0;			
		}		
#endif 
		
		
	//**鼠标右键   自动打击	
		if(RC_Ctl.mouse.press_r==1)                 //鼠标右键  自动打击（按住）
		{
			Vision_flag = 1;            
			workStatus=AUTOATTACK;
		}
		else if(Vision_flag==1)
		{
			Vision_flag = 0;
			workStatus=WORKING;
		}	

		
		
	//**鼠标左键   开火	
		if(RC_Ctl.mouse.press_l==1)                  //鼠标左键  开火（按住）
		{
			Shoot_flag = 1;
			fire_workstatus=FIRE;
		}
		else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_Z) //Z   退弹模式(短按即可，长按会烧电机)
		{
			Shoot_flag = 1; 
			fire_workstatus = BACK;			
		}
		else if(Shoot_flag==1) 
		{
			Shoot_flag = 0;
			fire_workstatus=STOP_FIRE;
		}	

	//**按shift    使用超级冲刺
  			if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT)   //Shift    电容放电 	 
				{  		
  			   	super_cap_data.Cap_Out_Flag=1;	  //放电标志1
            super_cap_data.Cap_Charge_Flag=0;	//充电标志0		
				}
				else
				{
						super_cap_data.Cap_Out_Flag=0;    //放电标志0
					  super_cap_data.Cap_Charge_Flag=1; //充电标志1
				}

 /**********前后左右移动**********/
    //前或后 
		if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_W)           //W  前进
		 {
				if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT)   //Shift    加速移动   	 
				{
					Remote_data.RC_ch1 = HIGH_FORWARD_BACK_SPEED; 			 				
				}
				else
				{
					if(Speed_Straight_ACC <= NORMAL_FORWARD_BACK_SPEED)
						  Speed_Straight_ACC++;
					Remote_data.RC_ch1 = Speed_Straight_ACC; 			 
				}
		 }
		 else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_S)      //S  后退
		 {			 
				if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT)   //Shift    加速移动   	 
				{
					Remote_data.RC_ch1 = -HIGH_FORWARD_BACK_SPEED; 			 				
				}
				else
				{
					if(Speed_Straight_ACC <= NORMAL_FORWARD_BACK_SPEED)
						  Speed_Straight_ACC++;
					Remote_data.RC_ch1 = -Speed_Straight_ACC;			 
				}
		 }
		 else
		 {
			 Speed_Straight_ACC=100;
			 Remote_data.RC_ch1 = 0; 				 			 
		 }
		 //左或右
		 if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_A)            //A  左平移
		 {
				if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT)   //Shift    加速移动   	 
				{
					Remote_data.RC_ch0 = -HIGH_LEFT_RIGHT_SPEED; 			 				
				}
				else
				{
					if(Speed_Across_ACC <= NORMAL_LEFT_RIGHT_SPEED)
						  Speed_Across_ACC++;
					Remote_data.RC_ch0 = -Speed_Across_ACC;					 
				}			 
		 }
		 else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_D)            //D  右平移
		 {
				if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT)   //Shift    加速移动   	 
				{
					Remote_data.RC_ch0 = HIGH_LEFT_RIGHT_SPEED; 			 				
				}
				else
				{
					if(Speed_Across_ACC <= NORMAL_LEFT_RIGHT_SPEED)
						  Speed_Across_ACC++;
					Remote_data.RC_ch0 = Speed_Across_ACC;			 
				}				 
		 }
		 else
		 {
			 Speed_Across_ACC = 100;
			 Remote_data.RC_ch0 = 0; 				 			 			 			 
		 }

		 
/**********鼠标控制云台**********/
		 if(workStatus == WORKING || workStatus == AUTOATTACK)    //手动控制
		{
			Remote_data.RC_ch2 += (RC_Ctl.mouse.x * MOUSE_YAW_SPEED);     //Y轴位置环量累加  
			Remote_data.RC_ch3 -= (RC_Ctl.mouse.y * MOUSE_PITCH_SPEED);   //P轴位置环量累加 
		}
//		else if(workStatus == AUTOATTACK)  //自动瞄准
//		{
//			Remote_data.RC_ch2 = 0;                         //自动瞄准时取消手动控制Y轴
//			Remote_data.RC_ch3 = 0;                         //自动瞄准时取消手动控制P轴		
//		}
		else  //待机模式，初始化模式，补给模式  均不可手动控制
		{
//			Remote_data.RC_ch2 = 0;   
//			Remote_data.RC_ch3 = 0;   
		}
	 
		
}



/*
*摇杆量清零
*/
void Remote_reload(void)
{
		Remote_data.RC_ch0 = 0;
		Remote_data.RC_ch1 = 0;
		Remote_data.RC_ch2 = 0;   
		Remote_data.RC_ch3 = 0;   
		Remote_data.RC_sw  = 0;   
}





