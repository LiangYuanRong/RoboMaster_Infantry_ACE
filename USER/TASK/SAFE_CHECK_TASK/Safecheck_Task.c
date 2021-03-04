/*************安全监测*******************
内容：1.遥控器安全监测（现象：断连，数据异常
                      处理：强行变为待机状态）
      2.can2断连检测（现象：断连，数据异常
                     处理：云台板：恢复待机状态
							       底盘板：恢复待机状态）
备注：can看门狗还没加
*****************************************/

#include "main.h"
#include "Mode_Def.h"
#include "Safecheck_Task.h"

//================参数定义
int safecheck_heart = 0;          //安全检测任务心跳

//================结构体定义
DogTypeDef Safecheck_dog={0,0,0};         //相关结构体声明
DogTypeDef Dog={0,1,1};

//================内部函数定义
static void RC_Check(void);
static void Dog_Feed(void);


 

/*=======安全监测任务======*/
void Safecheck_Task(void *pvParameters)   
{

	while(1)
	{
		/*心跳*/
		safecheck_heart = !safecheck_heart; // 心跳
		LEDE4 = safecheck_heart;
		
#ifdef watch_dog		
		//遥控断连检测
    RC_Check();   
		//喂狗
		Dog_Feed();
#endif
		
		//检测周期
    vTaskDelay(CHECK_CONTROL_TIME_MS/portTICK_RATE_MS);
	}
}


/*=================检测遥控连接========================*/
static void RC_Check(void)
{

	/*断连记录*/
  if(Safecheck_dog.RC_Receive_Flag==0 || RC_Ctl.rc.s1<0||RC_Ctl.rc.s1>3 || RC_Ctl.rc.s2<0||RC_Ctl.rc.s2>3)   //无遥控数据返回或遥控数据出现错误
	{
		Safecheck_dog.RC_Error++;
		if(Safecheck_dog.RC_Error>=100)
			Safecheck_dog.RC_Error=100;
	}
	else
	{
	  Safecheck_dog.RC_Error = 0;                		
	}
	
	/*断连标志位更新*/
	Safecheck_dog.RC_Receive_Flag = 0;     //更新遥控接收标志位
	
	/*断连处理*/
	if(Safecheck_dog.RC_Error >= 20)       //断连400ms
	{
		RC_Ctl.rc.s2    = 2;                 //进入待机模式
		INITIALIZE_flag = 0;                 //初始化标志位清零
		Safecheck_dog.RC_Connected = 0;      //遥控确定断连
	}
	else
	{
		Safecheck_dog.RC_Connected = 1;      //遥控已连接
	}
	
}






/*=======================喂狗函数=======================*/
static void Dog_Feed(void)
{
  if(Safecheck_dog.RC_Connected==1)  
	{
	  IWDG_ReloadCounter();     //重装载		
	}
}
