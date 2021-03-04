#include "stm32f4xx.h"
#include "pwm.h"
/************************************************************TIM3 PWM初始化**************************************************************/
/*
功能：弹仓舵机/电容充电 PWM初始化  TIM3  PA6（ch1） PA7（ch2）   】】 & 副云台p轴 PA7
形参1 arr：自动重装载值
形参2 psc：定时器分频
*/
void TIM3_PWM_Init(u32 arr,u32 psc)//周期=（arr+1）*（psc+1）/CLK 84m     频率=1/周期      占空比 CCRx/Arr*100%
{
	//结构体声明
	GPIO_InitTypeDef GPIO_InitStructure;                       //结构体_引脚-声明
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;            //结构体_时钟声明
	TIM_OCInitTypeDef  TIM_OCInitStructure;                    //PWM输出

	//时钟开启
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	 //TIM3时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //使能时钟	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6, GPIO_AF_TIM3); //GPIOB0复用为定时器3 （输出）
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7, GPIO_AF_TIM3); //GPIOB0复用为定时器3 （输出）
	
	//引脚属性设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;               //GPIOA6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            //复用
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	    //速度100MHz(高速)
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd =	GPIO_PuPd_DOWN;         //下拉 
	GPIO_Init(GPIOA,&GPIO_InitStructure);                   //初始化PA6
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;               //GPIOA7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            //复用
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	    //速度100MHz(高速)
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd =	GPIO_PuPd_DOWN;         //下拉 
	GPIO_Init(GPIOA,&GPIO_InitStructure);                   //初始化PA7
	
  //初始化定时器  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;                //定时器分频  psc
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;                     //自动重装载值  arr
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 	
  TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);


//初始化TIM3 Channel12 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;          //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高	
	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);                  //根据T指定的参数初始化外设TIM3 OC3
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);         //使能TIM3在CCR3上的预装载寄存器
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);                  //根据T指定的参数初始化外设TIM3 OC3
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);         //使能TIM3在CCR3上的预装载寄存器
	
  TIM_ARRPreloadConfig(TIM3,ENABLE);                        //ARPE使能 
	
//开启定时器4
	TIM_Cmd(TIM3,ENABLE); 	//使能定时器4

}


/************************************************************TIM4 PWM初始化**************************************************************/

/*
功能：摩擦轮初始化（PD12 PD13）
形参1 arr：自动重装载值
形参2 psc：定时器分频
*/
void TIM4_PWM_Init(u32 arr,u32 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	  //TIM4时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 	//使能GPIOD时钟	


	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12, GPIO_AF_TIM4); //GPIOD12复用为定时器4
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13, GPIO_AF_TIM4 ); //GPIOD13复用为定时器4

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;           //GPIOD12 D13
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure);              //初始化GPIOH
	  	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频  84000/8400=10KHZ 即100us
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值            t=0.1ms*211=21.1ms
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//初始化定时器12
	
	//初始化TIM4  PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC1
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC2
	
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR1上的预装载寄存器
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR2上的预装载寄存器
	
  TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM4, ENABLE);  //使能TIM4		

}



/************************************************************TIM5 PWM初始化**************************************************************/
/*
功能： 副云台电机pwm初始化
形参1 arr：自动重装载值
形参2 psc：定时器分频
*/
void TIM5_PWM_Init(u32 arr,u32 psc)             
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	//TIM5时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); 	//使能GPIOH时钟	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE); 	//使能GPIOH时钟	

	GPIO_PinAFConfig(GPIOH,GPIO_PinSource10, GPIO_AF_TIM5); //GPIOH10复用为定时器5
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource11, GPIO_AF_TIM5 ); //GPIOH11复用为定时器5
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource12, GPIO_AF_TIM5); //GPIOH12复用为定时器5
	GPIO_PinAFConfig(GPIOI,GPIO_PinSource0, GPIO_AF_TIM5 ); //GPIOH9复用为定时器5
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_0 ;           //GPIOH6.H9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOH,&GPIO_InitStructure);              //初始化GPIOH
	GPIO_Init(GPIOI,&GPIO_InitStructure);              //初始化GPIOI
	  	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频  84000/8400=10KHZ 即100us
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值            t=0.1ms*211=21.1ms
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);//初始化定时器12
	
	//初始化TIM5  PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM5 OC1
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM5 OC2
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM5 OC3
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM5 OC4
	
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM5在CCR1上的预装载寄存器
  TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM5在CCR2上的预装载寄存器
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM5在CCR3上的预装载寄存器
  TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM5在CCR4上的预装载寄存器
	
  TIM_ARRPreloadConfig(TIM5,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM5, ENABLE);  //使能TIM12										  
}  


/************************************************************TIM12 PWM初始化**************************************************************/
/*
功能：
形参1 arr：自动重装载值
形参2 psc：定时器分频
*/
void TIM12_PWM_Init(u32 arr,u32 psc)             
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);  	//TIM12时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); 	//使能GPIOH时钟	

	GPIO_PinAFConfig(GPIOH,GPIO_PinSource6, GPIO_AF_TIM12); //GPIOH6复用为定时器12
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource9, GPIO_AF_TIM12); //GPIOH9复用为定时器12

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9;           //GPIOH6.H9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOH,&GPIO_InitStructure);              //初始化GPIOH6, H9
	  	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频  84000/8400=10KHZ 即100us
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值            t=0.1ms*211=21.1ms
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM12,&TIM_TimeBaseStructure);//初始化定时器12
	
	//初始化TIM12 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM12, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM12 OC1
	TIM_OC2Init(TIM12, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM12 OC2

	TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);  //使能TIM12在CCR1上的预装载寄存器
  TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Enable);  //使能TIM12在CCR2上的预装载寄存器

  TIM_ARRPreloadConfig(TIM12,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM12, ENABLE);  //使能TIM12										  
}  







