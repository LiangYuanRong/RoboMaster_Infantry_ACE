#include "stm32f4xx.h"
#include "can1.h"

void CAN1_Configuration(uint8_t SJW, uint8_t BS1, uint8_t BS2, uint8_t Prescaler, uint8_t mode)
{
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    GPIO_InitTypeDef       GPIO_InitStructure;
	  NVIC_InitTypeDef       NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);		//打开GPIO时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);	//配置引脚复用功能
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;													//复用推挽输出
    GPIO_InitStructure.GPIO_OType=  GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOD, &GPIO_InitStructure);
    
		NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;					//中断配置 
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;           //抢占优先级 1
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                  //响应优先级 0
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure); 
    
    CAN_DeInit(CAN1);                        //将外围寄存器初始化到它们的默认重置值
    CAN_StructInit(&CAN_InitStructure);      //用它的默认值填充每个CAN_InitStructure成员
    /************CAN总线的配置*******************/
    CAN_InitStructure.CAN_TTCM = DISABLE;			//非时间触发通信模式
    CAN_InitStructure.CAN_ABOM = DISABLE;			//软件自动离线管理模式
    CAN_InitStructure.CAN_AWUM = DISABLE;			//自动唤醒模式，睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
    CAN_InitStructure.CAN_NART = DISABLE;			//非自动重传输模式，禁止报文自动传送 
    CAN_InitStructure.CAN_RFLM = DISABLE;			//接收FIFO锁定模式，报文不锁定,新的覆盖旧的 
    CAN_InitStructure.CAN_TXFP = ENABLE;			//发送FIFO优先迹，优先级由报文标识符决定 
    CAN_InitStructure.CAN_Mode = mode;		//模式设置： mode:0,普通模式;1,回环模式;
    CAN_InitStructure.CAN_SJW  = SJW;
    CAN_InitStructure.CAN_BS1=BS1;    //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
	  CAN_InitStructure.CAN_BS2=BS2;    //Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler = Prescaler;      //brp    CAN BaudRate 45/(1+8+6)/3=1Mbps
    CAN_Init(CAN1, &CAN_InitStructure);
		/******CAN总线的过滤配置(接收配置)********/
    CAN_FilterInitStructure.CAN_FilterNumber=0;                  //过滤器 0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;//0x0000; //0x4000
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//0x0000; //0xfe00
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;//0x0000;  //0x0006
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//the message which pass the filter save in fifo0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);			//滤波器初始化   

	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);		//接收中断    启用或禁用指定的CANx中断    CAN_IT_FMP0:等待中断的FIFO 0消息
}
