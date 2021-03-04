#include "usart.h"
#include "automatic_strike.h"

//=====================结构体声明
VISION_AUTO_DATA  Vision_Auto_Data={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0,0};



/*
*名称：串口三视觉中断函数
*功能：视觉数据接收,保存在接收数组中
*输入：无
*输出：无
*/
int16_t USART3_Receive_Flag=0;
u16 usart3_dataLen=0;
u8 MSDataBuffer[USART3_RX_LEN];

void USART3_IRQHandler(void)			// 串口三小电脑接收数据中断
{
	u16 i;
	u8 num=0;
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)//触发中断标志位
	{
		DMA_Cmd(DMA1_Stream1, DISABLE);                    //关闭DMA,防止处理期间有数据
		while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);  
		num=USART3->SR;
		num=USART3->DR;
		num=USART3_RX_LEN-DMA_GetCurrDataCounter(DMA1_Stream1);
		usart3_dataLen=num;
		for(i=0;i<num;i++)
		{
			MSDataBuffer[i]=Usart3_Rx[i];
		}
		DMA_SetCurrDataCounter(DMA1_Stream1,USART3_RX_LEN);
		DMA_Cmd(DMA1_Stream1, ENABLE);//开启DMA
		USART3_Receive_Flag=1;
	}
}




 
/*
*功能：发送数据到MiniPC（串口三）
*输入：8位数据自定义，8位数据自定义
*输出：无
*协议：帧头0xF5  校验段0xF5 0x00 0x00  帧尾：0xF6
*/
char SendBuff[5];

void MiniPC_Send_Data(u8 data, u8 mode, u8 shoot_speed)
{ 
	static unsigned char i = 0;

	SendBuff[0] = 0xFF;
	SendBuff[1] = data;
	SendBuff[2] = mode;
	SendBuff[3] = shoot_speed;	
	SendBuff[4] = 0xFE;
	for(i=0;i<5;i++)
	{	
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);  	 //发送数据到串口3
		USART_SendData(USART3,(uint8_t)SendBuff[i]);  //等待上次传输完成 
	}
}


/*
*名称：MiniPC串口接收数据处理
*功能：通过串口三接收并保存MiniPC发来的数据
*输入：无
*输出：无
*/

void MiniPC_Data_Deal(void) //视觉（裁判）系统数据
{
	u16 i;

	if(USART3_Receive_Flag)
	{
		for(i=0;i<usart3_dataLen;i++)
		{
			if(MSDataBuffer[i]==0xFF && MSDataBuffer[i+7]==0xFE)
			{
				Vision_Auto_Data.auto_yaw_angle = (hex2Float(MSDataBuffer[i+2],MSDataBuffer[i+1])/100);  //自动打击的y轴角度计算/100
				Vision_Auto_Data.auto_pitch_angle=(hex2Float(MSDataBuffer[i+4],MSDataBuffer[i+3])/100);  //自动打击的p轴角度计算
			  Vision_Auto_Data.len=(hex2Float(MSDataBuffer[i+6],MSDataBuffer[i+5])/100);//距离
				
				i=i+7;
			}
		}
		USART3_Receive_Flag=0;
	}
}


/*
*功能：视觉卡尔曼数据初始化
*
*/
void MiniPC_Kalman_Data_Init(void)
{
	{
		kalman_Filter_Init.A_data[0]=1;
		kalman_Filter_Init.A_data[2]=1;
		kalman_Filter_Init.A_data[5]=1;
		kalman_Filter_Init.A_data[7]=1;
		kalman_Filter_Init.A_data[10]=1;
		kalman_Filter_Init.A_data[15]=1;
		//观测矩阵
		kalman_Filter_Init.H_data[0]=1;
		kalman_Filter_Init.H_data[5]=1;
    //状态转移协方差矩阵
		kalman_Filter_Init.P_data[0]=1;
		kalman_Filter_Init.P_data[5]=1;
		kalman_Filter_Init.P_data[10]=1;
		kalman_Filter_Init.P_data[15]=1;
		//观测噪声方差
		kalman_Filter_Init.R_data[0]=1;
		kalman_Filter_Init.R_data[3]=1;
		//状态转移协方差矩阵
		kalman_Filter_Init.Q_data[0]=10;
		kalman_Filter_Init.Q_data[5]=10;
		kalman_Filter_Init.Q_data[10]=0.01;
		kalman_Filter_Init.Q_data[15]=0.01;
	}
	Kalman_Filter_Init(&kalman_Filter ,&kalman_Filter_Init);
}





/* 
*功能：高低八位数据整合
*/
float hex2Float(uint8_t HighByte, uint8_t LowByte)
{
  float high = (float) (HighByte & 0x7f);
  float low  = (float) LowByte;
  if (HighByte & 0x80)//MSB is 1 means a negative number
  {
    return (high*256.0f + low) - 32768;
  }
  else
  {
    return (high*256.0f + low);
  }
}


