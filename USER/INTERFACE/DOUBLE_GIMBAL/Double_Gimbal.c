#include "stm32f4xx.h"
#include "Double_Gimbal.h"

Second_Gimbal_Data_Typedef second_gimbal={0};


/*
*功能：副云台PY轴控制代码
*输入：副云台PY轴真实控制量
*输出：直接给PWM值到电机驱动板
*/
//PA0为副P轴转向控制口   PA1为副y轴转向控制口
//高电平逆转，低电平顺转
//P轴电机TIM5 CCR3  PH12
//Y轴电机TIM5 CCR4  PI0
void Second_Gimbal_Control(int16_t yaw_control_data, int16_t pitch_contral_data)
{
/*yaw轴控制*/
	if(yaw_control_data>0)//顺转
	{
	  GPIO_ResetBits(GPIOA,GPIO_Pin_1);
		TIM5->CCR4 = yaw_control_data;
	}
	else if(yaw_control_data<0)//逆转
	{
	  GPIO_SetBits(GPIOA,GPIO_Pin_1);  
		TIM5->CCR4 = (-yaw_control_data);
	}
	else
	{
	  GPIO_ResetBits(GPIOA,GPIO_Pin_1);		
		TIM5->CCR4 = 0;
	}
   
/*pitch轴控制*/
	if(pitch_contral_data>0)//顺转
	{
	  GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		TIM5->CCR3 = pitch_contral_data;
	}
	else if(pitch_contral_data<0)//逆转
	{
	  GPIO_SetBits(GPIOA,GPIO_Pin_0);  
		TIM5->CCR3 = (-pitch_contral_data);
	}
	else
	{
	  GPIO_ResetBits(GPIOA,GPIO_Pin_0);		
		TIM5->CCR3 = 0;
	}	 	
}




