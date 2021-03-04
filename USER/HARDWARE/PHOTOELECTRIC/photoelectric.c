#include "stm32f4xx.h"
#include "photoelectric.h"

/*主云台yaw轴中值光电*/
void Yaw_Zero_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*副云台yaw轴中值光电*/
void Yaw_Two_Zero_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}


/*
* 功能：检测主云台Y轴中值光电返回值
* 输入：无
* 输出：光电返回值1或0（到中值返回1，非中值返回0）
*/

u8 Yaw_Zero_Value(void)   //检测Y轴是否到位   到中值返回0，否则返回1
{
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3);
}

/*
* 功能：检测副云台Y轴中值光电返回值
* 输入：无
* 输出：光电返回值1或0（到中值返回1，非中值返回0）
*/

u8 Sec_Yaw_Zero_Value(void)   //检测Y轴是否到位   到中值返回0，否则返回1
{
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2);
}

