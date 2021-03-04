#include "stm32f4xx.h"
#include "iwdg.h"


/*看门狗初始化*/
void IWDG_Init(void)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //使能对IWDG->PR IWDG->RLR的写
	
	IWDG_SetPrescaler(4); //设置IWDG分频系数

	IWDG_SetReload(300);   //设置IWDG装载值

	IWDG_ReloadCounter(); //reload
	
	IWDG_Enable();       //使能看门狗
}

