#include "stm32f4xx.h"
#include "iwdg.h"


/*���Ź���ʼ��*/
void IWDG_Init(void)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //ʹ�ܶ�IWDG->PR IWDG->RLR��д
	
	IWDG_SetPrescaler(4); //����IWDG��Ƶϵ��

	IWDG_SetReload(300);   //����IWDGװ��ֵ

	IWDG_ReloadCounter(); //reload
	
	IWDG_Enable();       //ʹ�ܿ��Ź�
}

