#include "stm32f4xx.h"
#include "time.h"

/************************************************************ TIM2 **************************************************************/
void TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef tim;
    NVIC_InitTypeDef         nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    tim.TIM_Period = 0xFFFFFFFF;
    tim.TIM_Prescaler = 90 - 1;	 //1M 的时钟  
    tim.TIM_ClockDivision = TIM_CKD_DIV1;	
    tim.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_ARRPreloadConfig(TIM2, ENABLE);	
    TIM_TimeBaseInit(TIM2, &tim);

	  nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
	
    TIM_Cmd(TIM2,ENABLE);	
	  TIM_ITConfig(TIM2, TIM_IT_Update,ENABLE);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
}



void TIM2_IRQHandler(void)  
{
    if (TIM_GetITStatus(TIM2,TIM_IT_Update)!= RESET) 
	{

		
    }
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
}

uint32_t Get_Time_Micros(void)
{
	return TIM2->CNT;
}





/************************************************************ TIM6 **************************************************************/
/*
*功能：定时器6 输入捕获
*/
void TIM6_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    
    nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tim.TIM_Prescaler = 90-1;        //84M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 1000;  //1ms,1000Hz
    TIM_TimeBaseInit(TIM6,&tim);
	
	  TIM_Cmd(TIM6, ENABLE);	 
    TIM_ITConfig(TIM6, TIM_IT_Update,ENABLE);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);	
}

int32_t counting=0;
static int16_t ms_count=0;
int encoder[2] = {0 , 0};// 两次编码器读值，用以计算溢出方向
int volatile N = 0;// 圈数
int volatile EncCnt = 0;// 编码器取值


void TIM6_DAC_IRQHandler(void)  
{
	
    if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET) 
	  {
			TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
      TIM_ClearFlag(TIM6, TIM_FLAG_Update);

//				encoder[1] = TIM4->CNT;
//				if( (encoder[1] - encoder[0] ) > 0x7FFF)
//				{
//				  N--;
//				}
//				 else if( (encoder[0] - encoder[1] ) > 0x7FFF)
//				 {
//				  N++;
//				 }
//         EncCnt = N * 65536 + encoder[1];
//			   encoder[0] = encoder[1];
			
			ms_count++;
			if(ms_count==10)
			{		
//				 Pitch_Motor_Data.position = EncCnt;
//				 Pitch_Motor_Data.speed = (Pitch_Motor_Data.position - Pitch_Motor_Data.last_Position) ;
				 ms_count = 0;
//				 Pitch_Motor_Data.last_Position = Pitch_Motor_Data.position;
			
			}
	
		}
}


