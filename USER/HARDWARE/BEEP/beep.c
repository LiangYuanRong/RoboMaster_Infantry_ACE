#include "stm32f4xx.h"
#include "beep.h"

//初始化PB4为输出口		    
//BEEP IO初始化
void BEEP_Init(void)
{   
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM12, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM12, DISABLE);

    GPIO_PinAFConfig(GPIOH, GPIO_PinSource6, GPIO_AF_TIM12);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOH, &GPIO_InitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM12, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM12, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = 30000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 90 - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM12, &TIM_TimeBaseInitStructure);
    TIM_Cmd(TIM12, ENABLE);
	
}

const uint16_t tone_tab[] = 
{
  3822,  3405, 3033, 2863, 2551, 2272, 2024,	//bass 1~7
  1911,  1702, 1526, 1431, 1275, 1136, 1012,	//mid 1~7
  955,   851,  758,  715,   637, 568,   506,	//treble 1~7
};

/*DJI Macic启动音乐*/
const Sound_tone_e Mavic_Startup_music[Startup_Success_music_len] = 
{
  So5L, So5L, So5L, So5L, La6L, La6L, La6L, La6L, Mi3M, Mi3M, Mi3M, Mi3M, Mi3M, Silent,
};

/*影流之主*/
const Sound_tone_e Ying_liu_zhi_zhu[Ying_liu_zhi_zhu_music_len] = 
{
  Do1M, Re2M, Mi3M, Mi3M, So5M, La6M, La6M, Do1M, Do1M, Mi3M, Re2M, Do1M, Re2M, Mi3M, Silent,
	Do1M, Re2M, Mi3M, Mi3M, So5M, La6M, La6M, Do1M, Do1M, Mi3M, Re2M, Do1M, Do1M, Re2M, Do1M, Silent,
}; 

//
void Sing(Sound_tone_e tone)
{
  if(Silent == tone)
    BEEP_CH = 0;
  else 
  {
    BEEP_ARR = tone_tab[tone];
    BEEP_CH = tone_tab[tone] / 2;
  }
}


//play the start up music
void Sing_Startup_music(uint32_t index)
{
  if(index < Startup_Success_music_len)
    Sing(Mavic_Startup_music[index]);
}


//播放影流之主
void Sing_Ying_liu_zhi_zhu(uint32_t index)
{
  if(index < Ying_liu_zhi_zhu_music_len)
    Sing(Ying_liu_zhi_zhu[index]);
}

