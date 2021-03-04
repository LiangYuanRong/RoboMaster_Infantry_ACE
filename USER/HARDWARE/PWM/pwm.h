#ifndef __PWM_H
#define __PWM_H

void TIM12_PWM_Init(u32 arr,u32 psc);
void TIM5_PWM_Init(u32 arr,u32 psc);   
void TIM3_PWM_Init(u32 arr,u32 psc); 
void TIM4_PWM_Init(u32 arr,u32 psc);

#define Gimbal_Pitch_Output  TIM3->CCR3  //PB0

#endif
