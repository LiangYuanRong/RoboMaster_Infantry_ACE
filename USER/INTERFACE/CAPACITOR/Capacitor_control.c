#include "Capacitor_control.h"
#include "maths.h"
#include "stm32f4xx.h"

//=============结构体定义
Super_Cap super_cap_data={0,0,0,0}; 
acceleration_control_type_t    Cap_Charge_acc_limit_Data  = {0,0,0,0,0};  //电容充电加速度限制                        
//=============参数定义
int32_t Sum_motor_out=0;
//=============内部函数定义
static void Cap_Out_Ctrl(u8 cap_out);


/*****************超级电容充放电逻辑****************/
//类似大交，恒定电压冲（大交是恒定电流），电机慢速时
void Capacitor_control_chassis(PowerLimit_t *Power_data , float Charge_current , int16_t motor_out_1, int16_t motor_out_2, int16_t motor_out_3, int16_t motor_out_4)
{   
	Sum_motor_out=abs_int16(motor_out_1)+abs_int16(motor_out_2)+abs_int16(motor_out_3)+abs_int16(motor_out_4);
	
  if((super_cap_data.Cap_Out_Flag==1) && super_cap_data.Cap_Charge_Flag==0 )//放电条件：此时不充电，且按下按键shift或遥控器拨轮向下旋
	{
		Cap_Out_Ctrl(1);//开启放电
		super_cap_data.Cap_Charge_PWM=0;
	} 
	else if((super_cap_data.Cap_Charge_Flag==1) && super_cap_data.Cap_Out_Flag==0)//充电条件：此时不放电，且不按下按键shift或 遥控器拨轮向上旋或在中间
	{
		Cap_Out_Ctrl(0);//关闭放电

		if(Power_data->RemainPower[2] < 45.0f)//缓冲功率小于45j
		{
			super_cap_data.Cap_Charge_PWM=0;//停止充电
		}
		else
		{
			if(Sum_motor_out==0)//此时电机的输出期望值为0
			{ 
				super_cap_data.Cap_Charge_PWM=50;//充电速度直接拉满（先不拉满，试试50%先，最大速度不要到100%，硬件会炸，到97%就差不多了）
			}
			else //电机此时有输出量，优先将功率给电机用
			{
				super_cap_data.Cap_Charge_PWM = 0;//(80-Power_data->Real_Power[2])*0.5;///((Sum_motor_out/4)*0.006+1);    //需要调0.01参数，实际验证一下式子效果
			}
		}
	}
	else
	{
	  super_cap_data.Cap_Charge_PWM=0; 
	}		 
	 
	super_cap_data.Cap_Charge_PWM_filt = Motion_acceleration_control(&Cap_Charge_acc_limit_Data , super_cap_data.Cap_Charge_PWM , 1);
//	TIM3->CCR2=abs_int16(limit_int16(super_cap_data.Cap_Charge_PWM_filt,80,0));   //电容充电输出0-100,限制在80%内		

}




/*******电容放电逻辑*************/
static void Cap_Out_Ctrl(u8 cap_out)
{
	if(cap_out==1)
	{
	   GPIO_SetBits(GPIOA,GPIO_Pin_2);	//拉高IO电平，开启放电开关
	}
	else
	{
	   GPIO_ResetBits(GPIOA,GPIO_Pin_2);	//拉低IO电平，关闭放电开关
	}	
}

