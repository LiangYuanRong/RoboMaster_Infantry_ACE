#include "Capacitor_control.h"
#include "maths.h"
#include "stm32f4xx.h"

//=============�ṹ�嶨��
Super_Cap super_cap_data={0,0,0,0}; 
acceleration_control_type_t    Cap_Charge_acc_limit_Data  = {0,0,0,0,0};  //���ݳ����ٶ�����                        
//=============��������
int32_t Sum_motor_out=0;
//=============�ڲ���������
static void Cap_Out_Ctrl(u8 cap_out);


/*****************�������ݳ�ŵ��߼�****************/
//���ƴ󽻣��㶨��ѹ�壨���Ǻ㶨���������������ʱ
void Capacitor_control_chassis(PowerLimit_t *Power_data , float Charge_current , int16_t motor_out_1, int16_t motor_out_2, int16_t motor_out_3, int16_t motor_out_4)
{   
	Sum_motor_out=abs_int16(motor_out_1)+abs_int16(motor_out_2)+abs_int16(motor_out_3)+abs_int16(motor_out_4);
	
  if((super_cap_data.Cap_Out_Flag==1) && super_cap_data.Cap_Charge_Flag==0 )//�ŵ���������ʱ����磬�Ұ��°���shift��ң��������������
	{
		Cap_Out_Ctrl(1);//�����ŵ�
		super_cap_data.Cap_Charge_PWM=0;
	} 
	else if((super_cap_data.Cap_Charge_Flag==1) && super_cap_data.Cap_Out_Flag==0)//�����������ʱ���ŵ磬�Ҳ����°���shift�� ң�������������������м�
	{
		Cap_Out_Ctrl(0);//�رշŵ�

		if(Power_data->RemainPower[2] < 45.0f)//���幦��С��45j
		{
			super_cap_data.Cap_Charge_PWM=0;//ֹͣ���
		}
		else
		{
			if(Sum_motor_out==0)//��ʱ������������ֵΪ0
			{ 
				super_cap_data.Cap_Charge_PWM=50;//����ٶ�ֱ���������Ȳ�����������50%�ȣ�����ٶȲ�Ҫ��100%��Ӳ����ը����97%�Ͳ���ˣ�
			}
			else //�����ʱ������������Ƚ����ʸ������
			{
				super_cap_data.Cap_Charge_PWM = 0;//(80-Power_data->Real_Power[2])*0.5;///((Sum_motor_out/4)*0.006+1);    //��Ҫ��0.01������ʵ����֤һ��ʽ��Ч��
			}
		}
	}
	else
	{
	  super_cap_data.Cap_Charge_PWM=0; 
	}		 
	 
	super_cap_data.Cap_Charge_PWM_filt = Motion_acceleration_control(&Cap_Charge_acc_limit_Data , super_cap_data.Cap_Charge_PWM , 1);
//	TIM3->CCR2=abs_int16(limit_int16(super_cap_data.Cap_Charge_PWM_filt,80,0));   //���ݳ�����0-100,������80%��		

}




/*******���ݷŵ��߼�*************/
static void Cap_Out_Ctrl(u8 cap_out)
{
	if(cap_out==1)
	{
	   GPIO_SetBits(GPIOA,GPIO_Pin_2);	//����IO��ƽ�������ŵ翪��
	}
	else
	{
	   GPIO_ResetBits(GPIOA,GPIO_Pin_2);	//����IO��ƽ���رշŵ翪��
	}	
}

