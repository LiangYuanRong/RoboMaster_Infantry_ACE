#include "BMI160.h"
#include "usart.h"
#include "Mode_Def.h"
#include "RefereeDeal.h"

/*
*����2
*������ģ�飬�Լ����ʰ����ݻ�ȡ
*/

u8 GyroDataBuffer_tx[USART2_TX_LEN];
u8 GyroDataBuffer_rx[USART2_RX_LEN];


/*
*���ƣ����ڶ���������ǽ����ж�
*���ܣ����������ݽ��ղ�����
*/
int16_t USART2_Receive_Flag=0;
u16 usart2_dataLen=0;
void USART2_IRQHandler(void)			// ���������ж�
{
	u16 i;
	u8 num=0;
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)//�����жϱ�־λ
	{
		DMA_Cmd(DMA1_Stream5, DISABLE);//�ر�DMA,��ֹ�����ڼ�������
		while(DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);
		num=USART2->SR;
		num=USART2->DR;
		num=USART2_RX_LEN-DMA_GetCurrDataCounter(DMA1_Stream5);
		usart2_dataLen=num;
		for(i=0;i<num;i++)
			GyroDataBuffer_rx[i]=Usart2_Rx[i];
		DMA_SetCurrDataCounter(DMA1_Stream5,USART2_RX_LEN);
		DMA_Cmd(DMA1_Stream5, ENABLE);//����DMA
		USART2_Receive_Flag=1;
		#ifdef board_chassis
		Power_Deal();
		#else
		BMI_Data_Deal();
		#endif

	}
}



/*
*BMI160���ݴ���ṹ��
*/
BMI160_Data_Typedef BMI160_Data;
BMI160_Data_Typedef Sec_BMI160_Data;

float receive[6]={0};
int32_t yaw=0;

void BMI_Data_Deal()
{
	u16 i;

	if(USART2_Receive_Flag)
	{
		for(i=0;i<usart2_dataLen;i++)
		{
			if(GyroDataBuffer_rx[i]==0xfe && GyroDataBuffer_rx[i+13]==0xee)
			{
				
				  BMI160_Data.Gyro_X=((short)(GyroDataBuffer_rx[1]<<8|GyroDataBuffer_rx[2]))/100.0f;
//  				BMI160_Data.Gyro_Y=((short)(GyroDataBuffer_rx[3]<<8|GyroDataBuffer_rx[4]))/100.0f;
				  BMI160_Data.Gyro_Z = -((short)(GyroDataBuffer_rx[5]<<8|GyroDataBuffer_rx[6]))/100.0f;           //������ģ�鳯�ϣ��Ӹ��ţ���֮
//				BMI160_Data.roll_angle=((short)(GyroDataBuffer_rx[7]<<8|GyroDataBuffer_rx[8]))/100.0f;
  				BMI160_Data.pitch_angle=((short)(GyroDataBuffer_rx[9]<<8|GyroDataBuffer_rx[10]))/100.0f;
			    BMI160_Data.yaw_angle = -((short)(GyroDataBuffer_rx[11]<<8|GyroDataBuffer_rx[12]))/100.0f;      //������ģ�鳯�ϣ��Ӹ��ţ���֮
				
				i=i+13;
			}
		}
		USART2_Receive_Flag=0;
	}
}


/*���ʰ��������*///����������ļ�����д���հɣ���ʱ���ø��ˣ�����ļ�ͳһ�մ���2��Ϣ��
POWER_Data_Typedef  POWER_Data;
extern REFEREE_t REFEREE;

void Power_Deal(void)
{
		u16 i;

	if(USART2_Receive_Flag)
	{
		for(i=0;i<usart2_dataLen;i++)
		{
			if(GyroDataBuffer_rx[i]==0xfe && GyroDataBuffer_rx[i+13]==0xee)
			{
				
				POWER_Data.Motor_1=((short)(GyroDataBuffer_rx[1]<<8|GyroDataBuffer_rx[2]))/10000.0f;
				POWER_Data.Motor_2=((short)(GyroDataBuffer_rx[3]<<8|GyroDataBuffer_rx[4]))/10000.0f;
				POWER_Data.Motor_3=((short)(GyroDataBuffer_rx[5]<<8|GyroDataBuffer_rx[6]))/10000.0f;
				POWER_Data.Motor_4=((short)(GyroDataBuffer_rx[7]<<8|GyroDataBuffer_rx[8]))/10000.0f;
				POWER_Data.Cap_5  =((short)(GyroDataBuffer_rx[9]<<8|GyroDataBuffer_rx[10]))/10000.0f;				
				POWER_Data.chassis_power_sum = (POWER_Data.Motor_1 + POWER_Data.Motor_2 + POWER_Data.Motor_3 + POWER_Data.Motor_4)*(int16_t)(REFEREE.PowerHeat.chassis_volt*0.001);//(int16_t)(REFEREE.PowerHeat.chassis_volt*0.001); 
				 
				i=i+13;
			}
		}
		USART2_Receive_Flag=0;
	}
       
}



/*
*���ܣ��������ݵ�������ģ����г�ʼ��У׼����̨���ڶ���
*Э�飺֡ͷ  У���0x12 0x34  ֡β
*/

void BMI160_Zero_Correct(void)
{
	u8 i = 0;
  u8 SendBuff_Correct[2];
	SendBuff_Correct[0] = 0x12;
	SendBuff_Correct[1] = 0x34;
	
	for(i=0;i<2;i++)
	{
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);  	 //�������ݵ�����2
		USART_SendData(USART2,(uint8_t)SendBuff_Correct[i]);  //�ȴ��ϴδ������ 
	}
}

/*
*���ܣ��������ݵ�������ģ�����Y������㣨��̨���ڶ���
*Э�飺֡ͷ  У���0xfe 0xef  ֡β
*/

void BMI160_AngleZ_Zero(void)
{
	u8 i = 0;
  u8 SendBuff_Z_Zero[2];
	SendBuff_Z_Zero[0] = 0xfe;
	SendBuff_Z_Zero[1] = 0xef;
	for(i=0;i<2;i++)
	{
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);  	 //�������ݵ�����3
		USART_SendData(USART2,(uint8_t)SendBuff_Z_Zero[i]);  //�ȴ��ϴδ������ 
	}
}


/*
*���ܣ���������ι��������̨���ڶ���
*Э�飺0x01
*/

void Gyro_usart_iwdg(void)
{
	u8 i = 0;
  u8 gyro[1];
	gyro[0] = 0x01;
	for(i=0;i<1;i++)
	{
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);  	 //�������ݵ�����3
		USART_SendData(USART2,(uint8_t)gyro[i]);  //�ȴ��ϴδ������ 
	}
}

