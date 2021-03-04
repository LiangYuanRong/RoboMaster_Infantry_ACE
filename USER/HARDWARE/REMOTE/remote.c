#include "remote.h"
#include "usart.h"
#include "Safecheck_Task.h"

/*����*/
RC_Ctl_t RC_Ctl;							              	//ң�ؽṹ�嶨��
extern u8 RC_Receive_Flag;                    //ң�����Ӽ�¼��־λ���ڰ�ȫ���������һֱ����0��
extern u8 RC_Connected;                       //ң��ȷ�����ӱ�־λ


void DMA2_Stream5_IRQHandler(void)		//ң�����ݵĽ���
{
	//�ж��Ƿ�ΪDMA��������ж�
   if(DMA_GetFlagStatus(DMA2_Stream5,DMA_IT_TCIF5)==SET) 
	{
		DMA_ClearFlag(DMA2_Stream5,DMA_IT_TCIF5); 
		DMA_ClearITPendingBit(DMA2_Stream5,DMA_IT_TCIF5);
		Get_RC_Data();				                     //�����յ������ݱ��浽sbus_rx_buffer��	
	}
}



void Get_RC_Data(void)					//��ȡң������
{
	Safecheck_dog.RC_Receive_Flag = 1;          //ң�ؽ��ձ�־λ
	
	RC_Ctl.rc.ch0  = (Usart1_Rx[0]| (Usart1_Rx[1] << 8)) & 0x07ff; 						//!< Channel 0
	RC_Ctl.rc.ch1  = ((Usart1_Rx[1] >> 3) | (Usart1_Rx[2] << 5)) & 0x07ff; 		//!< Channel 1
	RC_Ctl.rc.ch2  = ((Usart1_Rx[2] >> 6) | (Usart1_Rx[3] << 2) | (Usart1_Rx[4] << 10)) & 0x07ff;//!< Channel 2
	RC_Ctl.rc.ch3  = ((Usart1_Rx[4] >> 1) | (Usart1_Rx[5] << 7)) & 0x07ff; //!< Channel 3
	RC_Ctl.rc.s1   = ((Usart1_Rx[5] >> 4)& 0x000C) >> 2;															//!< Switch left
	RC_Ctl.rc.s2   = ((Usart1_Rx[5] >> 4)& 0x0003); 																	//!< Switch right	
	RC_Ctl.mouse.x = Usart1_Rx[6] | (Usart1_Rx[7] << 8); 										    	//!< Mouse X axis
	RC_Ctl.mouse.y = Usart1_Rx[8] | (Usart1_Rx[9] << 8);										      //!< Mouse Y axis
	RC_Ctl.mouse.z = Usart1_Rx[10] | (Usart1_Rx[11] << 8); 									     	//!< Mouse Z axis
	RC_Ctl.mouse.press_l = Usart1_Rx[12]; 																				    	//!< Mouse Left Is Press ?
	RC_Ctl.mouse.press_r = Usart1_Rx[13]; 																				     	//!< Mouse Right Is Press ?
	RC_Ctl.key.v = Usart1_Rx[14] | (Usart1_Rx[15] << 8); 	                        //!< KeyBoard value
  RC_Ctl.rc.sw = ((int16_t)Usart1_Rx[16]|((int16_t)Usart1_Rx[17]<<8))&0x07FF;   //ң�����Ͻǲ���
   
}



void RC_unable(void)
{
        USART_Cmd(USART1, DISABLE);
}
void RC_restart(uint16_t dma_buf_num)
{
        USART_Cmd(USART1, DISABLE);
        DMA_Cmd(DMA2_Stream2, DISABLE);
        DMA_SetCurrDataCounter(DMA2_Stream2, dma_buf_num);

        USART_ClearFlag(USART1, USART_FLAG_IDLE);

        DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
        DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
        DMA_Cmd(DMA2_Stream2, ENABLE);
        USART_Cmd(USART1, ENABLE);
}

