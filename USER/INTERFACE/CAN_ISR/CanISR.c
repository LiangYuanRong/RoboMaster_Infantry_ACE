#include "CanISR.h"
#include "PID_Def.h"
#include "Mode_Def.h"

#include "rmmotor.h"
#include "remote.h"
#include "RefereeDeal.h"

#include "Safecheck_Task.h"

//================��������
int32_t Encoder_yaw_code=0;                   //Y���Ȧ����ֵ������ԭʼֵ
extern int16_t gimbal_output[2];              //��̨����������    1��Y����
extern int8_t YAW_ZERO_FLAG;                  //yaw����ֵ����־
extern int8_t GIMBAL_SUPPLY_FLAG;             //��̨������־λ
extern int16_t Sec_chassis_angle;               //����̨��Ե��̵Ĳ��

//================�ṹ�嶨��
first_order_low_filter_type_t second_yaw_speed_lowfilt= {0.0f,0.0f,0.0f,0.1};//test
extern REFEREE_t REFEREE;             //����ϵͳ����




/*
* ���ܣ�CAN1���շ��ص���Ϣ���������������
* ���ݽ��գ� ��̨�壺P�����ٶ�ֵ������P�ᵥȦ������λ�÷���������̨yaw��2006������ݷ���
*           ���̰壺����4������ݷ�����yaw��3508���ݷ������������2006���ݷ�����
 */ 
void CAN1_RX0_IRQHandler(void)                     
{
  CanRxMsg rx_message;    	
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);		
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0); 
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);       
		switch(rx_message.StdId)                                  //�ĸ����̵��
		{
	
#ifdef board_chassis	 //�����ĸ���� �����̰壩	
			/*���̵��*/
			case 0x201:  
			{
				Chassis_Motor[0].position= (rx_message.Data[0]<<8 | rx_message.Data[1]);
				Chassis_Motor[0].speed = (rx_message.Data[2] << 8) | rx_message.Data[3];
				break;	
			}	
			case 0x202:
			{
				Chassis_Motor[1].position= (rx_message.Data[0]<<8 | rx_message.Data[1]);
				Chassis_Motor[1].speed = (rx_message.Data[2] << 8) | rx_message.Data[3];
				break;	
			}	
			case 0x203:
			{
				Chassis_Motor[2].position= (rx_message.Data[0]<<8 | rx_message.Data[1]);
				Chassis_Motor[2].speed = (rx_message.Data[2] << 8) | rx_message.Data[3];
				break;	
			}	
			case 0x204:
			{
				Chassis_Motor[3].position= (rx_message.Data[0]<<8 | rx_message.Data[1]);
				Chassis_Motor[3].speed = (rx_message.Data[2] << 8) | rx_message.Data[3];
				break;	
			}
			
			/*yaw��3508����ٶ�*/
			case 0x205:
			{
				Yaw_Motor_Data.position = (rx_message.Data[0] << 8) | rx_message.Data[1];
				Motor_Actual_Position(&Yaw_Motor_Data,3*19,8192);//����yaw�������ʵ����ֵ
				Yaw_Motor_Data.speed = (rx_message.Data[2] << 8) | rx_message.Data[3];
				break;	  
			}	
			
			/*�������*/                         
			case 0x207:  
			{
				Fire_GD_Motor.position = (rx_message.Data[0] << 8) | rx_message.Data[1];
				Fire_GD_Motor.speed = (rx_message.Data[2] << 8) | rx_message.Data[3];
				break;	
			}
			
#endif 
			

#ifdef board_gimbal   //��̨can����  ����̨�壩
			
			/*Pitch�������λ�÷���*/
			case 0x01: 
			{ 
//					if(rx_message.Data[1]==0x08) 
//					{
						Pitch_Motor_Data.position = (rx_message.Data[6]<<24 | rx_message.Data[5]<<16 | rx_message.Data[4]<<8 | rx_message.Data[3]); 
						Pitch_Motor_Data.actual_Position = Pitch_Motor_Data.position - Pitch_Middle_Angle;
//					}	
				break;
			}
			
		 /*P��3510����ٶȷ���*/                          
      case 0x201: 
			{
				Pitch_Motor_Data.speed = ((rx_message.Data[2] << 8) | rx_message.Data[3]);
				break;	
			}
			
  #ifdef double_gimbal
			/*����̨yaw��2006������ݷ���*/
       case 0x202: 
			{ 
				Second_Yaw_Motor_Data.position = (rx_message.Data[0] << 8) | rx_message.Data[1];
				Motor_Actual_Position(&Second_Yaw_Motor_Data,1*3,8192);
				Second_Yaw_Motor_Data.speed = (rx_message.Data[2] << 8) | rx_message.Data[3];
				break; 	
			}	
  #endif
			
#endif

		}
	}
}




/*
* ���ܣ�CAN2���շ��ص���Ϣ�����ͨ�ţ�
* ���ݽ��գ� ��̨�壺ң�����ݣ���ң��ģʽ��RCģʽ�����߲����棩��yaw������ʵλ�ã�yaw�����ٶ�
*           ���̰壺��̨��ʼ���ɹ���־,yaw����ֵ�����־��yaw����������ݣ�����״̬��־
*/ 
void CAN2_RX0_IRQHandler(void)
{
	CanRxMsg rx_Message;
	if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);		
		CAN_Receive(CAN2, CAN_FIFO0, &rx_Message);
		switch(rx_Message.StdId)
		{		
#ifdef board_gimbal  
			case 0x400:
			{ 
				Safecheck_dog.RC_Receive_Flag = 1; 
				RC_Ctl.rc.ch2 = ((rx_Message.Data[0] << 8) | rx_Message.Data[1]);
				RC_Ctl.rc.ch3 = ((rx_Message.Data[2] << 8) | rx_Message.Data[3]);	
				RC_Ctl.rc.sw = (rx_Message.Data[4] << 8  | rx_Message.Data[5]);   //��̨���Բ��ã����滻�ɲ���ϵͳ��Ϣ��		
				RC_Ctl.rc.s1 = (rx_Message.Data[6])/10;
				RC_Ctl.rc.s2 = (rx_Message.Data[6])%10;
				RC_Ctl.mouse.press_l = (rx_Message.Data[7])/10-1;
				RC_Ctl.mouse.press_r = (rx_Message.Data[7])%10;
        
  			break;
			}
			case 0x401: 
			{
				Safecheck_dog.RC_Receive_Flag = 1;
				RC_Ctl.mouse.x = ((rx_Message.Data[0] << 8) | rx_Message.Data[1]);
				RC_Ctl.mouse.y = ((rx_Message.Data[2] << 8) | rx_Message.Data[3]);	
				RC_Ctl.key.v = (rx_Message.Data[4] << 8  | rx_Message.Data[5]); 				
				RC_Ctl.rc.s1 = (rx_Message.Data[6])/10;
				RC_Ctl.rc.s2 = (rx_Message.Data[6])%10;
				RC_Ctl.mouse.press_l = (rx_Message.Data[7])/10-1;
				RC_Ctl.mouse.press_r = (rx_Message.Data[7])%10;

				break;
			}
			case 0x402:
			{				
			  Safecheck_dog.RC_Receive_Flag = 1; 

				Yaw_Motor_Data.actual_Position = (rx_Message.Data[0]<<24 | rx_Message.Data[1]<<16 | rx_Message.Data[2]<<8 | rx_Message.Data[3]); 
        Yaw_Motor_Data.speed = (rx_Message.Data[4]<<8 | rx_Message.Data[5]);
				REFEREE.RobotStatus.shooter_heat0_speed_limit = rx_Message.Data[6];
				
				break;
			}	
#endif

#ifdef board_chassis  
			case 0x300:
			 {
//				  Yaw_Motor_Data.actual_Position = (rx_Message.Data[3]<<24 | rx_Message.Data[2]<<16 | rx_Message.Data[1]<<8 | rx_Message.Data[0]); 
				  INITIALIZE_flag  = rx_Message.Data[0]/100;
				  YAW_ZERO_FLAG    = (rx_Message.Data[0]%100)/10;
				  GIMBAL_SUPPLY_FLAG = (rx_Message.Data[0])%10;
			    gimbal_output[1] = (rx_Message.Data[1]<<8 | rx_Message.Data[2]); 
				  Sec_chassis_angle = (rx_Message.Data[3] << 8  | rx_Message.Data[4]);
			    break;
			 }
				 
#endif

		}			
	}
}



/***************************************************CAN1����*****************************************************************/


/*
 * ���ܣ���̨��can1���͵�P����
 */
void CAN1_SendCommand_Gimbal_Pitch(int16_t ESC_201,int16_t ESC_202)//P������201,����̨yaw2006��202
{ 
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;                                  //����һ��������Ϣ�Ľṹ��
	
	TxMessage.StdId = 0x200;                             //����820r���ñ�ʶ��
	TxMessage.IDE = CAN_ID_STD;                          //ָ����Ҫ�������Ϣ�ı�ʶ��������
	TxMessage.RTR = CAN_RTR_DATA;                        //ָ����֡�����������Ϣ������   ����֡��Զ��֡
	TxMessage.DLC = 8;                                   //ָ�����ݵĳ���
	TxMessage.Data[0] = (unsigned char)(ESC_201>>8);
	TxMessage.Data[1] = (unsigned char)(ESC_201);
	TxMessage.Data[2] = (unsigned char)(ESC_202>>8);
	TxMessage.Data[3] = (unsigned char)(ESC_202);
	TxMessage.Data[4] = 0;
	TxMessage.Data[5] = 0;
	TxMessage.Data[6] = 0;
	TxMessage.Data[7] = 0;
	mbox=CAN_Transmit(CAN1,&TxMessage);   //������Ϣ
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;   //�ȴ����ͽ���
}	

/*
 * ���ܣ����̰�can1���͵����̵��
 */
void CAN1_SendCommand_Chassis(int16_t ESC_201,int16_t ESC_202,int16_t ESC_203,int16_t ESC_204)
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;              //����һ��������Ϣ�Ľṹ��
	
	TxMessage.StdId = 0x200;         //����820r���ñ�ʶ��   0x200
	TxMessage.IDE = CAN_ID_STD;      //ָ����Ҫ�������Ϣ�ı�ʶ��������
	TxMessage.RTR = CAN_RTR_DATA;    //ָ����֡�����������Ϣ������   ����֡��Զ��֡
	TxMessage.DLC = 8;               //ָ�����ݵĳ���
	TxMessage.Data[0] = (unsigned char)(ESC_201>>8);
	TxMessage.Data[1] = (unsigned char)(ESC_201);
	TxMessage.Data[2] = (unsigned char)(ESC_202>>8);
	TxMessage.Data[3] = (unsigned char)(ESC_202);
	TxMessage.Data[4] = (unsigned char)(ESC_203>>8);
	TxMessage.Data[5] = (unsigned char)(ESC_203);
	TxMessage.Data[6] = (unsigned char)(ESC_204>>8);
	TxMessage.Data[7] = (unsigned char)(ESC_204);
	mbox=CAN_Transmit(CAN1,&TxMessage);   //������Ϣ
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;   //�ȴ����ͽ���
}


/*
 * ���ܣ����̰�can1���͵���̨Y����͹������
 */
void CAN1_SendCommand_Gimbal_Fire(int16_t ESC_205,int16_t ESC_206,int16_t ESC_207)//Y������205��P������206,���������207
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;                                  //����һ��������Ϣ�Ľṹ��
	
	TxMessage.StdId = 0x1ff;                             //����820r���ñ�ʶ��
	TxMessage.IDE = CAN_ID_STD;                          //ָ����Ҫ�������Ϣ�ı�ʶ��������
	TxMessage.RTR = CAN_RTR_DATA;                        //ָ����֡�����������Ϣ������   ����֡��Զ��֡
	TxMessage.DLC = 8;                                   //ָ�����ݵĳ���
	TxMessage.Data[0] = (unsigned char)(ESC_205>>8);
	TxMessage.Data[1] = (unsigned char)(ESC_205);
	TxMessage.Data[2] = (unsigned char)(ESC_206>>8);
	TxMessage.Data[3] = (unsigned char)(ESC_206);
	TxMessage.Data[4] = (unsigned char)(ESC_207>>8);
	TxMessage.Data[5] = (unsigned char)(ESC_207);
	TxMessage.Data[6] = 0;
	TxMessage.Data[7] = 0;
	mbox=CAN_Transmit(CAN1,&TxMessage);   //������Ϣ
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;   //�ȴ����ͽ���
}	


/*
 * ���ܣ�can1���͵������ر�������ȡ
 */
void CAN1_SendCommand_Encoder_Read(void)
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;              //����һ��������Ϣ�Ľṹ��
	
	TxMessage.StdId = 0x02;          //���ñ�ʶ��  0x02
	TxMessage.IDE = CAN_ID_STD;      //ָ����Ҫ�������Ϣ�ı�ʶ��������
	TxMessage.RTR = CAN_RTR_DATA;    //ָ����֡�����������Ϣ������   ����֡��Զ��֡
	TxMessage.DLC = 4;               //ָ�����ݵĳ���
	TxMessage.Data[0] = 0x04;    //���ݳ���
	TxMessage.Data[1] = 0x02;    //��������ַ
	TxMessage.Data[2] = 0x01;    //ָ����  
	TxMessage.Data[3] = 0x00;    //����1

	mbox=CAN_Transmit(CAN1,&TxMessage);   //������Ϣ
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;   //�ȴ����ͽ���
}


/*************************************************************************CAN2����***********************************************************************************/

/*===ң��ģʽʱ����can2����ң��ֵ1����̨��===*/
	void CAN2_Send_Data1_RC_To_Gimbal(void)
	{
		u16 i=0;
		CanTxMsg TxMessage;
		
		TxMessage.StdId = 0x400;    
		TxMessage.IDE = CAN_ID_STD;    
		TxMessage.RTR = CAN_RTR_DATA;   
		TxMessage.DLC = 8;     
		
		u8 mbox;
		
		TxMessage.Data[0] = (unsigned char)(RC_Ctl.rc.ch2>>8);
		TxMessage.Data[1] = (unsigned char)(RC_Ctl.rc.ch2);
		TxMessage.Data[2] = (unsigned char)(RC_Ctl.rc.ch3>>8);
		TxMessage.Data[3] = (unsigned char)(RC_Ctl.rc.ch3);
		TxMessage.Data[4] = (unsigned char)(RC_Ctl.rc.sw>>8);
		TxMessage.Data[5] = (unsigned char)(RC_Ctl.rc.sw);
		TxMessage.Data[6] = (unsigned char)((RC_Ctl.rc.s1)*10+(RC_Ctl.rc.s2));
		TxMessage.Data[7] = (unsigned char)((RC_Ctl.mouse.press_l+1)*10 + RC_Ctl.mouse.press_r);
		mbox=CAN_Transmit(CAN2,&TxMessage);   
		i=0;
		while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;  
	}

/*===����ģʽʱ����can2����ң��ֵ1����̨��===*/
	void CAN2_Send_Data1_MK_To_Gimbal(void)
	{
		u16 i=0;
		CanTxMsg TxMessage;
		
		TxMessage.StdId = 0x401;    
		TxMessage.IDE = CAN_ID_STD;    
		TxMessage.RTR = CAN_RTR_DATA;   
		TxMessage.DLC = 8;     
		
		u8 mbox;
		
		TxMessage.Data[0] = (unsigned char)(RC_Ctl.mouse.x>>8);
		TxMessage.Data[1] = (unsigned char)(RC_Ctl.mouse.x);
		TxMessage.Data[2] = (unsigned char)(RC_Ctl.mouse.y>>8);
		TxMessage.Data[3] = (unsigned char)(RC_Ctl.mouse.y);
		TxMessage.Data[4] = (unsigned char)(RC_Ctl.key.v>>8);
		TxMessage.Data[5] = (unsigned char)(RC_Ctl.key.v);
		TxMessage.Data[6] = (unsigned char)((RC_Ctl.rc.s1)*10+(RC_Ctl.rc.s2));
		TxMessage.Data[7] = (unsigned char)((RC_Ctl.mouse.press_l+1)*10 + RC_Ctl.mouse.press_r);
		mbox=CAN_Transmit(CAN2,&TxMessage);   
		i=0;
		while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;  
	}
		
/*����can2����ң��ֵ2����̨��*/
		void CAN2_Send_Data2_To_Gimbal(void)
		{
			u8 mbox;
			u16 i=0;
			CanTxMsg TxMessage;    
			
			TxMessage.StdId = 0x402;    
			TxMessage.IDE = CAN_ID_STD;    
			TxMessage.RTR = CAN_RTR_DATA;    
			TxMessage.DLC = 8;     
			TxMessage.Data[0] = (unsigned char)(Yaw_Motor_Data.actual_Position>>24);
			TxMessage.Data[1] = (unsigned char)(Yaw_Motor_Data.actual_Position>>16);
			TxMessage.Data[2] = (unsigned char)(Yaw_Motor_Data.actual_Position>>8);
			TxMessage.Data[3] = (unsigned char)(Yaw_Motor_Data.actual_Position);
			TxMessage.Data[4] = (unsigned char)(Yaw_Motor_Data.speed>>8);
			TxMessage.Data[5] = (unsigned char)(Yaw_Motor_Data.speed);
			TxMessage.Data[6] = REFEREE.RobotStatus.shooter_heat0_speed_limit;//�������ƴ�С
			TxMessage.Data[7] = 0;//����
			
			mbox=CAN_Transmit(CAN2,&TxMessage);  
			i=0;
			while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++; 
		}
		


		
/*��̨can2����Y�� ��ʵ����ֵ �� ��ʼ����ɱ�־ �� ��̨������ ������*/
		void CAN2_Send_Yaw_Data_To_Chassis(u8 INITIALIZE_flag , int16_t gimbal_output , u8 YAW_ZERO_FLAG , u8 GIMBAL_SUPPLY_FLAG , int16_t Sec_chassis_angle)
		{
			u8 mbox;
			u16 i=0;
			CanTxMsg TxMessage;    
			
			TxMessage.StdId = 0x300;    
			TxMessage.IDE = CAN_ID_STD;    
			TxMessage.RTR = CAN_RTR_DATA;    
			TxMessage.DLC = 5;     

			TxMessage.Data[0] = (INITIALIZE_flag)*100+YAW_ZERO_FLAG*10+GIMBAL_SUPPLY_FLAG;
			TxMessage.Data[1] = (unsigned char)(gimbal_output>>8);
			TxMessage.Data[2] = (unsigned char)(gimbal_output);
			TxMessage.Data[3] = (unsigned char)(Sec_chassis_angle>>8);
			TxMessage.Data[4] = (unsigned char)(Sec_chassis_angle);
 			TxMessage.Data[5] = 
			TxMessage.Data[6] = // (unsigned char)(gimbal_output[1]);
			
			mbox=CAN_Transmit(CAN2,&TxMessage);  
			i=0;
			while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++; 
		}
	
		
		
		








