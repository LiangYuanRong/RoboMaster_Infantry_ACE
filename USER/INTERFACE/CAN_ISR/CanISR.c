#include "CanISR.h"
#include "PID_Def.h"
#include "Mode_Def.h"

#include "rmmotor.h"
#include "remote.h"
#include "RefereeDeal.h"

#include "Safecheck_Task.h"

//================参数定义
int32_t Encoder_yaw_code=0;                   //Y轴多圈绝对值编码器原始值
extern int16_t gimbal_output[2];              //云台电机处理输出    1：Y轴电机
extern int8_t YAW_ZERO_FLAG;                  //yaw轴中值光电标志
extern int8_t GIMBAL_SUPPLY_FLAG;             //云台补给标志位
extern int16_t Sec_chassis_angle;               //副云台相对底盘的差角

//================结构体定义
first_order_low_filter_type_t second_yaw_speed_lowfilt= {0.0f,0.0f,0.0f,0.1};//test
extern REFEREE_t REFEREE;             //裁判系统数据




/*
* 功能：CAN1接收返回的信息（电机及传感器）
* 数据接收： 云台板：P轴电机速度值反馈，P轴单圈编码器位置反馈，副云台yaw轴2006电机数据反馈
*           底盘板：底盘4电机数据反馈，yaw轴3508数据反馈，拨弹电机2006数据反馈，
 */ 
void CAN1_RX0_IRQHandler(void)                     
{
  CanRxMsg rx_message;    	
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);		
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0); 
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);       
		switch(rx_message.StdId)                                  //四个底盘电机
		{
	
#ifdef board_chassis	 //底盘四个电机 （底盘板）	
			/*底盘电机*/
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
			
			/*yaw轴3508电机速度*/
			case 0x205:
			{
				Yaw_Motor_Data.position = (rx_message.Data[0] << 8) | rx_message.Data[1];
				Motor_Actual_Position(&Yaw_Motor_Data,3*19,8192);//计算yaw电机的真实码盘值
				Yaw_Motor_Data.speed = (rx_message.Data[2] << 8) | rx_message.Data[3];
				break;	  
			}	
			
			/*拨弹电机*/                         
			case 0x207:  
			{
				Fire_GD_Motor.position = (rx_message.Data[0] << 8) | rx_message.Data[1];
				Fire_GD_Motor.speed = (rx_message.Data[2] << 8) | rx_message.Data[3];
				break;	
			}
			
#endif 
			

#ifdef board_gimbal   //云台can接收  （云台板）
			
			/*Pitch轴编码器位置反馈*/
			case 0x01: 
			{ 
//					if(rx_message.Data[1]==0x08) 
//					{
						Pitch_Motor_Data.position = (rx_message.Data[6]<<24 | rx_message.Data[5]<<16 | rx_message.Data[4]<<8 | rx_message.Data[3]); 
						Pitch_Motor_Data.actual_Position = Pitch_Motor_Data.position - Pitch_Middle_Angle;
//					}	
				break;
			}
			
		 /*P轴3510电机速度反馈*/                          
      case 0x201: 
			{
				Pitch_Motor_Data.speed = ((rx_message.Data[2] << 8) | rx_message.Data[3]);
				break;	
			}
			
  #ifdef double_gimbal
			/*副云台yaw轴2006电机数据反馈*/
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
* 功能：CAN2接收返回的信息（板间通信）
* 数据接收： 云台板：遥控数据（分遥控模式和RC模式，两者不共存），yaw轴电机真实位置，yaw轴电机速度
*           底盘板：云台初始化成功标志,yaw轴中值到达标志，yaw轴电机输出数据，补给状态标志
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
				RC_Ctl.rc.sw = (rx_Message.Data[4] << 8  | rx_Message.Data[5]);   //云台可以不用，后面换成裁判系统信息等		
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



/***************************************************CAN1发送*****************************************************************/


/*
 * 功能：云台板can1发送到P轴电机
 */
void CAN1_SendCommand_Gimbal_Pitch(int16_t ESC_201,int16_t ESC_202)//P轴电机是201,副云台yaw2006是202
{ 
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;                                  //定义一个发送信息的结构体
	
	TxMessage.StdId = 0x200;                             //根据820r设置标识符
	TxMessage.IDE = CAN_ID_STD;                          //指定将要传输的消息的标识符的类型
	TxMessage.RTR = CAN_RTR_DATA;                        //指定的帧将被传输的消息的类型   数据帧或远程帧
	TxMessage.DLC = 8;                                   //指定数据的长度
	TxMessage.Data[0] = (unsigned char)(ESC_201>>8);
	TxMessage.Data[1] = (unsigned char)(ESC_201);
	TxMessage.Data[2] = (unsigned char)(ESC_202>>8);
	TxMessage.Data[3] = (unsigned char)(ESC_202);
	TxMessage.Data[4] = 0;
	TxMessage.Data[5] = 0;
	TxMessage.Data[6] = 0;
	TxMessage.Data[7] = 0;
	mbox=CAN_Transmit(CAN1,&TxMessage);   //发送信息
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;   //等待发送结束
}	

/*
 * 功能：底盘板can1发送到底盘电机
 */
void CAN1_SendCommand_Chassis(int16_t ESC_201,int16_t ESC_202,int16_t ESC_203,int16_t ESC_204)
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;              //定义一个发送信息的结构体
	
	TxMessage.StdId = 0x200;         //根据820r设置标识符   0x200
	TxMessage.IDE = CAN_ID_STD;      //指定将要传输的消息的标识符的类型
	TxMessage.RTR = CAN_RTR_DATA;    //指定的帧将被传输的消息的类型   数据帧或远程帧
	TxMessage.DLC = 8;               //指定数据的长度
	TxMessage.Data[0] = (unsigned char)(ESC_201>>8);
	TxMessage.Data[1] = (unsigned char)(ESC_201);
	TxMessage.Data[2] = (unsigned char)(ESC_202>>8);
	TxMessage.Data[3] = (unsigned char)(ESC_202);
	TxMessage.Data[4] = (unsigned char)(ESC_203>>8);
	TxMessage.Data[5] = (unsigned char)(ESC_203);
	TxMessage.Data[6] = (unsigned char)(ESC_204>>8);
	TxMessage.Data[7] = (unsigned char)(ESC_204);
	mbox=CAN_Transmit(CAN1,&TxMessage);   //发送信息
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;   //等待发送结束
}


/*
 * 功能：底盘板can1发送到云台Y电机和供弹电机
 */
void CAN1_SendCommand_Gimbal_Fire(int16_t ESC_205,int16_t ESC_206,int16_t ESC_207)//Y轴电机是205，P轴电机是206,供弹电机是207
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;                                  //定义一个发送信息的结构体
	
	TxMessage.StdId = 0x1ff;                             //根据820r设置标识符
	TxMessage.IDE = CAN_ID_STD;                          //指定将要传输的消息的标识符的类型
	TxMessage.RTR = CAN_RTR_DATA;                        //指定的帧将被传输的消息的类型   数据帧或远程帧
	TxMessage.DLC = 8;                                   //指定数据的长度
	TxMessage.Data[0] = (unsigned char)(ESC_205>>8);
	TxMessage.Data[1] = (unsigned char)(ESC_205);
	TxMessage.Data[2] = (unsigned char)(ESC_206>>8);
	TxMessage.Data[3] = (unsigned char)(ESC_206);
	TxMessage.Data[4] = (unsigned char)(ESC_207>>8);
	TxMessage.Data[5] = (unsigned char)(ESC_207);
	TxMessage.Data[6] = 0;
	TxMessage.Data[7] = 0;
	mbox=CAN_Transmit(CAN1,&TxMessage);   //发送信息
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;   //等待发送结束
}	


/*
 * 功能：can1发送到布瑞特编码器读取
 */
void CAN1_SendCommand_Encoder_Read(void)
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;              //定义一个发送信息的结构体
	
	TxMessage.StdId = 0x02;          //设置标识符  0x02
	TxMessage.IDE = CAN_ID_STD;      //指定将要传输的消息的标识符的类型
	TxMessage.RTR = CAN_RTR_DATA;    //指定的帧将被传输的消息的类型   数据帧或远程帧
	TxMessage.DLC = 4;               //指定数据的长度
	TxMessage.Data[0] = 0x04;    //数据长度
	TxMessage.Data[1] = 0x02;    //编码器地址
	TxMessage.Data[2] = 0x01;    //指令码  
	TxMessage.Data[3] = 0x00;    //数据1

	mbox=CAN_Transmit(CAN1,&TxMessage);   //发送信息
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;   //等待发送结束
}


/*************************************************************************CAN2发送***********************************************************************************/

/*===遥控模式时底盘can2发送遥控值1到云台板===*/
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

/*===键盘模式时底盘can2发送遥控值1到云台板===*/
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
		
/*底盘can2发送遥控值2到云台板*/
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
			TxMessage.Data[6] = REFEREE.RobotStatus.shooter_heat0_speed_limit;//射速限制大小
			TxMessage.Data[7] = 0;//热量
			
			mbox=CAN_Transmit(CAN2,&TxMessage);  
			i=0;
			while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++; 
		}
		


		
/*云台can2发送Y轴 真实码盘值 和 初始化完成标志 和 云台控制量 到底盘*/
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
	
		
		
		








