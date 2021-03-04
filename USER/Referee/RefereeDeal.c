/*************************************************************************************************
模块名称 : ReferDeal
实现功能 : 处理裁判系统数据
备    注 : 无
-------------------------------------------------------------------------------------------------
日期 : 20-03-02
版本 : 1.0
功能 : 接收比赛状态数据
说明 : 目前只有只处理了一类数据，且发送数据功能未完成
			 后续会补齐，并添加发送功能
			 但可以获取数据指针后，去写代码，并不影响
-------------------------------------------------------------------------------------------------
日期 : 20-04-09
版本 : 1.1
功能 : 增加接收机器人状态信息，功率热量信息
说明 : 目前只有只处理了一类数据
			 可以选择在中断接收数据，在外部处理数据 -> 通过REFEREE.RECEIVE_FLAG标志位
*************************************************************************************************/
#include "usart.h"
#include "RefereeDeal.h"
#include "CRC.h"

/*裁判系统数据结构体*/
REFEREE_t REFEREE;

/*************************************************************************************************
*名称:	Return_Referee_Point
*功能:	获得裁判系统数据控制量
*形参: 	无
*返回:	裁判系统数据控制指针 &REFEREE
*说明:	无
*************************************************************************************************/
REFEREE_t*Return_Referee_Point(void)
{
	return &REFEREE;
}



/*比赛状态*/
static void GAME_STATUS(REFEREE_t*referee,unsigned char k);
/*机器人状态*/
static void ROBOT_STATUS(REFEREE_t*referee,unsigned char k);
/*功率热量*/
static void POWER_HEAT(REFEREE_t*referee,unsigned char k);
	
/*数据段长度*/
static const u8 HeaderLen = 5;
static const u8 CmdIdLen  = 2;
static const u8 CRC16Len  = 2;
/*比赛状态数据段长度*/
static const u8 GameStatusLen  = (3 + CmdIdLen + CRC16Len);
/*机器人状态数据段长度*/
static const u8 RobotStatusLen = (18 + CmdIdLen + CRC16Len);
/*功率热量数据段长度*/
static const u8 PowerHeatLen   = (16 + CmdIdLen + CRC16Len);


/*中断*/
void USART6_IRQHandler(void)																		 
{ 
	unsigned short i = 0;
	unsigned short Len = 0;
	if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)					  //触发中断标志位
	{
		DMA_Cmd(DMA2_Stream1, DISABLE);															  //关闭DMA,防止处理期间有数据
		while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);
		USART6->SR;                                               
		USART6->DR;                                               
		Len = USART6_RX_LEN - DMA_GetCurrDataCounter(DMA2_Stream1);   //DMA_GetCurrDataCounter用于获得传输中剩余的单元数
		REFEREE.DataLen = Len;
		
		for(i = 0;i < Len;i ++)
		{	
			REFEREE.RefereeData[i] = Usart6_Rx[i];
		}
		
			RefereeDataDeal(&REFEREE);
		DMA_SetCurrDataCounter(DMA2_Stream1,USART6_RX_LEN);           //设置要传入的数据单元数
		DMA_Cmd(DMA2_Stream1, ENABLE);																//开启DMA
		REFEREE.RECEIVE_FLAG = 1;
	}
}




/*裁判数据接收数据处理*/
void RefereeDataDeal(REFEREE_t*referee)
{
	u8 i,k;
	for(i = 0;i < referee->DataLen;i ++)
	{
		if(referee->RefereeData[i] == 0xA5)																//帧头
		{
			if(Verify_CRC8_Check_Sum(referee->RefereeData,HeaderLen) == 1)	//CRC8校验
			{
				referee->RealLen = ( (referee->RefereeData[i + 1]) | (referee->RefereeData[i + 2] << 8) );											//数据长度
				referee->Cmd_ID = ( (referee->RefereeData[i + HeaderLen]) | (referee->RefereeData[i + HeaderLen + 1] << 8) );   //命令码ID
				
				for(k = 0;k < 7;k ++)
					referee->RealData[k] = referee->RefereeData[k + i];             //提取数据
				
				if(referee->Cmd_ID == 0x0001)
				{
					GAME_STATUS(referee,(i + HeaderLen + CmdIdLen));								//比赛状态数据处理
					i = i + HeaderLen + GameStatusLen - 1;
				}
				
				else if(referee->Cmd_ID == 0x0003)																//机器人血量
				{
				
				}
				else if(referee->Cmd_ID == 0x0004)																//飞镖发射状态
				{
					
				}
				else if(referee->Cmd_ID == 0x0101)																//场地事件数据
				{
					
				}
				else if(referee->Cmd_ID == 0x0102)																//补给站动作标识数据
				{
					
				}
				else if(referee->Cmd_ID == 0x0105)																//飞镖发射口数据
				{
					
				}
				else if(referee->Cmd_ID == 0x0201)																//机器人状态数据
				{
					ROBOT_STATUS(referee,(i + HeaderLen + CmdIdLen));
					i = i + HeaderLen + RobotStatusLen - 1;
				}
				else if(referee->Cmd_ID == 0x0202)																//功率热量数据
				{
					POWER_HEAT(referee,(i + HeaderLen + CmdIdLen));
					i = i + HeaderLen + PowerHeatLen - 1;
				}
				else if(referee->Cmd_ID == 0x0204)																//机器人增益数据
				{
					
				}
				else if(referee->Cmd_ID == 0x0205)																//飞机数据
				{
					
				}
				else if(referee->Cmd_ID == 0x0206)																//伤害状态
				{
					
				}
				else if(referee->Cmd_ID == 0x0207)																//实时射击信息
				{
					
				}
				else if(referee->Cmd_ID == 0x0208)																//子弹剩余发射数
				{
					
				}
				else if(referee->Cmd_ID == 0x0209)																//RFID状态
				{
					
				}
			}
		}
	}
	REFEREE.RECEIVE_FLAG = 0;					//处理完一组数据
}



/*比赛状态数据*/
static void GAME_STATUS(REFEREE_t*referee,unsigned char k)
{
	u8 j;
	for(j = 0;j < GameStatusLen;j ++)			//数据重组
			referee->RealData[HeaderLen + CmdIdLen + j] = referee->RefereeData[k + j];
	
	if(Verify_CRC16_Check_Sum(referee->RealData, (HeaderLen+GameStatusLen) ) == 1) //CRC16校验
	{
		referee->GameStatus.game_type = referee->RealData[7];
		referee->GameStatus.game_progress = (referee->RealData[7]>>4);
		referee->GameStatus.stage_remain_time = ( (referee->RealData[8]) | (referee->RealData[9]<<8) );
		referee->GameStatus.error = 0;
	}
	else
	{
		referee->GameStatus.error = 1;
	}
}


/*机器人状态数据*/
static void ROBOT_STATUS(REFEREE_t*referee,unsigned char k)
{
	u8 j;
	for(j = 0;j < RobotStatusLen;j ++)			//数据重组
		referee->RealData[HeaderLen + CmdIdLen + j] = referee->RefereeData[k + j];
	
		if(Verify_CRC16_Check_Sum(referee->RealData, (HeaderLen+RobotStatusLen) ) == 1) //CRC16校验
	{
		referee->RobotStatus.robot_id = referee->RealData[7];
		referee->RobotStatus.robot_level = referee->RealData[8];
		referee->RobotStatus.remain_HP = ( (referee->RealData[9]) | (referee->RealData[10]<<8) );
		referee->RobotStatus.max_HP = ( (referee->RealData[11]) | (referee->RealData[12]<<8) );
		referee->RobotStatus.shooter_heat0_cooling_rate = ( (referee->RealData[13]) | (referee->RealData[14]<<8) );
		referee->RobotStatus.shooter_heat0_cooling_limit = ( (referee->RealData[15]) | (referee->RealData[16]<<8) );
		referee->RobotStatus.shooter_heat1_cooling_rate = ( (referee->RealData[17]) | (referee->RealData[18]<<8) );
		referee->RobotStatus.shooter_heat1_cooling_limit = ( (referee->RealData[19]) | (referee->RealData[20]<<8) );
		referee->RobotStatus.shooter_heat0_speed_limit = (referee->RealData[21]);
		referee->RobotStatus.shooter_heat1_speed_limit = (referee->RealData[22]);
		referee->RobotStatus.max_chassis_power = (referee->RealData[23]);
		referee->RobotStatus.mains_power_gimbal_output = referee->RealData[24];
		referee->RobotStatus.mains_power_chassis_output = (referee->RealData[24]>>1);
		referee->RobotStatus.mains_power_shooter_output = (referee->RealData[24]>>2);
		
		referee->RobotStatus.error = 0;
	}
	else
	{
		referee->RobotStatus.error = 1;
	}
	
}

static void POWER_HEAT(REFEREE_t*referee,unsigned char k)
{
	u8 j;
	Float_t F;
	for(j = 0;j < PowerHeatLen;j ++)			//数据重组
			referee->RealData[HeaderLen + CmdIdLen + j] = referee->RefereeData[k + j];
	
	if(Verify_CRC16_Check_Sum(referee->RealData, (HeaderLen+PowerHeatLen) ) == 1) //CRC16校验
	{
			referee->PowerHeat.chassis_volt = ( (referee->RealData[7]) | (referee->RealData[8]<<8) );
			referee->PowerHeat.chassis_current = ( (referee->RealData[9]) | (referee->RealData[10]<<8) );
			F.float_byte.LB = (referee->RealData[11]);
			F.float_byte.MLB = (referee->RealData[12]);
			F.float_byte.MHB = (referee->RealData[13]);
			F.float_byte.HB = (referee->RealData[14]);
			referee->PowerHeat.chassis_power = F.value;
			referee->PowerHeat.chassis_power_buffer = ( (referee->RealData[15]) | (referee->RealData[16]<<8) );
			referee->PowerHeat.shooter_heat0 = ( (referee->RealData[17]) | (referee->RealData[18]<<8) );
			referee->PowerHeat.shooter_heat1 = ( (referee->RealData[19]) | (referee->RealData[20]<<8) );
			referee->PowerHeat.mobile_shooter_heat2 = ( (referee->RealData[21]) | (referee->RealData[22]<<8) );
	}
	else
	{
		referee->PowerHeat.error = 1;
	}
}
