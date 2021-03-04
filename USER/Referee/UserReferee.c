/*************************************************************************************************
模块名称 : UserReferee
实现功能 : 发送用户自定义数据至裁判系统
备    注 : 无
-------------------------------------------------------------------------------------------------
日期 : 20-03-03
版本 : 1.0
功能 : 无
说明 : 通用
-------------------------------------------------------------------------------------------------
日期 : 20-04-10
版本 : 1.1
功能 : 适用用户所有发送操作
说明 : 若机器人间通信需要用到裁判数据则需要初始化，以获取裁判系统数据指针
*************************************************************************************************/

#include "UserReferee.h"
#include "RefereeDeal.h"
#include "CRC.h"
#include <stdlib.h>
#include "rt_heap.h"


/*裁判系统数据结构体*/
REFEREE_t *UserReffee;
/*初始化*/
void UserDataInit(void)
{
		UserReffee = Return_Referee_Point();
}


/*************************************************************************************************
*名称:	SendUserData
*功能:	发送用户自定义数据至裁判系统
*形参: 	DataLen：数据段长度
				seq：包序号0-255
				ContentID：自定义数据类型
				SendID：发送者ID
				ReceiveID：接收者ID
				data：数据
				user_data_t：用户数据结构体
*返回:	1 —— 失败
				0 —— 成功
*说明:	通用
				实际需要测试
*************************************************************************************************/
u8 SendUserData(uint16_t DataLen,u8 seq,uint16_t ContentID,uint16_t SendID,uint16_t ReceiveID,u8 *data)
{
	static int16_t CmdID = 0x0301;
	uint16_t i = 0;
	uint16_t Clock = 0;
	/*BCD转十进制*/
	uint16_t Len = ( ((DataLen>>8)*100) + ( ((DataLen>>4) & 0x0f) * 10 ) + (DataLen & 0x0f) );
	/*申请空间*/
	u8*user = (u8*)malloc( (sizeof(u8)) * (15 + Len) );
	if(user == NULL)
	{
		return 1;
	}
	
	/*帧头*/
	user[0] = 0xA5;
	user[1] = DataLen;
	user[2] = (DataLen >> 8);
	user[3] = seq;
	/*CRC-8*/
	Append_CRC8_Check_Sum(user,5);
	
	/*自定义数据类别*/
	user[5] = CmdID;
	user[6] = (CmdID >> 8);
	user[7] = ContentID;
	user[8] = (ContentID >> 8);
	user[9] = SendID;
	user[10] = (SendID >> 8);
	user[11] = ReceiveID;
	user[12] = (ReceiveID >> 8);
	
	/*自定义数据*/
	for(i = 13;i < (13 + Len);i ++)
			user[i] = data[i-13];
	/*CRC-16*/
	Append_CRC16_Check_Sum(user,(15 + Len));
	
	/*发送*/
	for(i = 0;i < (15 + Len);i ++)
	{
		Clock = 0;
		while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET)
		{
			Clock ++;
			if(Clock >= 0xf1f1)
			{
				free(user);
				user = NULL;
				return 1;
			}
		}
		USART_SendData(USART6,user[i]); 
	}
	
	free(user);
	user = NULL;
	return 0;
}



/*************************************************************************************************
*名称:	SendPowerHeat
*功能:	发送功率热量到客户端
*形参:	SendID	              ：机器人端ID
				ReceiveID						  ：对应客户端ID
*返回:	1 —— 失败
				0 —— 成功
*说明:	1. 机器人 ID：1，英雄(红)；2，工程(红)；3/4/5，步兵(红)；6，空中(红)；7，哨兵(红)；11，英雄(蓝)；
					 12，工程(蓝)；13/14/15，步兵(蓝)；16，空中(蓝)；17，哨兵(蓝)。 
				2. 客户端 ID：0x0101 为英雄操作手客户端(红)；0x0102，工程操作手客户端((红)；
					 0x0103/0x0104/0x0105，步兵操作手客户端(红)；0x0106，空中操作手客户端((红)； 0x0111，英雄操作手客户端(蓝)；
					 0x0112，工程操作手客户端(蓝)；0x0113/0x0114/0x0115，操作手客户端步兵(蓝)；0x0116，空中操作手客户端(蓝)
*************************************************************************************************/
u8 SendPowerHeat(uint16_t SendID,uint16_t ReceiveID)
{
	u8 data[13];
	Float_t p;
	p.value = UserReffee->PowerHeat.chassis_power;
	data[0] = p.float_byte.LB;
	data[1] = p.float_byte.MLB;
	data[2] = p.float_byte.MHB;
	data[3] = p.float_byte.HB;
	p.value = 0.0f;
	data[4] = p.float_byte.LB;
	data[5] = p.float_byte.MLB;
	data[6] = p.float_byte.MHB;
	data[7] = p.float_byte.HB;
	p.value = 0.0f;
	data[8] = p.float_byte.LB;
	data[9] = p.float_byte.MLB;
	data[10] = p.float_byte.MHB;
	data[11] = p.float_byte.HB;
	data[12] = 0x15;
	
	if(SendUserData(0x13,0x01,0xD180,SendID,ReceiveID,data) == 0)
		return 0;
	else
		return 1;
}



/*************************************************************************************************
*名称:	RobotCommuni
*功能:	机器人间通信
*形参: 	Len              ：数据段长度
				ContentID        ：0x0200~0x02FF
				SendID           ：发送者ID
				ReceiveID        ：接收者ID
				data             ：数据
*返回:	1 —— 失败
				0 —— 成功
*说明:	内容段ID最好不要冲突
*************************************************************************************************/
u8 RobotCommuni(uint16_t SendID,uint16_t ReceiveID,u8 Len,uint16_t ContentID,u8*data)
{
	/*十进制转BCD码*/
	uint16_t DataLen = ( ((Len/100)<<8) | ((Len/10%10)<<4) | (Len%10) );
	
	if(SendUserData(DataLen,0x02,ContentID,SendID,ReceiveID,data) == 0)
		return 0;
	else
		return 1;
}



/*************************************************************************************************
*名称:	DeleteAllMap 
*功能:	删除所有自定义图形
*形参: 	SendID           ：发送者ID
				ReceiveID        ：接收者ID
*返回:	1 —— 失败
				0 —— 成功
*说明:	无
*************************************************************************************************/
u8 DeleteAllMap(uint16_t SendID,uint16_t ReceiveID)
{
		u8 data[55] = {0};
		data[0] = 0x06;
		if(SendUserData(0x55,0x03,0x0100,SendID,ReceiveID,data) == 0)
		return 0;
	else
		return 1;
}



/*************************************************************************************************
*名称:	ClientMap 
*功能:	自定义图形
*形参: 	SendID           ：发送者ID
				ReceiveID        ：接收者ID
				ClientMap_t			 ：图形结构体
*返回:	1 —— 失败
				0 —— 成功
*说明:	无
*************************************************************************************************/
u8 ClientMap(uint16_t SendID,uint16_t ReceiveID,ClientMap_t*user)
{
		u8 data[55] = {0};
		u8 i = 0;
		
		data[0] = user->operate_tpye;
		data[1] = user->graphic_tpye;
		for(i = 0;i < 5;i ++)
			data[i+2] = user->graphic_name[i];
		data[7] = user->layer;
		data[8] = user->color;
		data[9] = user->width;
		data[10] = user->start_x;
		data[11] = (user->start_x>>8);
		data[12] = user->start_y;
		data[13] = (user->start_y>>8);
		data[14] = user->radius;
		data[15] = (user->radius>>8);
		data[16] = user->end_x;
		data[17] = (user->end_x>>8);
		data[18] = user->end_y;
		data[19] = (user->end_y>>8);
		data[20] = user->start_angle;
		data[21] = (user->start_angle>>8);
		data[22] = user->end_angle;
		data[23] = (user->end_angle>>8);
		i =  ( ((user->text_lenght/10)<<4) | (user->text_lenght%10) );
		data[24] = i;
		for(i = 0;i < user->text_lenght;i ++)
			data[25+i] = user->text[i];
		
		if(SendUserData(0x55,0x04,0x0100,SendID,ReceiveID,data) == 0)
			return 0;
		else
			return 1;
}
