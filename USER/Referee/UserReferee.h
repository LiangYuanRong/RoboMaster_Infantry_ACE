#ifndef __USERREFEREE_H_
#define __USERREFEREE_H_
#include "stm32f4xx.h"

typedef __packed struct{ 
	uint8_t operate_tpye; 
	uint8_t graphic_tpye; 
	uint8_t graphic_name[5]; 
	uint8_t layer; 
	uint8_t color; 
	uint8_t width; 
	uint16_t start_x; 
	uint16_t start_y;  
	uint16_t radius; 
	uint16_t end_x; 
	uint16_t end_y; 
	int16_t start_angle;
	int16_t end_angle;
	uint8_t text_lenght;
	uint8_t text[30]; 
}ClientMap_t;

u8 SendUserData(uint16_t DataLen,u8 seq,uint16_t ContentID,uint16_t SendID,uint16_t ReceiveID,u8 *data);
u8 SendPowerHeat(uint16_t SendID,uint16_t ReceiveID);
u8 RobotCommuni(uint16_t SendID,uint16_t ReceiveID,u8 Len,uint16_t ContentID,u8*data);
u8 DeleteAllMap(uint16_t SendID,uint16_t ReceiveID);
u8 ClientMap(uint16_t SendID,uint16_t ReceiveID,ClientMap_t*user);
#endif
