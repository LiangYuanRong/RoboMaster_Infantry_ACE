#ifndef __REMOTE_H
#define __REMOTE_H
#include "sys.h"

/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W 		((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S 		((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A 		((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D 		((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_SHIFT 	((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_CTRL 	((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_Q 		((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_E 		((uint16_t)0x01<<7)
#define KEY_PRESSED_OFFSET_R 		((uint16_t)0x01<<8)
#define KEY_PRESSED_OFFSET_F 		((uint16_t)0x01<<9)
#define KEY_PRESSED_OFFSET_G 		((uint16_t)0x01<<10)
#define KEY_PRESSED_OFFSET_Z 		((uint16_t)0x01<<11)
#define KEY_PRESSED_OFFSET_X 		((uint16_t)0x01<<12)
#define KEY_PRESSED_OFFSET_C 		((uint16_t)0x01<<13)
#define KEY_PRESSED_OFFSET_V 		((uint16_t)0x01<<14)
#define KEY_PRESSED_OFFSET_B 		((uint16_t)0x01<<15)
/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
	int16_t sw;
	
}Remote;

typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;	

typedef	__packed struct
{
	uint16_t v;
}Key;

typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
	
}RC_Ctl_t;

/*函数声明*/
void Get_RC_Data(void);		              //获取数据
void RC_unable(void);                   //关闭遥控
void RC_restart(uint16_t dma_buf_num);  //遥控重启

extern RC_Ctl_t RC_Ctl;

#endif
