#ifndef __LED_H
#define __LED_H


//LED端口定义
#define LEDE0 PEout(0)  //遥控
#define LEDE1 PEout(1)  //底盘
#define LEDE2 PEout(2)	//云台	
#define LEDE3 PEout(3)  //火控	
#define LEDE4 PEout(4)	//安检
#define LEDB7 PBout(7)  //工作指示灯


void LED_Init(void);
void Laser_Init(void);
void Laser_ON(void);
void Double_GPIO_Init(void);
void Cap_OUT_IO_Init(void);

#endif
