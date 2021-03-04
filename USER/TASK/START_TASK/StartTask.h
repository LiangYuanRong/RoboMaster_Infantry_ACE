#ifndef __STARTTASK_H
#define __STARTTASK_H

void Start_Task(void);


/*************系统的一些模式枚举*****************/

/*整机工作状态*/
typedef enum 
{
	INITIALIZE,   //初始化状态
	WORKING,      //手动状态
	AUTOATTACK,   //自瞄状态
	AUTOBUFF,     //打符状态
	REPLENISHMEN, //补给状态
	DOUBLE_GIMBAL,//双云台状态（主云台自瞄，副云台操作） 
	POWEROFF,     //待机状态
	
}WorkStatus;



/*外部使用声明*/ 
extern WorkStatus          workStatus;               //默认工作模式为待机模式



#endif
