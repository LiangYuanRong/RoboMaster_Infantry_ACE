#ifndef __REMOTETASK_H
#define __REMOTETASK_H
#include "remote.h"
#include "StartTask.h"

/*OS控制任务周期以及启动时间*/
#define REMOTE_TASK_INIT_TIME  5    //遥控任务启动缓冲时间
#define REMOTE_CONTROL_TIME_MS 7      //遥控任务循环时间

/*遥控设备选择*/
#define RC_CONTROL 		      ((uint16_t)1) //上
#define MOUSE_CONTROL 	    ((uint16_t)3) //中
#define ALL_STOP 	          ((uint16_t)2) //下

/*遥控三段开关切换*/
#define RC_SW_UP 	  ((uint16_t)1)  //上
#define RC_SW_MID 	((uint16_t)3)  //中
#define RC_SW_DOWN 	((uint16_t)2)  //下


/*键盘键位值定义*/
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0) //0x01
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)0x01<<8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)0x01<<9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)0x01<<10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)0x01<<11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)0x01<<12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)0x01<<13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)0x01<<14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)0x01<<15)


//处理后的遥控数据
typedef __packed struct
{
    float RC_ch0;
    float RC_ch1;
    float RC_ch2;
    float RC_ch3;
    float RC_sw;
} RC_Deal_data;

/*键盘按键结构体*/
typedef struct
{

    u8 KEY_F; //扭腰
    u8 KEY_R; //小陀螺
    u8 KEY_X; //
    u8 KEY_Z; //退弹
    u8 KEY_C; //低射速
    u8 KEY_V; //高射速
    u8 KEY_G; //补给模式
    u8 KEY_Q; //整车左旋
    u8 KEY_E; //整车右旋
    u8 Mouse_L;  //开火
    u8 Mouse_R;  //自动瞄准
} Key_Press_Sigh;



/*函数声明*/
void Remote_Task(void *pvParameters);    //遥控任务
void Select_Ctl_Mode(void);              //模式选择
void RC_Data_Process(void);              //遥控器数据处理
void MK_Data_Process(void);              //键盘数据处理
void Remote_reload(void);                //摇杆量清零


/*遥控处理值*/
extern RC_Deal_data        Remote_data;            //遥控处理后的值



#endif
