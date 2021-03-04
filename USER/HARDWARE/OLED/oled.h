#ifndef __OLED_H
#define __OLED_H

#include "main.h"

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据


//-----------------OLED端口定义----------------




#define OLED_SCLK_Clr() GPIO_ResetBits(GPIOC,GPIO_Pin_8)
#define OLED_SCLK_Set() GPIO_SetBits(GPIOC,GPIO_Pin_8)

#define OLED_SDIN_Clr() GPIO_ResetBits(GPIOC,GPIO_Pin_10)
#define OLED_SDIN_Set() GPIO_SetBits(GPIOC,GPIO_Pin_10)

#define OLED_RST_Clr() GPIO_ResetBits(GPIOC,GPIO_Pin_12)
#define OLED_RST_Set() GPIO_SetBits(GPIOC,GPIO_Pin_12)

#define OLED_DC_Clr() GPIO_ResetBits(GPIOC,GPIO_Pin_13)
#define OLED_DC_Set() GPIO_SetBits(GPIOC,GPIO_Pin_13)

#define OLED_CS_Clr()  GPIO_ResetBits(GPIOD,GPIO_Pin_1)
#define OLED_CS_Set()  GPIO_SetBits(GPIOD,GPIO_Pin_1)

//OLED模式设置
//并行8080模式

#define SIZE 16
#define XLevelL		0x02
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF
#define X_WIDTH 	128
#define Y_WIDTH 	64

/*GUI状态*/
typedef enum
{
    INIT,           //初始化界面
    MAIN,           //主页面
    REMOTE_MONITOR, //监控器页面
    GYRO_MONITOR,   //陀螺仪监视页面
    MENU,           //菜单
    TWO_LEVEL_MENU, //二级菜单（测试，后面换成具体功能菜单）

} GUI_STATE;

/*按键结构体*/
typedef struct
{
    u8 KEY_DETERMINE;  //确认
    u8 KEY_BACK;       //返回
    u8 KEY_UP;         //向上
    u8 KEY_DOWN;       //向下

} Oled_Key_Press_Sigh;

typedef struct
{
    int button_flag[4];        //按键状态（1/0）
    int menu_flag;               //菜单数
    int oled_switch;             //切换页面时刷新屏幕标志位

    int option_flag;             //选项数
    int last_option_flag;      //上次上下翻的数据，选项上下翻直接减

} OLED_DATA_TYPE_DEF;


//结构体和变量定义
extern GUI_STATE GUI_state;       //图形用户界面枚举
extern Oled_Key_Press_Sigh  Oled_Key_Press; //按键消抖


//OLED控制用函数
void OLED_WR_Byte(u8 dat,u8 cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void OLED_ShowChar(u8 x,u8 y,u8 chr);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size2);
void OLED_ShowString(u8 x,u8 y, u8 *p);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(u8 x,u8 y,u8 no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);

//GUI相关显示函数
void GUI_Task(void);
void Oled_Display_Main(void);
void Oled_Display_Monitor_Remote(void);
void Oled_Display_Monitor_Gyro(void);
void Oled_Display_Menu(void);

#endif




