#ifndef __OLED_H
#define __OLED_H

#include "main.h"

#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����


//-----------------OLED�˿ڶ���----------------




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

//OLEDģʽ����
//����8080ģʽ

#define SIZE 16
#define XLevelL		0x02
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF
#define X_WIDTH 	128
#define Y_WIDTH 	64

/*GUI״̬*/
typedef enum
{
    INIT,           //��ʼ������
    MAIN,           //��ҳ��
    REMOTE_MONITOR, //�����ҳ��
    GYRO_MONITOR,   //�����Ǽ���ҳ��
    MENU,           //�˵�
    TWO_LEVEL_MENU, //�����˵������ԣ����滻�ɾ��幦�ܲ˵���

} GUI_STATE;

/*�����ṹ��*/
typedef struct
{
    u8 KEY_DETERMINE;  //ȷ��
    u8 KEY_BACK;       //����
    u8 KEY_UP;         //����
    u8 KEY_DOWN;       //����

} Oled_Key_Press_Sigh;

typedef struct
{
    int button_flag[4];        //����״̬��1/0��
    int menu_flag;               //�˵���
    int oled_switch;             //�л�ҳ��ʱˢ����Ļ��־λ

    int option_flag;             //ѡ����
    int last_option_flag;      //�ϴ����·������ݣ�ѡ�����·�ֱ�Ӽ�

} OLED_DATA_TYPE_DEF;


//�ṹ��ͱ�������
extern GUI_STATE GUI_state;       //ͼ���û�����ö��
extern Oled_Key_Press_Sigh  Oled_Key_Press; //��������


//OLED�����ú���
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

//GUI�����ʾ����
void GUI_Task(void);
void Oled_Display_Main(void);
void Oled_Display_Monitor_Remote(void);
void Oled_Display_Monitor_Gyro(void);
void Oled_Display_Menu(void);

#endif




