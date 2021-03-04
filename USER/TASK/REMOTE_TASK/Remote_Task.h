#ifndef __REMOTETASK_H
#define __REMOTETASK_H
#include "remote.h"
#include "StartTask.h"

/*OS�������������Լ�����ʱ��*/
#define REMOTE_TASK_INIT_TIME  5    //ң��������������ʱ��
#define REMOTE_CONTROL_TIME_MS 7      //ң������ѭ��ʱ��

/*ң���豸ѡ��*/
#define RC_CONTROL 		      ((uint16_t)1) //��
#define MOUSE_CONTROL 	    ((uint16_t)3) //��
#define ALL_STOP 	          ((uint16_t)2) //��

/*ң�����ο����л�*/
#define RC_SW_UP 	  ((uint16_t)1)  //��
#define RC_SW_MID 	((uint16_t)3)  //��
#define RC_SW_DOWN 	((uint16_t)2)  //��


/*���̼�λֵ����*/
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


//������ң������
typedef __packed struct
{
    float RC_ch0;
    float RC_ch1;
    float RC_ch2;
    float RC_ch3;
    float RC_sw;
} RC_Deal_data;

/*���̰����ṹ��*/
typedef struct
{

    u8 KEY_F; //Ť��
    u8 KEY_R; //С����
    u8 KEY_X; //
    u8 KEY_Z; //�˵�
    u8 KEY_C; //������
    u8 KEY_V; //������
    u8 KEY_G; //����ģʽ
    u8 KEY_Q; //��������
    u8 KEY_E; //��������
    u8 Mouse_L;  //����
    u8 Mouse_R;  //�Զ���׼
} Key_Press_Sigh;



/*��������*/
void Remote_Task(void *pvParameters);    //ң������
void Select_Ctl_Mode(void);              //ģʽѡ��
void RC_Data_Process(void);              //ң�������ݴ���
void MK_Data_Process(void);              //�������ݴ���
void Remote_reload(void);                //ҡ��������


/*ң�ش���ֵ*/
extern RC_Deal_data        Remote_data;            //ң�ش�����ֵ



#endif
