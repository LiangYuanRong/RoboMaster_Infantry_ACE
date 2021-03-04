
//#include "mcusys.h"
#include "oled.h"
#include "oledfont.h"
#include "delay.h"


//OLED���Դ�
//��Ÿ�ʽ����.
//[0]0 1 2 3 ... 127
//[1]0 1 2 3 ... 127
//[2]0 1 2 3 ... 127
//[3]0 1 2 3 ... 127
//[4]0 1 2 3 ... 127
//[5]0 1 2 3 ... 127
//[6]0 1 2 3 ... 127
//[7]0 1 2 3 ... 127

//��SSD1306д��һ���ֽڡ�
//dat:Ҫд�������/����
//cmd:����/�����־ 0,��ʾ����;1,��ʾ����;
void OLED_WR_Byte(u8 dat,u8 cmd)
{
    u8 i;
    if(cmd)
        OLED_DC_Set();
    else
        OLED_DC_Clr();
    OLED_CS_Clr();
    for(i=0; i<8; i++)
    {
        OLED_SCLK_Clr();
        if(dat&0x80)
        {
            OLED_SDIN_Set();
        }
        else
            OLED_SDIN_Clr();
        OLED_SCLK_Set();

        dat<<=1;
    }
    OLED_CS_Set();
    OLED_DC_Set();
}



void OLED_Set_Pos(unsigned char x, unsigned char y)
{
    OLED_WR_Byte(0xb0+y,OLED_CMD);
    OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
    OLED_WR_Byte((x&0x0f)|0x01,OLED_CMD);
}


//����OLED��ʾ
void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
    OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
    OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}


//�ر�OLED��ʾ
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
    OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
    OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}



//��������,������,������Ļ�Ǻ�ɫ��!��û����һ��!!!
void OLED_Clear(void)
{
    u8 i,n;
    for(i=0; i<8; i++)
    {
        OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
        OLED_WR_Byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
        OLED_WR_Byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ
        for(n=0; n<128; n++)OLED_WR_Byte(0,OLED_DATA);
    } //������ʾ
}


//��ָ��λ����ʾһ���ַ�,���������ַ�
//x:0~127
//y:0~63
//mode:0,������ʾ;1,������ʾ
//size:ѡ������ 16/12
void OLED_ShowChar(u8 x,u8 y,u8 chr)
{
    unsigned char c=0,i=0;
    c=chr-' ';//�õ�ƫ�ƺ��ֵ
    if(x>Max_Column-1) {
        x=0;
        y=y+2;
    }
    if(SIZE ==16)
    {
        OLED_Set_Pos(x,y);
        for(i=0; i<8; i++)
            OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
        OLED_Set_Pos(x,y+1);
        for(i=0; i<8; i++)
            OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
    }
    else {
        OLED_Set_Pos(x,y+1);
        for(i=0; i<6; i++)
            OLED_WR_Byte(F6x8[c][i],OLED_DATA);

    }
}



//m^n����
u32 oled_pow(u8 m,u8 n)
{
    u32 result=1;
    while(n--)result*=m;
    return result;
}


//��ʾ2������
//x,y :�������
//len :���ֵ�λ��
//size:�����С
//mode:ģʽ	0,���ģʽ;1,����ģʽ
//num:��ֵ(0~4294967295);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size2)
{
    u8 t,temp;
    u8 enshow=0;
    for(t=0; t<len; t++)
    {
        temp=(num/oled_pow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1))
        {
            if(temp==0)
            {
                OLED_ShowChar(x+(size2/2)*t,y,' ');
                continue;
            } else enshow=1;

        }
        OLED_ShowChar(x+(size2/2)*t,y,temp+'0');
    }
}



//��ʾһ���ַ��Ŵ�
void OLED_ShowString(u8 x,u8 y,u8 *chr)
{
    unsigned char j=0;
    while (chr[j]!='\0')
    {   OLED_ShowChar(x,y,chr[j]);
        x+=8;
        if(x>120) {
            x=0;
            y+=2;
        }
        j++;
    }
}



//��ʾ����
void OLED_ShowCHinese(u8 x,u8 y,u8 no)
{
    u8 t,adder=0;
    OLED_Set_Pos(x,y);
    for(t=0; t<16; t++)
    {
        OLED_WR_Byte(Hzk[2*no][t],OLED_DATA);
        adder+=1;
    }
    OLED_Set_Pos(x,y+1);
    for(t=0; t<16; t++)
    {
        OLED_WR_Byte(Hzk[2*no+1][t],OLED_DATA);
        adder+=1;
    }
}






/***********������������ʾ��ʾBMPͼƬ128��64��ʼ������(x,y),x�ķ�Χ0��127��yΪҳ�ķ�Χ0��7*****************/
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
{
    unsigned int j=0;
    unsigned char x,y;

    if(y1%8==0) y=y1/8;
    else y=y1/8+1;
    for(y=y0; y<y1; y++)
    {
        OLED_Set_Pos(x0,y);
        for(x=x0; x<x1; x++)
        {
            OLED_WR_Byte(BMP[j++],OLED_DATA);
        }
    }
}



//��ʼ��SSD1306
void OLED_Init(void)
{

    GPIO_InitTypeDef  GPIO_InitStructure;					//����IO���ýṹ��

//	//Ҫ�õ������PA15  ���Թص�JTAG���湦��
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	  //����AFIOʱ��
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //����JTAG-DP Disabled and SW-DP Enabled
//	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE); //JTAG and SWD ȫ������


    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOD,ENABLE);	 //ʹ��PB,D�˿�ʱ��

    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_12|GPIO_Pin_13; //�������
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIO


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				 //�������

    GPIO_Init(GPIOD, &GPIO_InitStructure);





    OLED_RST_Set();
    delay_ms(100);
    OLED_RST_Clr();
    delay_ms(100);
    OLED_RST_Set();


    OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
    OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
    OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
    OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
    OLED_WR_Byte(0xCF,OLED_CMD); // Set SEG Output Current Brightness
    OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0���ҷ��� 0xa1����
    OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0���·��� 0xc8����
    OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
    OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
    OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
    OLED_WR_Byte(0x00,OLED_CMD);//-not offset
    OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
    OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
    OLED_WR_Byte(0x12,OLED_CMD);
    OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
    OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
    OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WR_Byte(0x02,OLED_CMD);//
    OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
    OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
    OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
    OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7)
    OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel

    OLED_WR_Byte(0xAF,OLED_CMD); /*display ON*/
    OLED_Clear();
    OLED_Set_Pos(0,0);

}





/***************************************************************Ӧ�ò�********************************************************************************/
GUI_STATE GUI_state = INIT;    //ͼ���û�����ö��
Oled_Key_Press_Sigh  Oled_Key_Press= {0}; //��������
 

/*****����������ͼ���û�����v1.0********/
OLED_DATA_TYPE_DEF OLED_Data= {{0,0,0,0},0,0,0,0};

extern unsigned char BMP_Ugly_smile[];
extern unsigned char BMP_Laugh[];
extern unsigned char BMP_Smile[];

void GUI_Task(void)
{
    /***************����״̬�л�********************/
//    {
//        //��ȡ����ֵ
//        OLED_Data.button_flag[0] = !GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8);
//        OLED_Data.button_flag[1] = !GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
////		button_flag[2] = !GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2);
////		button_flag[3] = !GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4);

        //��¼����ֵ
//        /*ȷ�ϼ�*/
//        if(OLED_Data.button_flag[0]==1 || PS2_Data.PS2_KEY==14 )
//        {
//            delay_ms(10);
//            if(OLED_Data.button_flag[0]==1 || PS2_Data.PS2_KEY==14 )//����
//            {
//                if(Oled_Key_Press.KEY_DETERMINE == 0)
//                {
//                    OLED_Data.menu_flag++;
//                    if(OLED_Data.menu_flag>3)OLED_Data.menu_flag=3;
//                }
//                Oled_Key_Press.KEY_DETERMINE = 1;
//            }
//        }
//        else
//        {
//            Oled_Key_Press.KEY_DETERMINE = 0;
//        }

//        /*���ؼ�*/
//        if(OLED_Data.button_flag[1]==1|| PS2_Data.PS2_KEY==15)
//        {
//            delay_ms(10);
//            if(OLED_Data.button_flag[1]==1 || PS2_Data.PS2_KEY==15)//����
//            {
//                if(Oled_Key_Press.KEY_BACK == 0)
//                {
//                    OLED_Data.menu_flag--;
//                    if(OLED_Data.menu_flag<0)OLED_Data.menu_flag=0;
//                }
//                Oled_Key_Press.KEY_BACK = 1;
//            }
//        }
//        else
//        {
//            Oled_Key_Press.KEY_BACK = 0;
//        }

//        /*�Ϸ���*/
//        if( PS2_Data.PS2_KEY==7 /*button_flag[2]==1*/)
//        {
//            if(Oled_Key_Press.KEY_UP == 0)
//            {
//                OLED_Data.option_flag++;
//                if(OLED_Data.option_flag>3)OLED_Data.option_flag=3;
//            }
//            Oled_Key_Press.KEY_UP = 1;
//        }
//        else
//        {
//            Oled_Key_Press.KEY_UP = 0;
//        }

//        /*�·���*/
//        if( PS2_Data.PS2_KEY==5 /*button_flag[3]==1*/)
//        {
//            if(Oled_Key_Press.KEY_DOWN == 0)
//            {
//                OLED_Data.option_flag--;
//                if(OLED_Data.option_flag<0)OLED_Data.option_flag=0;
//            }
//            Oled_Key_Press.KEY_DOWN = 1;
//        }
//        else
//        {
//            Oled_Key_Press.KEY_DOWN = 0;
//        }
//    }

    //GUI�˵��л�
    {
        switch(OLED_Data.menu_flag)
        {
        case 0:
            GUI_state = MAIN;
            break;
        case 1:
            GUI_state = REMOTE_MONITOR;
            break;
        case 2:
            GUI_state = GYRO_MONITOR;
            break;
        case 3:
            GUI_state = MENU;
            break;
        }
    }
    //GUI��ʾ
    {
        if(GUI_state==MAIN)
        {
            if(OLED_Data.oled_switch!=0) //�����л�ˢ��
            {
                OLED_Clear();
                OLED_Data.oled_switch=0;
            }
            Oled_Display_Main();  //������
        }
        else if(GUI_state==REMOTE_MONITOR)
        {
            if(OLED_Data.oled_switch!=1) //�����л�ˢ��
            {
                OLED_Clear();
            }
            Oled_Display_Monitor_Remote(); //ң�ؼ���������
        }
        else if(GUI_state==GYRO_MONITOR)
        {
            if(OLED_Data.oled_switch!=2) //�����л�ˢ��
            {
                OLED_Clear();
            }
            Oled_Display_Monitor_Gyro(); //�����Ǽ���������
        }
        else if(GUI_state==MENU)
        {
            if(OLED_Data.oled_switch!=3) //�����л�ˢ��
            {
                OLED_Clear();
                OLED_Data.oled_switch=3;
            }
            Oled_Display_Menu(); //�˵�����
        }
    }
}




/****************������******************/
void Oled_Display_Main(void)
{
    //������ʾ(���ᣬ���ᣬ�ַ���)
    OLED_ShowCHinese(50,0,21);  //A�ձ�

    OLED_ShowString(19,2,"ACE");//ACEʵ����
    OLED_ShowCHinese(19+26,2,14);
    OLED_ShowCHinese(19+26+18,2,15);
    OLED_ShowCHinese(19+26+36,2,16);

    OLED_ShowCHinese(18,4,17); //����������
    OLED_ShowCHinese(36,4,18);
    OLED_ShowCHinese(54,4,11);
    OLED_ShowCHinese(72,4,12);
    OLED_ShowCHinese(90,4,13);
    OLED_ShowCHinese(0,6,22); //R�ձ�
    OLED_ShowString(20,6, "   GUI   v1.0");
//		delay_ms(300);
}

/**************ң��״̬���ӽ���*************/

void Oled_Display_Monitor_Remote(void)
{
    if(OLED_Data.oled_switch != 1)
    {
        OLED_Data.option_flag = 0;
        OLED_Data.last_option_flag = OLED_Data.option_flag;
        OLED_Data.oled_switch=1;
    }

    if((OLED_Data.last_option_flag != OLED_Data.option_flag)) //���ݱ仯ˢ��
    {
        OLED_Clear();
    }

    //�ַ�����ʾ
    OLED_ShowString(76,0, "K:");
//    OLED_ShowNum(88,0, PS2_Data.PS2_KEY ,2,18);  //option_flag

//    OLED_ShowString(9,0, "LX:");
//    OLED_ShowNum(30,0, PS2_Data.PS2_LX ,3,18);  //option_flag

//    OLED_ShowString(9,2, "LY:");
//    OLED_ShowNum(30,2, PS2_Data.PS2_LY ,3,18);  //option_flag

//    OLED_ShowString(9,4, "RX:");
//    OLED_ShowNum(30,4, PS2_Data.PS2_RX ,3,18);  //option_flag

//    OLED_ShowString(9,6, "RY:");
//    OLED_ShowNum(30,6, PS2_Data.PS2_RY ,3,18);  //option_flag

    //ѡ���л�
    if(OLED_Data.option_flag>3)OLED_Data.option_flag=3;
    if(OLED_Data.option_flag<0)OLED_Data.option_flag=0;


    OLED_ShowChar(0,OLED_Data.option_flag*2,'>');
    OLED_Data.last_option_flag = OLED_Data.option_flag;

//    {
//        last_key=PS2_Data.PS2_KEY;
//        last_lx =PS2_Data.PS2_LX;
//        last_ly =PS2_Data.PS2_LY;
//        last_rx =PS2_Data.PS2_RX;
//        last_ry =PS2_Data.PS2_RY;
//    }
}

/**************������״̬���ӽ���*************/

void Oled_Display_Monitor_Gyro(void)
{
    if(OLED_Data.oled_switch != 2)
    {
        OLED_Data.option_flag = 0;
        OLED_Data.last_option_flag = OLED_Data.option_flag;
        OLED_Data.oled_switch=2;
    }
    if((OLED_Data.last_option_flag != OLED_Data.option_flag)) //���ݱ仯ˢ��
    {
        OLED_Clear();
    }

    //�ַ�����ʾ
    OLED_ShowString(9,0, "Pitch:");
//    OLED_ShowNum(58,0, (int)MPU6050_Data.pitch_angle+90 ,3,18);  //option_flag

    OLED_ShowString(9,2, "Yaw:");
//    OLED_ShowNum(58,2, (int)MPU6050_Data.yaw_angle+90 ,3,18);  //option_flag

    OLED_ShowString(9,4, "Roll:");
//    OLED_ShowNum(58,4, (int)MPU6050_Data.roll_angle+90 ,3,18);  //option_flag

//	  OLED_ShowString(9,6, "Temp:");
//	  OLED_ShowNum(58,6, (int)MPU6050_Data.Temp ,3,18);  //option_flag

    //ѡ���л�
    if(OLED_Data.option_flag>3)OLED_Data.option_flag=3;
    if(OLED_Data.option_flag<0)OLED_Data.option_flag=0;


    OLED_ShowChar(0,OLED_Data.option_flag*2,'>');
    OLED_Data.last_option_flag = OLED_Data.option_flag;

//    {
//        last_pitch = (int)MPU6050_Data.pitch_angle;
//        last_yaw   = (int)MPU6050_Data.yaw_angle;
//        last_roll  = (int)MPU6050_Data.roll_angle;
//    }
}





/**************�˵�����(���ޣ���ʾͼƬ)*****************/
void Oled_Display_Menu(void)
{
    //BMPͼƬ��ʾ
    OLED_DrawBMP(0,0,127,7,BMP_Ugly_smile);

}




















