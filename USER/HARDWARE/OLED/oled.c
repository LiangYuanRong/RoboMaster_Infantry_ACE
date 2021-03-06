
//#include "mcusys.h"
#include "oled.h"
#include "oledfont.h"
#include "delay.h"


//OLED的显存
//存放格式如下.
//[0]0 1 2 3 ... 127
//[1]0 1 2 3 ... 127
//[2]0 1 2 3 ... 127
//[3]0 1 2 3 ... 127
//[4]0 1 2 3 ... 127
//[5]0 1 2 3 ... 127
//[6]0 1 2 3 ... 127
//[7]0 1 2 3 ... 127

//向SSD1306写入一个字节。
//dat:要写入的数据/命令
//cmd:数据/命令标志 0,表示命令;1,表示数据;
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


//开启OLED显示
void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
    OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
    OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}


//关闭OLED显示
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
    OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
    OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}



//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!
void OLED_Clear(void)
{
    u8 i,n;
    for(i=0; i<8; i++)
    {
        OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
        OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
        OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址
        for(n=0; n<128; n++)OLED_WR_Byte(0,OLED_DATA);
    } //更新显示
}


//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示
//size:选择字体 16/12
void OLED_ShowChar(u8 x,u8 y,u8 chr)
{
    unsigned char c=0,i=0;
    c=chr-' ';//得到偏移后的值
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



//m^n函数
u32 oled_pow(u8 m,u8 n)
{
    u32 result=1;
    while(n--)result*=m;
    return result;
}


//显示2个数字
//x,y :起点坐标
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);
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



//显示一个字符号串
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



//显示汉字
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






/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
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



//初始化SSD1306
void OLED_Init(void)
{

    GPIO_InitTypeDef  GPIO_InitStructure;					//定义IO配置结构体

//	//要用到仿真口PA15  所以关掉JTAG仿真功能
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	  //启动AFIO时钟
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //配置JTAG-DP Disabled and SW-DP Enabled
//	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE); //JTAG and SWD 全部禁用


    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOD,ENABLE);	 //使能PB,D端口时钟

    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_12|GPIO_Pin_13; //推挽输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				 //推挽输出

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
    OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
    OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
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





/***************************************************************应用层********************************************************************************/
GUI_STATE GUI_state = INIT;    //图形用户界面枚举
Oled_Key_Press_Sigh  Oled_Key_Press= {0}; //按键消抖
 

/*****步兵机器人图形用户界面v1.0********/
OLED_DATA_TYPE_DEF OLED_Data= {{0,0,0,0},0,0,0,0};

extern unsigned char BMP_Ugly_smile[];
extern unsigned char BMP_Laugh[];
extern unsigned char BMP_Smile[];

void GUI_Task(void)
{
    /***************按键状态切换********************/
//    {
//        //读取按键值
//        OLED_Data.button_flag[0] = !GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8);
//        OLED_Data.button_flag[1] = !GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
////		button_flag[2] = !GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2);
////		button_flag[3] = !GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4);

        //记录按键值
//        /*确认键*/
//        if(OLED_Data.button_flag[0]==1 || PS2_Data.PS2_KEY==14 )
//        {
//            delay_ms(10);
//            if(OLED_Data.button_flag[0]==1 || PS2_Data.PS2_KEY==14 )//消抖
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

//        /*返回键*/
//        if(OLED_Data.button_flag[1]==1|| PS2_Data.PS2_KEY==15)
//        {
//            delay_ms(10);
//            if(OLED_Data.button_flag[1]==1 || PS2_Data.PS2_KEY==15)//消抖
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

//        /*上翻键*/
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

//        /*下翻键*/
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

    //GUI菜单切换
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
    //GUI显示
    {
        if(GUI_state==MAIN)
        {
            if(OLED_Data.oled_switch!=0) //初次切换刷新
            {
                OLED_Clear();
                OLED_Data.oled_switch=0;
            }
            Oled_Display_Main();  //主界面
        }
        else if(GUI_state==REMOTE_MONITOR)
        {
            if(OLED_Data.oled_switch!=1) //初次切换刷新
            {
                OLED_Clear();
            }
            Oled_Display_Monitor_Remote(); //遥控监视器界面
        }
        else if(GUI_state==GYRO_MONITOR)
        {
            if(OLED_Data.oled_switch!=2) //初次切换刷新
            {
                OLED_Clear();
            }
            Oled_Display_Monitor_Gyro(); //陀螺仪监视器界面
        }
        else if(GUI_state==MENU)
        {
            if(OLED_Data.oled_switch!=3) //初次切换刷新
            {
                OLED_Clear();
                OLED_Data.oled_switch=3;
            }
            Oled_Display_Menu(); //菜单界面
        }
    }
}




/****************主界面******************/
void Oled_Display_Main(void)
{
    //中文显示(横轴，纵轴，字符号)
    OLED_ShowCHinese(50,0,21);  //A徽标

    OLED_ShowString(19,2,"ACE");//ACE实验室
    OLED_ShowCHinese(19+26,2,14);
    OLED_ShowCHinese(19+26+18,2,15);
    OLED_ShowCHinese(19+26+36,2,16);

    OLED_ShowCHinese(18,4,17); //步兵机器人
    OLED_ShowCHinese(36,4,18);
    OLED_ShowCHinese(54,4,11);
    OLED_ShowCHinese(72,4,12);
    OLED_ShowCHinese(90,4,13);
    OLED_ShowCHinese(0,6,22); //R徽标
    OLED_ShowString(20,6, "   GUI   v1.0");
//		delay_ms(300);
}

/**************遥控状态监视界面*************/

void Oled_Display_Monitor_Remote(void)
{
    if(OLED_Data.oled_switch != 1)
    {
        OLED_Data.option_flag = 0;
        OLED_Data.last_option_flag = OLED_Data.option_flag;
        OLED_Data.oled_switch=1;
    }

    if((OLED_Data.last_option_flag != OLED_Data.option_flag)) //数据变化刷新
    {
        OLED_Clear();
    }

    //字符串显示
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

    //选项切换
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

/**************陀螺仪状态监视界面*************/

void Oled_Display_Monitor_Gyro(void)
{
    if(OLED_Data.oled_switch != 2)
    {
        OLED_Data.option_flag = 0;
        OLED_Data.last_option_flag = OLED_Data.option_flag;
        OLED_Data.oled_switch=2;
    }
    if((OLED_Data.last_option_flag != OLED_Data.option_flag)) //数据变化刷新
    {
        OLED_Clear();
    }

    //字符串显示
    OLED_ShowString(9,0, "Pitch:");
//    OLED_ShowNum(58,0, (int)MPU6050_Data.pitch_angle+90 ,3,18);  //option_flag

    OLED_ShowString(9,2, "Yaw:");
//    OLED_ShowNum(58,2, (int)MPU6050_Data.yaw_angle+90 ,3,18);  //option_flag

    OLED_ShowString(9,4, "Roll:");
//    OLED_ShowNum(58,4, (int)MPU6050_Data.roll_angle+90 ,3,18);  //option_flag

//	  OLED_ShowString(9,6, "Temp:");
//	  OLED_ShowNum(58,6, (int)MPU6050_Data.Temp ,3,18);  //option_flag

    //选项切换
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





/**************菜单界面(暂无，显示图片)*****************/
void Oled_Display_Menu(void)
{
    //BMP图片显示
    OLED_DrawBMP(0,0,127,7,BMP_Ugly_smile);

}





















