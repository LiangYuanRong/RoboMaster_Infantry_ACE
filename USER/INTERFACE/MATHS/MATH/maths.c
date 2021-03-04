/***************************************************************************************************************************
文件名 ：算法汇总

简介   ：集成各种基本数值处理和运动处理算法

函数功能： 
			 1.数值限制
       2.取绝对值
       3.取最大值
			 4.运动控制斜坡函数（加速度限制）（16位）
			 5.一阶低通滤波
			 6.均值滑窗滤波（16位 & 浮点）
			 7.底盘防止运动轨迹失真
			 8.循环限制函数
****************************************************************************************************************************/
#include "stm32f4xx.h"
#include "maths.h"

/*数值限制*/
int32_t limit_int32(int32_t x,int32_t max,int32_t min)
{
	if(x>max)
		x=max;
	else if(x<min)
		x=min;
	return x;
}

int16_t limit_int16(int16_t x,int16_t max,int16_t min)    
{
	if(x>max)
		x=max;
	else if(x<min)
		x=min;
	return x;
}

float limit_float(float x,float max,float min)
{
	if(x>max)
		x=max;
	else if(x<min)
		x=min;
	return x;
}

/*绝对值计算*/
int16_t abs_int16(int16_t x)
{
	if(x<0) return(-x);
	else    return x;
}

float abs_float(float x)
{
	if(x<0) return(-x);
	else    return x;	
}


/*取两个数最大值的绝对值*/
int16_t max_abs(int16_t x,int16_t y)		
{
	if(abs_int16(x) >= abs_int16(y))
		return abs_int16(x);
	else
		return abs_int16(y);
}


/*
*功能：运动控制斜坡函数（加速度限制）
*传入：1.加速度限制对应结构体  2.控制量 3.限制加速度
*传出：处理量
*类型：16位整形
*/  
int16_t Motion_acceleration_control(acceleration_control_type_t *acceleration_control , int16_t Input, int16_t Limit)
{
	acceleration_control->Input = Input;
	acceleration_control->acc_limit = Limit;
	
	acceleration_control->acc_now = acceleration_control->Input - acceleration_control->Last_Input;
	
	if(abs_int16(acceleration_control->acc_now) > acceleration_control->acc_limit)
	{
		acceleration_control->Output = acceleration_control->Last_Input + acceleration_control->acc_now/abs_int16(acceleration_control->acc_now)*acceleration_control->acc_limit;
	}
	
	acceleration_control->Last_Input = acceleration_control->Output;
	 
	return acceleration_control->Output;
}


/*
*功能：一阶低通滤波计算（浮点型）
*传入：1.低通滤波结构体  2.需要滤波数据
*传出：一阶低通滤波后数据
*备注：Y(n)=a*X(n)+(1-a)*Y(n-1)   //Y(n):本次滤波值  a：滤波系数  X(n):本次采样值   Y(n-1)：上次滤波值
*                                 //滤波系数越小，滤波结果越平稳，但是灵敏度越低；
*                                 //滤波系数越大，灵敏度越高，但是滤波结果越不稳定。
*
*                                 //当数据快速变化时，滤波结果能及时跟进（灵敏度优先）；
*                                 //当数据趋于稳定，在一个固定的点上下振荡时，滤波结果能趋于平稳（平稳度优先）。
*/ 
float First_Order_Low_Filter(first_order_low_filter_type_t *first_order_low_filter_type, float Input)
{
	first_order_low_filter_type->Input = Input;
	first_order_low_filter_type->Output = (first_order_low_filter_type->Input * first_order_low_filter_type->Param) + ((1-first_order_low_filter_type->Param) * first_order_low_filter_type->Last_Input) ;
	first_order_low_filter_type->Last_Input = first_order_low_filter_type->Output ; 
  
	return first_order_low_filter_type->Output;
}




/*
*功能：滑动均值滤波参数初始化(浮点型)
*输入：滤波对象结构体
*/
void Sliding_Mean_Filter_Init(sliding_mean_filter_type_t *mean_filter)
{
	mean_filter->count_num=0;
	for(int i=0;i<20;i++)
	  mean_filter->FIFO[i]=0.0f;
	mean_filter->Input = 0.0f;
	mean_filter->Output = 0.0f;
	mean_filter->Sum = 0.0f;
	mean_filter->sum_flag = 0;
}	

/*
*功能：滑动均值滤波（浮点型）------抑制小幅度高频噪声
*传入：1.滤波对象结构体  2.更新值 3.均值数量
*传出：滑动滤波输出值（250次）
*/
float Sliding_Mean_Filter(sliding_mean_filter_type_t *mean_filter, float Input ,int num)
{
	//更新
	mean_filter->Input = Input;                                         
	mean_filter->FIFO[mean_filter->count_num] = mean_filter->Input;   
	mean_filter->count_num++;
	
	if(mean_filter->count_num==num)
	{
		mean_filter->count_num=0;
	  mean_filter->sum_flag = 1;
	}
	//求和
	if(mean_filter->sum_flag == 1)
	{ 
		for(int count=0;count<num;count++)
		{
			mean_filter->Sum += mean_filter->FIFO[count];
		}
	}
	//均值
	mean_filter->Output = mean_filter->Sum/num;
	mean_filter->Sum = 0;
	
	return mean_filter->Output;
}


/*
*功能：滑动均值滤波参数初始化(16位)
*输入：滤波对象结构体
*/
void Sliding_Mean_Filter_Int16_Init(sliding_mean_filter_Int16_type_t *mean_filter)
{
	mean_filter->count_num=0;
	for(int i=0;i<250;i++)
	  mean_filter->FIFO[i]=0;
	mean_filter->Input = 0;
	mean_filter->Output = 0;
	mean_filter->Sum = 0;
	mean_filter->sum_flag = 0;
}	

/*
*功能：滑动均值滤波（16位）------抑制小幅度高频噪声
*传入：1.滤波对象结构体  2.更新值
*传出：滑动滤波输出值（10次）
*/ 
int16_t Sliding_Mean_Filter_Int16(sliding_mean_filter_Int16_type_t *mean_filter, int16_t Input)
{
	//更新
	mean_filter->Input = Input;                                         
	mean_filter->FIFO[mean_filter->count_num] = mean_filter->Input;   
	mean_filter->count_num++;
	
	if(mean_filter->count_num==100)
	{
		mean_filter->count_num=0;
	  mean_filter->sum_flag = 1;
	}
	//求和
	if(mean_filter->sum_flag == 1)
	{ 
		for(int count=0;count<100;count++)
		{
			mean_filter->Sum += mean_filter->FIFO[count];
		}
	}
	//均值
	mean_filter->Output = mean_filter->Sum/10;
	mean_filter->Sum = 0;
	
	return mean_filter->Output;
}


/*
*功能：正负循环限制（16位）
*传入：1.输入值  2.限制幅度(正数)  
*传出：限幅输出值
*描述：将输入值限制在 +-限制幅度 的范围内
*/ 
int16_t Loop_Restriction_Int16(int16_t num, int16_t limit_num)
{
	if(abs_int16(num)>limit_num)
	{
		if(num>=0)num-=limit_num;
		else num+=limit_num;
	}	
}

/*
*功能：正负循环限制（float）
*传入：1.输入值  2.限制幅度(正数)  
*传出：限幅输出值
*描述：将输入值限制在 +-限制幅度 的范围内
*/ 
float Loop_Restriction_Float(float num, float limit_num)
{
	if(abs_float(num)>limit_num)
	{
		if(num>=0)num-=limit_num;
		else num+=limit_num;
	}	
	return num;
}

/*循环限幅32*/
float loop_fp32_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

 

//斜坡函数(加速度限制)
void Data_Accelerated_Control(float *input , float acc)				
{
	static int16_t last_num = 0;
	int16_t temp;
	temp = *input - last_num;

	if(abs_float(temp) > acc)
			*input = last_num + temp/abs_float(temp)*acc;
	
	last_num = *input;	
}
