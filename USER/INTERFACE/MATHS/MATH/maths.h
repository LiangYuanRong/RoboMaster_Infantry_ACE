#ifndef __MATHS_H
#define __MATHS_H

//运动加速度限制斜坡函数
typedef __packed struct
{
	  float Input;       //当前取样值
	  float Last_Input;  //上次取样值
	  float Output;      //输出值
	  float acc_now;     //当前加速度
	  float acc_limit;   //需要限制的加速度
}acceleration_control_type_t;

//一阶低通滤波参数
typedef __packed struct
{
    float Input;        //当前取样值
	  float Last_Input;   //上次取样值
   	float Output;       //滤波输出
    float Param;        //滤波参数
}first_order_low_filter_type_t;


//滑动均值滤波参数（浮点）
typedef __packed struct
{
    float Input;        //当前取样值  
	  int count_num;      //取样次数
   	float Output;       //滤波输出
	  float Sum;          //累计总和
	  float FIFO[250];    //队列
	  int  sum_flag;      //已经够250个标志
}sliding_mean_filter_type_t;


//滑动均值滤波参数(16位)
typedef __packed struct
{
    int16_t Input;        //当前取样值
	  int16_t count_num;      //取样次数
   	int16_t Output;       //滤波输出
	  int16_t Sum;          //累计总和
	  int16_t FIFO[250];    //队列
	  int16_t  sum_flag;      //已经够250个标志
}sliding_mean_filter_Int16_type_t;

 
 

/*函数声明*/
int32_t limit_int32(int32_t x,int32_t max,int32_t min);
int16_t limit_int16(int16_t x,int16_t max,int16_t min); 
float limit_float(float x,float max,float min);

int16_t abs_int16(int16_t x);
float abs_float(float x);

int16_t max_abs(int16_t x,int16_t y);		


int16_t Motion_acceleration_control(acceleration_control_type_t *acceleration_control , int16_t Input, int16_t Limit); //运动加速度限制
float First_Order_Low_Filter(first_order_low_filter_type_t *first_order_low_filter_type, float Input);  //一阶低通滤波
float Sliding_Mean_Filter(sliding_mean_filter_type_t *mean_filter, float Input , int num);              //均值滑窗滤波
void Sliding_Mean_Filter_Init(sliding_mean_filter_type_t *mean_filter);                                 //均值滑窗滤波初始化（可不用，直接定义结构体时给初值）
void Sliding_Mean_Filter_Int16_Init(sliding_mean_filter_Int16_type_t *mean_filter);                     //16位均值滑窗初始化
int16_t Sliding_Mean_Filter_Int16(sliding_mean_filter_Int16_type_t *mean_filter, int16_t Input);        //16位均值滑窗处理
int16_t Loop_Restriction_Int16(int16_t num, int16_t limit_num);                                         //16位循环限幅
float Loop_Restriction_Float(float num, float limit_num);                                               //浮点循环限幅
float loop_fp32_constrain(float Input, float minValue, float maxValue);                                 //循环限制（云台角度处理）
void Data_Accelerated_Control(float *input , float acc);	                                              //加速度限制斜坡函数



#endif
