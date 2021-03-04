/***************************************************************************************************************************
�ļ��� ���㷨����

���   �����ɸ��ֻ�����ֵ������˶������㷨

�������ܣ� 
			 1.��ֵ����
       2.ȡ����ֵ
       3.ȡ���ֵ
			 4.�˶�����б�º��������ٶ����ƣ���16λ��
			 5.һ�׵�ͨ�˲�
			 6.��ֵ�����˲���16λ & ���㣩
			 7.���̷�ֹ�˶��켣ʧ��
			 8.ѭ�����ƺ���
****************************************************************************************************************************/
#include "stm32f4xx.h"
#include "maths.h"

/*��ֵ����*/
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

/*����ֵ����*/
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


/*ȡ���������ֵ�ľ���ֵ*/
int16_t max_abs(int16_t x,int16_t y)		
{
	if(abs_int16(x) >= abs_int16(y))
		return abs_int16(x);
	else
		return abs_int16(y);
}


/*
*���ܣ��˶�����б�º��������ٶ����ƣ�
*���룺1.���ٶ����ƶ�Ӧ�ṹ��  2.������ 3.���Ƽ��ٶ�
*������������
*���ͣ�16λ����
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
*���ܣ�һ�׵�ͨ�˲����㣨�����ͣ�
*���룺1.��ͨ�˲��ṹ��  2.��Ҫ�˲�����
*������һ�׵�ͨ�˲�������
*��ע��Y(n)=a*X(n)+(1-a)*Y(n-1)   //Y(n):�����˲�ֵ  a���˲�ϵ��  X(n):���β���ֵ   Y(n-1)���ϴ��˲�ֵ
*                                 //�˲�ϵ��ԽС���˲����Խƽ�ȣ�����������Խ�ͣ�
*                                 //�˲�ϵ��Խ��������Խ�ߣ������˲����Խ���ȶ���
*
*                                 //�����ݿ��ٱ仯ʱ���˲�����ܼ�ʱ���������������ȣ���
*                                 //�����������ȶ�����һ���̶��ĵ�������ʱ���˲����������ƽ�ȣ�ƽ�ȶ����ȣ���
*/ 
float First_Order_Low_Filter(first_order_low_filter_type_t *first_order_low_filter_type, float Input)
{
	first_order_low_filter_type->Input = Input;
	first_order_low_filter_type->Output = (first_order_low_filter_type->Input * first_order_low_filter_type->Param) + ((1-first_order_low_filter_type->Param) * first_order_low_filter_type->Last_Input) ;
	first_order_low_filter_type->Last_Input = first_order_low_filter_type->Output ; 
  
	return first_order_low_filter_type->Output;
}




/*
*���ܣ�������ֵ�˲�������ʼ��(������)
*���룺�˲�����ṹ��
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
*���ܣ�������ֵ�˲��������ͣ�------����С���ȸ�Ƶ����
*���룺1.�˲�����ṹ��  2.����ֵ 3.��ֵ����
*�����������˲����ֵ��250�Σ�
*/
float Sliding_Mean_Filter(sliding_mean_filter_type_t *mean_filter, float Input ,int num)
{
	//����
	mean_filter->Input = Input;                                         
	mean_filter->FIFO[mean_filter->count_num] = mean_filter->Input;   
	mean_filter->count_num++;
	
	if(mean_filter->count_num==num)
	{
		mean_filter->count_num=0;
	  mean_filter->sum_flag = 1;
	}
	//���
	if(mean_filter->sum_flag == 1)
	{ 
		for(int count=0;count<num;count++)
		{
			mean_filter->Sum += mean_filter->FIFO[count];
		}
	}
	//��ֵ
	mean_filter->Output = mean_filter->Sum/num;
	mean_filter->Sum = 0;
	
	return mean_filter->Output;
}


/*
*���ܣ�������ֵ�˲�������ʼ��(16λ)
*���룺�˲�����ṹ��
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
*���ܣ�������ֵ�˲���16λ��------����С���ȸ�Ƶ����
*���룺1.�˲�����ṹ��  2.����ֵ
*�����������˲����ֵ��10�Σ�
*/ 
int16_t Sliding_Mean_Filter_Int16(sliding_mean_filter_Int16_type_t *mean_filter, int16_t Input)
{
	//����
	mean_filter->Input = Input;                                         
	mean_filter->FIFO[mean_filter->count_num] = mean_filter->Input;   
	mean_filter->count_num++;
	
	if(mean_filter->count_num==100)
	{
		mean_filter->count_num=0;
	  mean_filter->sum_flag = 1;
	}
	//���
	if(mean_filter->sum_flag == 1)
	{ 
		for(int count=0;count<100;count++)
		{
			mean_filter->Sum += mean_filter->FIFO[count];
		}
	}
	//��ֵ
	mean_filter->Output = mean_filter->Sum/10;
	mean_filter->Sum = 0;
	
	return mean_filter->Output;
}


/*
*���ܣ�����ѭ�����ƣ�16λ��
*���룺1.����ֵ  2.���Ʒ���(����)  
*�������޷����ֵ
*������������ֵ������ +-���Ʒ��� �ķ�Χ��
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
*���ܣ�����ѭ�����ƣ�float��
*���룺1.����ֵ  2.���Ʒ���(����)  
*�������޷����ֵ
*������������ֵ������ +-���Ʒ��� �ķ�Χ��
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

/*ѭ���޷�32*/
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

 

//б�º���(���ٶ�����)
void Data_Accelerated_Control(float *input , float acc)				
{
	static int16_t last_num = 0;
	int16_t temp;
	temp = *input - last_num;

	if(abs_float(temp) > acc)
			*input = last_num + temp/abs_float(temp)*acc;
	
	last_num = *input;	
}
