#ifndef __MATHS_H
#define __MATHS_H

//�˶����ٶ�����б�º���
typedef __packed struct
{
	  float Input;       //��ǰȡ��ֵ
	  float Last_Input;  //�ϴ�ȡ��ֵ
	  float Output;      //���ֵ
	  float acc_now;     //��ǰ���ٶ�
	  float acc_limit;   //��Ҫ���Ƶļ��ٶ�
}acceleration_control_type_t;

//һ�׵�ͨ�˲�����
typedef __packed struct
{
    float Input;        //��ǰȡ��ֵ
	  float Last_Input;   //�ϴ�ȡ��ֵ
   	float Output;       //�˲����
    float Param;        //�˲�����
}first_order_low_filter_type_t;


//������ֵ�˲����������㣩
typedef __packed struct
{
    float Input;        //��ǰȡ��ֵ  
	  int count_num;      //ȡ������
   	float Output;       //�˲����
	  float Sum;          //�ۼ��ܺ�
	  float FIFO[250];    //����
	  int  sum_flag;      //�Ѿ���250����־
}sliding_mean_filter_type_t;


//������ֵ�˲�����(16λ)
typedef __packed struct
{
    int16_t Input;        //��ǰȡ��ֵ
	  int16_t count_num;      //ȡ������
   	int16_t Output;       //�˲����
	  int16_t Sum;          //�ۼ��ܺ�
	  int16_t FIFO[250];    //����
	  int16_t  sum_flag;      //�Ѿ���250����־
}sliding_mean_filter_Int16_type_t;

 
 

/*��������*/
int32_t limit_int32(int32_t x,int32_t max,int32_t min);
int16_t limit_int16(int16_t x,int16_t max,int16_t min); 
float limit_float(float x,float max,float min);

int16_t abs_int16(int16_t x);
float abs_float(float x);

int16_t max_abs(int16_t x,int16_t y);		


int16_t Motion_acceleration_control(acceleration_control_type_t *acceleration_control , int16_t Input, int16_t Limit); //�˶����ٶ�����
float First_Order_Low_Filter(first_order_low_filter_type_t *first_order_low_filter_type, float Input);  //һ�׵�ͨ�˲�
float Sliding_Mean_Filter(sliding_mean_filter_type_t *mean_filter, float Input , int num);              //��ֵ�����˲�
void Sliding_Mean_Filter_Init(sliding_mean_filter_type_t *mean_filter);                                 //��ֵ�����˲���ʼ�����ɲ��ã�ֱ�Ӷ���ṹ��ʱ����ֵ��
void Sliding_Mean_Filter_Int16_Init(sliding_mean_filter_Int16_type_t *mean_filter);                     //16λ��ֵ������ʼ��
int16_t Sliding_Mean_Filter_Int16(sliding_mean_filter_Int16_type_t *mean_filter, int16_t Input);        //16λ��ֵ��������
int16_t Loop_Restriction_Int16(int16_t num, int16_t limit_num);                                         //16λѭ���޷�
float Loop_Restriction_Float(float num, float limit_num);                                               //����ѭ���޷�
float loop_fp32_constrain(float Input, float minValue, float maxValue);                                 //ѭ�����ƣ���̨�Ƕȴ���
void Data_Accelerated_Control(float *input , float acc);	                                              //���ٶ�����б�º���



#endif
