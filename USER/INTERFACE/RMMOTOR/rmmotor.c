#include "stm32f4xx.h"
#include "rmmotor.h"
 
/**
 *�ļ����ݣ� 1.���ֵ����PID�ṹ�嶨��͵����ʵֵ�Ķ���
 *          2.����ıջ��㷨���ٶȻ����ٶ�λ�û���       
 *          3.������ٵ������ֵ���㷨
 *          4.��������ٽ紦��
 *          5.���ٶ������㷨
 *          6.�����ת���
 *          7.������̨���̲���㷨
 *��ע���ṹ�帳ֵ��ʼ����system�ļ���
 *      PID����������PID_Def�ļ���
 */


/*==============================PID����==========================*/
/*****���̵��PID����******/
PID_t CHASSIS_1_S_PID;
PID_t CHASSIS_2_S_PID;
PID_t CHASSIS_3_S_PID;
PID_t CHASSIS_4_S_PID;

PID_t CHASSIS_1_P_PID;
PID_t CHASSIS_2_P_PID;
PID_t CHASSIS_3_P_PID;
PID_t CHASSIS_4_P_PID;

PID_t CHASSIS_MOVE_FOLLOW_PID; // �����ƶ�����PID
PID_t CHASSIS_ROTATE_FOLLOW_PID; // ������ת����PID

/******��̨���PID����******/
PID_t GIMBAL_P_PITCH_PID;
PID_t GIMBAL_S_PITCH_PID;
PID_t GIMBAL_P_YAW_PID;
PID_t GIMBAL_S_YAW_PID;

PID_t GIMBAL_INDEPENDENT_P_YAW_PID;
PID_t GIMBAL_INDEPENDENT_S_YAW_PID;

PID_t GIMBAL_AUTO_P_PITCH_PID;
PID_t GIMBAL_AUTO_S_PITCH_PID;
PID_t GIMBAL_AUTO_P_YAW_PID;
PID_t GIMBAL_AUTO_S_YAW_PID;

PID_t GIMBAL_AUTO_BUFF_P_PITCH_PID;
PID_t GIMBAL_AUTO_BUFF_S_PITCH_PID;
PID_t GIMBAL_AUTO_BUFF_P_YAW_PID;
PID_t GIMBAL_AUTO_BUFF_S_YAW_PID;

PID_t GIMBAL_DOUBLE_P_PITCH_PID;
PID_t GIMBAL_DOUBLE_S_PITCH_PID;
PID_t GIMBAL_DOUBLE_P_YAW_PID;
PID_t GIMBAL_DOUBLE_S_YAW_PID;

/******�������PID����********/
PID_t FIRE_S_PID;  

/*===========================�����ʵֵ===========================*/
Motor_Deal_Data Chassis_Motor[4]={0};      //�����ĸ����
Motor_Deal_Data Pitch_Motor_Data;          //P��������
Motor_Deal_Data Yaw_Motor_Data;            //Y��������
Motor_Deal_Data Fire_GD_Motor;             //�����������
Motor_Deal_Data Second_Yaw_Motor_Data;     //�����������



/*=========================����ջ��㷨==============================*/
/*
���ܣ��ٶȻ����ƣ�
����������ٶȻ�pid�ṹ��spid��Ŀ���ٶ�ֵsetSpeed��ʵ���ٶ�ֵactualSpeed
����������
*/
int16_t Rmmotor_Speed_control(PID_t *spid, int16_t setSpeed, int16_t actualSpeed, int16_t current_limit)
{
	int32_t output;
	
	spid->SetValue=setSpeed;
	spid->ActualVaule=actualSpeed;
	output=Location_Pid_Int32(spid,spid->ActualVaule);
	output=limit_int32(output,current_limit,-current_limit);
	
	return output;
}

/*
*���ܣ�λ���ٶȴ����ջ�����
*���룺����ٶ�pid�ṹ�壬���λ��pid�ṹ�壬�����ʵλ�ã������ʵ�ٶȣ�λ��Ŀ��ֵ����������ֵ
*����������
*/
int16_t Motor_Position_Speed_Control(PID_t *speed_pid, PID_t *position_pid, int16_t actual_position , int16_t actual_speed , int16_t setPosition, int16_t current_limit)
{
	int32_t output; 
  
	position_pid->SetValue = setPosition;                                           //λ�û��趨ֵΪ��������� �����ǽǶȴ��� (����ͷʼ����ǰ)
	speed_pid->SetValue = Location_Pid_Int32(position_pid , actual_position);       //�ٶȻ��趨ֵ��λ�û�����

	output = Location_Pid_Int32(speed_pid,actual_speed);            //��������    
	output = limit_int32(output,current_limit,-current_limit);      //�������      
	
	return output;
}



/*=========================RM���һ���㷨========================*/
/*��ʵ����ֵ����(ͨ�ã�������ʲô������������ֻҪ�漰����Ȧ����ʽ�Ķ�������)
*���룺������ݽṹ�壬���ٱȳ˴����ȣ���Ȧ����ֵ
*��������ʼֵΪ0�������Ҹ�����Ȧ���ɵ�����
*/
void Motor_Actual_Position(Motor_Deal_Data *rmMotor, int16_t gear_Ratio,int16_t lap_encoder)      
{
	if(rmMotor->first_Flag==0)//��һ�ν���ʱ��¼����ֵ 
	{
		rmMotor->last_Position = rmMotor->position;
		rmMotor->first_Flag=1;
	}
	rmMotor->actual_Position += Angle_Limiting_Int16(rmMotor->position - rmMotor->last_Position,lap_encoder);  //��ֵ�ۼ�
	rmMotor->actual_Position =  Check_CodeValue(rmMotor->actual_Position,gear_Ratio,lap_encoder);     //���ٽ�ֵ��λ����ֵ
	rmMotor->last_Position   =  rmMotor->position;  
}


//�ٽǴ���16λ����Ӧ�Ƕ���ֵ��
int16_t Angle_Limiting_Int16(int16_t Angl_Err, int16_t lap_encoder)   
{
	if(Angl_Err<-(lap_encoder/2))
	{
		Angl_Err+=(lap_encoder-1);
	}
	if(Angl_Err>(lap_encoder/2))
	{
		Angl_Err-=(lap_encoder-1);
	}
	return Angl_Err;
}


//�ٽǴ���32λ�����ٱȼ���
int32_t Angle_Limiting_Int32(int32_t Angl_Error,int16_t buff,int16_t lap_encoder)
{
	if(Angl_Error<-(buff*(lap_encoder/2)))
	{
		Angl_Error+=(buff*(lap_encoder-1));
	}
	if(Angl_Error>(buff*(lap_encoder/2)))
	{
		Angl_Error-=(buff*(lap_encoder-1));
	}
	return Angl_Error;
}


//���ٽ�ֵ��λ����ֵ ��������360�ȵ�����ֵѭ�� DJI�����
int32_t Check_CodeValue(int32_t value, int16_t gear_Ratio, int16_t lap_encoder) 
{

	if(value>(gear_Ratio*lap_encoder)/2)
	{
		value = value - (gear_Ratio*lap_encoder);
	}	
	if(value<(-(gear_Ratio*lap_encoder)/2))
	{
		value = (gear_Ratio*lap_encoder) - value;		
	}
	
	return value;
}




//�����ת���
static int16_t Block_Count=0;

int16_t Check_Motor_Block(int16_t position)  
{
	static int16_t last_Position;	
	if(abs_int16(last_Position-position)<10)
		Block_Count++;
	else
		Block_Count=0;
	last_Position=position;	
	if(Block_Count>100)
		return 1;
	else
		return 0;
}
 

//Y������������Ĳ�ǻ�ȡ(�Ե�������Ϊ0�ȣ�����Ϊ���ǣ�����Ϊ���ǣ���С��ΧΪ0~180��)
float Yaw_Different_Angle = 0;                      //��̨���̲��
int32_t Yaw_Middle_Code = (8192*YAW_RATIO)/2;  //Y�������̵İ�ֵ

float Get_Yaw_Different_Angle(Motor_Deal_Data *yaw_position , int16_t Ratio)
{

	Yaw_Different_Angle = (yaw_position->actual_Position*360)/(8192*Ratio);

	return Yaw_Different_Angle;
}



/*
*��Ȧ����ֵ����������ת��
*����ֵ����ʵ����ֵ
*/
int16_t rotate_50_cirecle=0;
int16_t last_code=0;

int16_t Encoder_Real(int32_t read_code)
{
//	int code = 0;
	int code_output = 0;
	
	if (read_code <= 49152)
	{
//		code = (read_code) / 3072;
//		code_output = read_code - code * 3072;//����Ȧ������ɵ�Ȧ��

		code_output = read_code%3072;
		return code_output;
	}
	else if (read_code > 49152)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}

/*
*���ܣ�Yaw����ʵλ�ã���Ȧ���������������Ҹ���
*���룺��ȡ������ʵ��Ȧ����ֵ����Ȧ�����������м�λ�õ�����ֵ
*���������м�λ��Ϊ��׼���ٽǴ���λ��ֵ�����Ȧ�����Ұ�Ȧ����
*/

int16_t Yaw_Actual_Code_Conversion(int16_t actual_code , int16_t max_code , int16_t middle_code)
{
	if(0<=actual_code && actual_code<=middle_code)  //������ֵ���м�Ϊ��׼
	{
		last_code = max_code-(middle_code-actual_code);
		if(last_code>=(max_code/2) && last_code<=max_code)  //�ٽǴ���
			last_code=last_code-max_code;
		else if(last_code>=0 && last_code<(max_code/2))
			last_code=last_code;
	}
	else if(middle_code<actual_code && actual_code<=3072)
	{
		last_code = actual_code-middle_code;
		if(last_code>=(max_code/2) && last_code<=max_code)  //�ٽǴ���
			last_code=last_code-max_code;
		else if(last_code>=0 && last_code<(max_code/2))
			last_code=last_code;
	}
	else last_code=-1;
	
	return last_code;
}

