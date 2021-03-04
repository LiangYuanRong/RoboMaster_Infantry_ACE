#include "stm32f4xx.h"
#include "rmmotor.h"
 
/**
 *文件内容： 1.各种电机的PID结构体定义和电机真实值的定义
 *          2.电机的闭环算法（速度环，速度位置环）       
 *          3.计算减速电机码盘值的算法
 *          4.电机码盘临界处理
 *          5.加速度限制算法
 *          6.电机堵转检测
 *          7.计算云台底盘差角算法
 *备注：结构体赋值初始化在system文件里
 *      PID参数定义在PID_Def文件里
 */


/*==============================PID参数==========================*/
/*****底盘电机PID参数******/
PID_t CHASSIS_1_S_PID;
PID_t CHASSIS_2_S_PID;
PID_t CHASSIS_3_S_PID;
PID_t CHASSIS_4_S_PID;

PID_t CHASSIS_1_P_PID;
PID_t CHASSIS_2_P_PID;
PID_t CHASSIS_3_P_PID;
PID_t CHASSIS_4_P_PID;

PID_t CHASSIS_MOVE_FOLLOW_PID; // 底盘移动跟随PID
PID_t CHASSIS_ROTATE_FOLLOW_PID; // 底盘旋转跟随PID

/******云台电机PID参数******/
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

/******供弹电机PID参数********/
PID_t FIRE_S_PID;  

/*===========================电机真实值===========================*/
Motor_Deal_Data Chassis_Motor[4]={0};      //底盘四个电机
Motor_Deal_Data Pitch_Motor_Data;          //P轴电机数据
Motor_Deal_Data Yaw_Motor_Data;            //Y轴电机数据
Motor_Deal_Data Fire_GD_Motor;             //供弹电机数据
Motor_Deal_Data Second_Yaw_Motor_Data;     //供弹电机数据



/*=========================电机闭环算法==============================*/
/*
功能：速度环控制；
传入参数：速度环pid结构体spid，目标速度值setSpeed，实际速度值actualSpeed
传出：电流
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
*功能：位置速度串级闭环控制
*传入：电机速度pid结构体，电机位置pid结构体，电机真实位置，电机真实速度，位置目标值，电流限制值
*传出：电流
*/
int16_t Motor_Position_Speed_Control(PID_t *speed_pid, PID_t *position_pid, int16_t actual_position , int16_t actual_speed , int16_t setPosition, int16_t current_limit)
{
	int32_t output; 
  
	position_pid->SetValue = setPosition;                                           //位置环设定值为输入控制量 陀螺仪角度处理 (保持头始终向前)
	speed_pid->SetValue = Location_Pid_Int32(position_pid , actual_position);       //速度环设定值由位置环处理

	output = Location_Pid_Int32(speed_pid,actual_speed);            //电机输出量    
	output = limit_int32(output,current_limit,-current_limit);      //输出限制      
	
	return output;
}



/*=========================RM电机一般算法========================*/
/*真实码盘值计算(通用，无论是什么电机或编码器，只要涉及到单圈增量式的都可以用)
*传入：电机数据结构体，减速比乘传动比，单圈码盘值
*传出：初始值为0，左正右负，半圈过渡的数据
*/
void Motor_Actual_Position(Motor_Deal_Data *rmMotor, int16_t gear_Ratio,int16_t lap_encoder)      
{
	if(rmMotor->first_Flag==0)//第一次进入时记录码盘值 
	{
		rmMotor->last_Position = rmMotor->position;
		rmMotor->first_Flag=1;
	}
	rmMotor->actual_Position += Angle_Limiting_Int16(rmMotor->position - rmMotor->last_Position,lap_encoder);  //差值累加
	rmMotor->actual_Position =  Check_CodeValue(rmMotor->actual_Position,gear_Ratio,lap_encoder);     //过临界值复位码盘值
	rmMotor->last_Position   =  rmMotor->position;  
}


//临角处理16位（对应角度正值）
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


//临角处理32位带减速比计算
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


//过临界值复位码盘值 （限制于360度的码盘值循环 DJI电机）
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




//电机堵转检测
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
 

//Y轴电机与底盘中心差角获取(以底盘中心为0度，向右为正角，向左为负角，大小范围为0~180度)
float Yaw_Different_Angle = 0;                      //云台底盘差角
int32_t Yaw_Middle_Code = (8192*YAW_RATIO)/2;  //Y轴总码盘的半值

float Get_Yaw_Different_Angle(Motor_Deal_Data *yaw_position , int16_t Ratio)
{

	Yaw_Different_Angle = (yaw_position->actual_Position*360)/(8192*Ratio);

	return Yaw_Different_Angle;
}



/*
*多圈绝对值编码器数据转换
*传入值：真实码盘值
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
//		code_output = read_code - code * 3072;//将总圈数换算成单圈数

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
*功能：Yaw轴真实位置（多圈编码器）（左正右负）
*传入：读取到的真实单圈码盘值，单圈码盘总数，中间位置的码盘值
*传出：以中间位置为标准的临角处理位置值（左半圈正，右半圈负）
*/

int16_t Yaw_Actual_Code_Conversion(int16_t actual_code , int16_t max_code , int16_t middle_code)
{
	if(0<=actual_code && actual_code<=middle_code)  //将码盘值以中间为标准
	{
		last_code = max_code-(middle_code-actual_code);
		if(last_code>=(max_code/2) && last_code<=max_code)  //临角处理
			last_code=last_code-max_code;
		else if(last_code>=0 && last_code<(max_code/2))
			last_code=last_code;
	}
	else if(middle_code<actual_code && actual_code<=3072)
	{
		last_code = actual_code-middle_code;
		if(last_code>=(max_code/2) && last_code<=max_code)  //临角处理
			last_code=last_code-max_code;
		else if(last_code>=0 && last_code<(max_code/2))
			last_code=last_code;
	}
	else last_code=-1;
	
	return last_code;
}

