/*==================================================*
*�ļ����ܣ�1.��̨��������
*         2.������̨ģʽ�Ŀ����㷨
*             
*��ע��ԓ�ļ���û���븱��̨����
*==================================================*/

//===============ͷ�ļ�
#include "main.h"
#include "Gimbal_Task.h"
#include "Chassis_Task.h"
#include "Remote_Task.h"
#include "Fire_Task.h"
#include "Speed_Def.h"
#include "PID_Def.h"
#include "Mode_Def.h"
#include "Gyro_Task.h"
#include "kalman_filter.h"

//================��������
int gimbal_heart=0;                     //��̨��������
int GimbalTask_water=0;                 //��ջʣ��

int8_t INITIALIZE_flag=0;               //���͸����̵ĳ�ʼ���ɹ���־
int8_t YAW_ZERO_FLAG = 0;               //yaw����ֵ����־
int8_t SEC_YAW_ZERO_FLAG = 0;           //��yaw����ֵ����־

int8_t gimbal_work_mode=0;              //��̨״̬�л������������־λ
int16_t gimbal_output[4] = {0,0,0,0};   //��̨����������    0:P����   1��Y����   2:��yaw����   3����pitch����

int16_t yaw_different_angle=0;          //��̨���̲��
float Gimbal_yaw_angle=0.0f;            //��̨��ǰ�Ƕ�
static float Last_Gimbal_yaw_angle=0.0f;//��̨����Ƕ�
int8_t GIMBAL_SUPPLY_FLAG = 0;          //����״̬��־λ(0:����״̬�����벹��״̬  1:��̨90��ת����  2:����ָ��λ��  3:������   4������ģʽ����)

int16_t second_yaw=0;
extern float Yaw_different_angle;       //����̨��Ե��̵Ĳ��
float  Sec_Yaw_different_angle=0.0f;    //����̨�������̨�Ĳ��
int16_t  Sec_chassis_angle=0;          //����̨��Ե��̵Ĳ��

//================�ṹ�嶨��
//extKalman gimbal_buff_yaw;             //����һ�����yaw�Ῠ�����ṹ��            

//================�ڲ���������
  //��̨����
static void Gimbal_Control(void);       //��̨״̬����          
  //��̨��Ϊ
static void Gimbal_Work(void);          //��̨��������ģʽ
static void Gimbal_Init(void);          //��̨��ʼ��
static  void Yaw_Init(void);            //Yaw���ʼ��
static void Sec_Yaw_Init(void);         //��Yaw���ʼ��
static	void Pitch_Init(void);          //Pitch���ʼ��
static void Gimbal_Auto(void);          //��̨����ģʽ
static void Gimbal_Buff(void);          //��̨���ģʽ
static void Gimbal_Independent(void);   //��̨����ģʽ
static void Gimbal_Supply(void);        //��̨����ģʽ
static void Gimbal_Stop(void);          //��̨����
 //�����㷨
static void Gimbal_Angle_Limit(void);   //��̨�Ƕ�����
static void Correct_Yaw_Zero(void);     //yaw���У��
static void Correct_Sec_Yaw_Zero(void); //��yaw���У��
static void Change_camera(int camera_num);//ʵʱ�ο�С�������ݸı��������






/**====��̨����====**/
void Gimbal_Task(void *pvParameters)
{
  vTaskDelay(GIMBAL_TASK_INIT_TIME);	
	
	BMI160_Zero_Correct();            //����У׼�źŸ�������ģ��
//	BMI160_AngleZ_Zero();           //������ģ��yaw�����	
	MiniPC_Kalman_Data_Init();        //�Ӿ����ݿ�����������ʼ��
	
	while(1)
	{
		gimbal_heart = !gimbal_heart;    //��̨��������
		LEDE2 = gimbal_heart;

    Gimbal_Control();                //��̨����
		
#ifdef chassis_exclusive_use
		INITIALIZE_flag = 1;             //���̵���ʹ��ʱ����ʼ����־Ϊ1
#endif		
		//ϵͳ��ʱ
    vTaskDelay(GIMBAL_CONTROL_TIME_MS);
	}
}



/**====��̨״̬����====**/
static void Gimbal_Control(void)
{
  Correct_Yaw_Zero();                        //yaw����������
//  Correct_Sec_Yaw_Zero();                    //��yaw����������

#ifdef board_gimbal
//==============Ԥ����============
	YAW_ZERO_FLAG = Yaw_Zero_Value();         //ʵʱ���YawУ׼���  (����ֵ����1)
	SEC_YAW_ZERO_FLAG = Sec_Yaw_Zero_Value(); //ʵʱ��⸱YawУ׼���(����ֵ����1)	
	MiniPC_Data_Deal();                       //С���Դ������ݴ���
	CAN2_Send_Yaw_Data_To_Chassis(INITIALIZE_flag , gimbal_output[1] , YAW_ZERO_FLAG , GIMBAL_SUPPLY_FLAG , Sec_chassis_angle);          //��̨���ݷ��͸�����
	BMI_Data_Deal();                          //��������� 
	Sec_Yaw_different_angle = Get_Yaw_Different_Angle(&Second_Yaw_Motor_Data , 1*3);                             //���̽��������̨������̨֮��Ĳ�� 
	Sec_chassis_angle       = loop_fp32_constrain((Yaw_different_angle - Sec_Yaw_different_angle) , -180 , 180); //���̽��������̨�����֮��Ĳ��
	if(workStatus != INITIALIZE)
	{Gyro_usart_iwdg();}                      //��������ι����

	
//==============״̬�л�===========
	if(workStatus == INITIALIZE)
	{ 
		gimbal_work_mode=0;
		Gimbal_Init();         //��̨��ʼ��
	}
	else if(workStatus == AUTOATTACK)
	{
		MiniPC_Send_Data(1 , 0 , 12); //���͵�С���ԣ�  0����ɫ ��1����ɫ  ; 0�����飬1�����    ��  ��ǰ���������٣�		
		Gimbal_Auto();                //��̨�Զ����
	}
	else if(workStatus == AUTOBUFF)  
	{ 
		MiniPC_Send_Data(1 , 1 , 12); //���͵�С���ԣ�  0����ɫ ��1����ɫ  ; 0�����飬1�����    ��  ��ǰ���������٣�				
		Gimbal_Buff();                //��̨��糵ģʽ
	}
	else if(workStatus == REPLENISHMEN)
	{
		gimbal_work_mode=0;		
		Gimbal_Supply();              //��̨����ģʽ
	}
	else if(workStatus == WORKING)
	{
		if(chassis_workStatus==TWIST_WAIST||chassis_workStatus==ROTATION/*INDEPENDENT*/)//Ť��������ʱ����һ�ײ���
		{
			Gimbal_Work();              //��̨��������			
//			Gimbal_Independent();
		}
		else
		{			
			Gimbal_Work();              //��̨��������
		}
	}
	else
	{
		gimbal_work_mode=0;		
		Gimbal_Stop();                //ֹͣ
	}
#endif

	
/*�����̨���*/
	
#ifdef gimbal_work
	
  #ifdef board_gimbal      //ֻ����̨���������̨����ʱ���ܿ���PY��
	    #ifdef double_gimbal
   	   CAN1_SendCommand_Gimbal_Pitch(gimbal_output[0],gimbal_output[2]);  //��̨��can1����P��͸���̨yaw	    
      #else	     
   	   CAN1_SendCommand_Gimbal_Pitch(gimbal_output[0],0);                 //��̨��can1����P��͸���̨yaw	 
      #endif	
	#endif 

	#ifdef board_chassis
	   #ifdef fire_work
     CAN1_SendCommand_Gimbal_Fire(gimbal_output[1],0,Fire_param.GD_output);//���̰�can1����Y�ṩ��	
	   #else
     CAN1_SendCommand_Gimbal_Fire(gimbal_output[1],0,0);        //���̰�can1����Y�ṩ��	
	   #endif
	#endif
	
#else	   
     gimbal_output[1]=0;
	   CAN1_SendCommand_Gimbal_Fire(0,0,0);
#endif
	
}	





/*===================================��̨��ʼ��==============================*/
int16_t yaw_init_success=0;           //Y���ʼ���ɹ���־
static int16_t sec_yaw_init_success=0;//��Y���ʼ���ɹ���־
static int16_t pitch_init_success=0;  //P���ʼ���ɹ���־
static int16_t pitch_init_state=0;    //P���ʼ��״̬

//��̨��ʼ����
static void Gimbal_Init(void)
{		
	//ֱ�ӳ�ʼ��������Ҫ�ӣ�����ֱ��ȥע�ͼ���
		yaw_init_success   = 1;
		pitch_init_success = 1;
		sec_yaw_init_success = 1;
		
		
		Yaw_Init();    //Y���ʼ��
		Sec_Yaw_Init();//��Y���ʼ��
		Pitch_Init();  //P���ʼ��
	
	if(yaw_init_success==1 && pitch_init_success==1 && sec_yaw_init_success==1)
	{
		yaw_init_success     = 0;
		sec_yaw_init_success = 0;
		pitch_init_success   = 0;
		pitch_init_state     = 0;
		INITIALIZE_flag      = 1;
		workStatus           = WORKING;   //��ʼ����
	}
}

//Yaw���ʼ��
static void Yaw_Init(void)
{ 
	if(yaw_init_success == 0)  //Y���ʼ��δ�ɹ�
	{
    gimbal_output[1] = Rmmotor_Speed_control(&GIMBAL_S_YAW_PID, 900, Yaw_Motor_Data.speed, 1500);		//�ٶȻ���900Ŀ��ֵ	
		if(YAW_ZERO_FLAG == 1)  //���̽ӽ���ֵ
		{
			 yaw_init_success = 1;  		
		}
	} 	
  else
	{
		gimbal_output[1] = 0;
	}		
}

//Sec_Yaw���ʼ��
static void Sec_Yaw_Init(void)
{ 
	if(sec_yaw_init_success == 0)  //��Y���ʼ��δ�ɹ�
	{
    gimbal_output[2] = Rmmotor_Speed_control(&GIMBAL_DOUBLE_S_YAW_PID, 850, Second_Yaw_Motor_Data.speed, 1000);	//�ٶȻ���850Ŀ��ֵ		
		if(SEC_YAW_ZERO_FLAG==1)  //���̽ӽ���ֵ
		{
			 sec_yaw_init_success = 1;  		
		}
	}
  else
	{
		gimbal_output[2] = 0; 
	}		
}
 	

//Pitch���ʼ��  
static int16_t pitch_init_error = 0;   //P���ʼ����ʧ�ܴ���
static int16_t pitch_init_count = 0;   //P���ʼ����̨������������

static void Pitch_Init(void)
{
	if(pitch_init_success == 0) //P��δ��ʼ�����
	{
			if(pitch_init_state==0)   //P�Ὣ����̨���뿨��
			{
					gimbal_output[0]=-5600; //�ٶȿ���
					if(Pitch_Motor_Data.actual_Position>=100) //����̨�ѿ��뿨��
					{
						pitch_init_state=1;
					}
			}
			
//			pitch_init_state=1;    //ֱ����ͨ���У����ø���̨p��

			if(pitch_init_state==1)//P�����
			{
				//λ�û�Ŀ��ֵΪ0
				 gimbal_output[0] = Motor_Position_Speed_Control(&GIMBAL_S_PITCH_PID , &GIMBAL_P_PITCH_PID , 0 , Pitch_Motor_Data.speed , (Pitch_Motor_Data.actual_Position*360/1024) , PITCH_OUTPUT_LIMIT); 
				
				if(abs(Pitch_Middle_Angle - Pitch_Motor_Data.position) < 20)  //���̽ӽ���ֵ
				{
					if(pitch_init_count >= 50) //��ʼ������
					 { 
						 pitch_init_success = 1;  
						 pitch_init_count   = 0; //��λ
						 pitch_init_error   = 0;
						 pitch_init_state   = 0;			
					 }
					 else
					 {
						 pitch_init_count++;     //�ɹ������ۼ�
					 }
				}
				else
				{
					pitch_init_error++;        //��ʼ��ʧ�ܼ�¼
				}
			 }
		
			if(pitch_init_error == 1500) //��γ�ʼ��ʧ�ܣ�ǿ�Ƴ�ʼ���ɹ�
			{
				 pitch_init_success = 1;  
				 pitch_init_count   = 0;   //��λ
				 pitch_init_error   = 0;	
				 pitch_init_state   = 0;			
			}    
  }
	else  //P���ʼ�����
	{
	  gimbal_output[0] = Motor_Position_Speed_Control(&GIMBAL_S_PITCH_PID , &GIMBAL_P_PITCH_PID , 0 , Pitch_Motor_Data.speed , (Pitch_Motor_Data.actual_Position*360/1024) , PITCH_OUTPUT_LIMIT); 
	}
}


/*===============================��̨��������========================*/
first_order_low_filter_type_t  Gimbal_Updown_LowFilt_Pitch_Data = {0.0f,0.0f,0.0f,Gimbal_Pitch_Fir_Ord_Low_Fil_Param}; //��ͨ����������
//first_order_low_filter_type_t  Gimbal_Updown_LowFilt_Yaw_Data = {0.0f,0.0f,0.0f,Gimbal_Yaw_Fir_Ord_Low_Fil_Param};     //��ͨƫ��������
//first_order_low_filter_type_t  Gimbal_Updown_LowFilt_Remote_data = {0.0f,0.0f,0.0f,Gimbal_Remote_Fir_Ord_Low_Fil_Param};//��ͨң����
//acceleration_control_type_t    Gimbal_Pitch_acc_limit_Data  = {0,0,0,0,GIMBAL_PITCH_ACCELERAD};  //P����ٶ�����                        
//acceleration_control_type_t    Gimbal_Yaw_acc_limit_Data  = {0,0,0,0,GIMBAL_YAW_ACCELERAD};    //Y����ٶ�����   
sliding_mean_filter_type_t    Gimbal_Updown_Slidmean_Pitch_Data ;                                //P�Ử���˲���

int16_t pitch_filt_output=0;          //P���˲�ֵ
int16_t yaw_filt_output=0;            //Y���˲�ֵ
//int16_t remote_ch3_filt_output=0;     //ң���˲�ֵ
int16_t pitch_real_jscope=0;   //P���ӡʵ������
int16_t pitch_set_jscope=0;    //P���ӡ�趨����
float yaw_real_jscope=0;       //Y���ӡʵ������
float yaw_set_jscope=0;        //Y���ӡ�趨����
float sec_yaw_set_jscope=0;    //��Y���ӡ�趨����
float sec_yaw_real_jscope=0;   //��Y���ӡʵ������
float Sec_yaw_set_angle=0;     //��yaw���趨ת��Ƕ�
float Sec_yaw_angle=0;         //��yaw����ԽǶ�
float Gimbal_set_position=0;   //����̨pid�趨λ��
static int8_t chassis_dance_flag=0;   //���ڵ�������ʱ������̨���Ʒ�ʽ�л����µľ��ԽǶ����ת��

static void Gimbal_Work(void)
{
	if(gimbal_work_mode != 1) //ʹ����̨����ʱˢ�³���һ���Ƕȣ���ֹ�л�ģʽ�Ƕȱ仯��
	{
		Gimbal_yaw_angle=0;
		Last_Gimbal_yaw_angle = BMI160_Data.yaw_angle;
		gimbal_work_mode = 1;
		Sliding_Mean_Filter_Init(&Gimbal_Updown_Slidmean_Pitch_Data);  //��ʼ��P�Ử���˲���
		
		Remote_reload();
	}
  Gimbal_yaw_angle = BMI160_Data.yaw_angle-Last_Gimbal_yaw_angle;         //��ȡ��̨��������ʱ��������Y��Ƕ�
 	Gimbal_Angle_Limit();                                                   //��̨�Ƕ�����

  /*����̨����*/
	Gimbal_set_position = loop_fp32_constrain((-Remote_data.RC_ch2-(YAW_ANGLE_FLAG*Gimbal_yaw_angle))  ,  -180.0f  ,  180.0f);  //��̨�趨λ��ѭ���޷�
 
	gimbal_output[0] = Sliding_Mean_Filter(&Gimbal_Updown_Slidmean_Pitch_Data , gimbal_output[0] , 55);     //��ֵ�����˲������ͺ�	
	gimbal_output[0] = First_Order_Low_Filter(&Gimbal_Updown_LowFilt_Pitch_Data , gimbal_output[0]); 

  gimbal_output[0] = Motor_Position_Speed_Control(&GIMBAL_S_PITCH_PID , &GIMBAL_P_PITCH_PID , 0 , /*Pitch_Motor_Data.speed*/BMI160_Data.Gyro_X , -((Remote_data.RC_ch3)-(Pitch_Motor_Data.actual_Position*360/1024)) , PITCH_OUTPUT_LIMIT); //-(Remote_data.RC_ch3-(Pitch_Motor_Data.actual_Position*360/1024))
	gimbal_output[1] = Motor_Position_Speed_Control(&GIMBAL_S_YAW_PID , &GIMBAL_P_YAW_PID , 0 , /*(YAW_GYRO_ANGULAR_SPEED)*/Yaw_Motor_Data.speed, Gimbal_set_position , YAW_OUTPUT_LIMIT);//Yawλ�ÿ���
  

  /*����̨����*/	
  #ifdef double_gimbal 
	Sec_yaw_set_angle = loop_fp32_constrain((Yaw_different_angle - Sec_Yaw_different_angle) , -180 , 180);
	Sec_yaw_angle     = loop_fp32_constrain((Gimbal_yaw_angle - Sec_Yaw_different_angle)    , -180 , 180); //��yaw����ԽǶȽ���
	
	//Yaw����
	  gimbal_output[2] = Motor_Position_Speed_Control(&GIMBAL_DOUBLE_S_YAW_PID , &GIMBAL_DOUBLE_P_YAW_PID , 0 , Second_Yaw_Motor_Data.speed, loop_fp32_constrain(-(Sec_Yaw_different_angle),-180,180) , 2000);  //����̨���ջ�����
	
	//Pitch����
    //TIM3->CCR1=limit_int16(-((Pitch_Motor_Data.position-195)*30/130)+140 , 169 , 111);     //������p��                        
//	TIM3->CCR1=limit_int16((-(Remote_data.RC_sw/660)*30)+140 , 169 , 111);   //���ֵ��� (170-110)                           
     TIM3->CCR1=limit_int16((-(Remote_data.RC_ch3))+140 , 169 , 111);   //ң��ch3����                           

	#endif 
	

	/*��ӡ����*/
	pitch_real_jscope = gimbal_output[0];    //jscope�۲����߱仯
//  pitch_set_jscope = (Pitch_Motor_Data.actual_Position*360/1024); 
//	yaw_real_jscope = YAW_ANGLE_FLAG*Gimbal_yaw_angle;    //jscope�۲����߱仯
//  yaw_set_jscope = -Remote_data.RC_ch2;

}



/*==============================��̨ģʽ��̨==========================*/
static void Gimbal_Independent(void)
{
	if(gimbal_work_mode != 1) //ʹ����̨����ʱˢ�³���һ���Ƕȣ���ֹ�л�ģʽ�Ƕȱ仯��
	{
		Gimbal_yaw_angle=0;
		Last_Gimbal_yaw_angle = BMI160_Data.yaw_angle;
		gimbal_work_mode = 1;
	}
  Gimbal_yaw_angle = BMI160_Data.yaw_angle-Last_Gimbal_yaw_angle;//��ȡ��̨��������ʱ��������Y��Ƕ�
	
 	Gimbal_Angle_Limit();                                                   //��̨�Ƕ�����


  gimbal_output[0] = Motor_Position_Speed_Control(&GIMBAL_S_PITCH_PID , &GIMBAL_P_PITCH_PID , 0 , Pitch_Motor_Data.speed , -((Remote_data.RC_ch3)-(Pitch_Motor_Data.actual_Position*360/1024)) , PITCH_OUTPUT_LIMIT); //-(Remote_data.RC_ch3-(Pitch_Motor_Data.actual_Position*360/1024))
	gimbal_output[1] = Motor_Position_Speed_Control(&GIMBAL_INDEPENDENT_S_YAW_PID , &GIMBAL_INDEPENDENT_P_YAW_PID , 0 ,Yaw_Motor_Data.speed, (-Remote_data.RC_ch2-(YAW_ANGLE_FLAG*Gimbal_yaw_angle)) , YAW_OUTPUT_LIMIT);//Yawλ�ÿ���
 
	/*��ͨ�˲�*/
//	remote_ch3_filt_output = First_Order_Low_Filter(&Gimgimbal_outputbal_Updown_LowFilt_Remote_data , -(Remote_data.RC_ch3-(Pitch_Motor_Data.actual_Position*360/1024))); //Y���������ͨ�˲�  
//	yaw_filt_output = First_Order_Low_Filter(&Gimbal_Updown_LowFilt_Yaw_Data , gimbal_output[1]); //Y���������ͨ�˲�
//	pitch_filt_output = First_Order_Low_Filter(&Gimbal_Updown_LowFilt_Pitch_Data , [0]);  //P���������ͨ�˲�
//  pitch_filt_output = Motion_acceleration_control(&Gimbal_Pitch_acc_limit_Data , gimbal_output[0]); //P����ٶ�����
	
	/*��ӡ����*/
	pitch_real_jscope = gimbal_output[0];    //jscope�۲����߱仯
  pitch_set_jscope = (Pitch_Motor_Data.actual_Position*360/1024); 
	
//  gimbal_output[1] = yaw_filt_output;	
//  gimbal_output[0] = pitch_filt_output;	
}


/*==============================��̨����============================*/
static float Gimbal_supply_angle = 0;         //����ʱ�Ƕ�
static float Gimbal_supply_angle_back = 0.0f; //�������ؽǶ�

static void Gimbal_Supply(void)
{
	/*���벹��ģʽ*/
	if(GIMBAL_SUPPLY_FLAG==0)
	{
		Gimbal_supply_angle = BMI160_Data.yaw_angle + 70.0f;  //����ʱĿ��Ƕ�Ϊ60��
	  GIMBAL_SUPPLY_FLAG = 1;                               //���벹��״̬
	}
	/*�������Ƕ�*/
	else if(GIMBAL_SUPPLY_FLAG==1)  
	{
	  if(abs_float(BMI160_Data.yaw_angle - Gimbal_supply_angle) <= 3.0f) //����ָ��λ��
		{
			GIMBAL_SUPPLY_FLAG = 2;
			gimbal_output[1] = 0;
		  Gimbal_supply_angle_back = BMI160_Data.yaw_angle - 70.0f;  //����ʱĿ��Ƕ�Ϊ-90��			
		}
		else
		{
			gimbal_output[1] = Motor_Position_Speed_Control(&GIMBAL_S_YAW_PID , &GIMBAL_P_YAW_PID , 0 , Yaw_Motor_Data.speed, (Gimbal_supply_angle-(YAW_ANGLE_FLAG*BMI160_Data.yaw_angle)) , 600);//Yawλ�ÿ��� 	
		}
	}
	/*�����м��*/
	else if(GIMBAL_SUPPLY_FLAG==3)//����λǰ�ᰴ�°�����
	{
	  gimbal_output[1] = Motor_Position_Speed_Control(&GIMBAL_S_YAW_PID , &GIMBAL_P_YAW_PID , 0 , Yaw_Motor_Data.speed, (Gimbal_supply_angle_back-(YAW_ANGLE_FLAG*BMI160_Data.yaw_angle)) , 600);//Yawλ�ÿ��� 			
	  if(abs_float(BMI160_Data.yaw_angle - Gimbal_supply_angle_back) <= 3.0f) //����ָ��λ��
		{
			GIMBAL_SUPPLY_FLAG = 4;
			Gimbal_supply_angle = 0;
			Gimbal_supply_angle_back = 0;
		}	  
	}
	
	 /*����̨����*/
	 #ifdef double_gimbal
		Sec_yaw_set_angle = loop_fp32_constrain((Yaw_different_angle - Sec_Yaw_different_angle) , -180 , 180);
		Sec_yaw_angle     = loop_fp32_constrain((Remote_data.RC_ch2+(Gimbal_yaw_angle - Sec_Yaw_different_angle)) , -180 , 180); //��yaw����ԽǶȽ���
	  gimbal_output[2] = Motor_Position_Speed_Control(&GIMBAL_DOUBLE_S_YAW_PID , &GIMBAL_DOUBLE_P_YAW_PID , 0 , Second_Yaw_Motor_Data.speed, (Sec_yaw_set_angle) , 2000);  //����̨���ջ�����

	//Pitch����
   	TIM3->CCR1=limit_int16((-(Remote_data.RC_ch3))+140 , 169 , 111);                            
	 #endif
}



/*===============================��̨�Զ���׼==================================*/
static void Gimbal_Auto(void)
{
	if(gimbal_work_mode != 2) //ʹ����̨����ʱˢ�³���һ���Ƕȣ���ֹ�л�ģʽ�Ƕȱ仯��
	{ 
		Gimbal_yaw_angle=0;
		Last_Gimbal_yaw_angle = BMI160_Data.yaw_angle;		
		gimbal_work_mode = 2;
		Change_camera(1);      //����ģʽ���л�����ͷ����Ϊ�̽����
		Vision_Auto_Data.pitch_control_data = 0;
		Vision_Auto_Data.yaw_control_data   = 0;		
	}
	 /*Ԥ����*/
   Gimbal_yaw_angle = BMI160_Data.yaw_angle-Last_Gimbal_yaw_angle;                                                             //��ȡ��̨�Զ����ʱ��������Y��Ƕ�
//	 Kalman_Filter_Calc(&kalman_Filter, Vision_Auto_Data.auto_yaw_angle+Gimbal_yaw_angle, Vision_Auto_Data.auto_pitch_angle);  //��С���Դ���ǶȽ��п������˲�

   /*��̨�Ӿ�������*/	//(y�ᣬp�ỹû�ÿ�����)
	 Vision_Auto_Data.pitch_control_data = Vision_Auto_Data.auto_pitch_angle*1.7f; //    Vision_Auto_Data.auto_pitch_sum - (Pitch_Motor_Data.actual_Position*360/1024)
	 Vision_Auto_Data.yaw_control_data   = Vision_Auto_Data.auto_yaw_angle;        // - Vision_Auto_Data.auto_yaw_speed*9;    //14  auto_yaw_angle_kf��һ��������kalman������yaw�ĽǶȣ���Ϊ����Ŀ��ƣ����Բ��ü�ȥ�����ǵĽǶȣ�����ֱ�Ӽ�������
	
   /*������ջ�����*/
//		if( Vision_Auto_Data.yaw_control_data==0 && Vision_Auto_Data.pitch_control_data==0) //�Ӿ������ݷ���ʱ��y������
//		{
//			gimbal_output[1] = Rmmotor_Speed_control(&GIMBAL_S_YAW_PID, -900, Yaw_Motor_Data.speed, 1500);		
//		}
//		else  //ʶ��Ŀ�꣬����
//		{
			gimbal_output[1] = Motor_Position_Speed_Control(&GIMBAL_AUTO_S_YAW_PID , &GIMBAL_AUTO_P_YAW_PID , 0 , /*(-YAW_GYRO_ANGULAR_SPEED)*/Yaw_Motor_Data.speed , (-Vision_Auto_Data.yaw_control_data) , YAW_OUTPUT_LIMIT);//Yawλ�ÿ���
//		}
		
	 gimbal_output[0] = Motor_Position_Speed_Control(&GIMBAL_AUTO_S_PITCH_PID , &GIMBAL_AUTO_P_PITCH_PID , 0 , /*Pitch_Motor_Data.speed*/ BMI160_Data.Gyro_X , /*-*/(Vision_Auto_Data.pitch_control_data) , PITCH_OUTPUT_LIMIT); 

	 /*����̨����*/
	 #ifdef double_gimbal
		Sec_yaw_set_angle = loop_fp32_constrain((Yaw_different_angle - Sec_Yaw_different_angle) , -180 , 180);
		Sec_yaw_angle     = loop_fp32_constrain((Remote_data.RC_ch2+(Gimbal_yaw_angle - Sec_Yaw_different_angle)) , -180 , 180); //��yaw����ԽǶȽ���
	  gimbal_output[2] = Motor_Position_Speed_Control(&GIMBAL_DOUBLE_S_YAW_PID , &GIMBAL_DOUBLE_P_YAW_PID , 0 , Second_Yaw_Motor_Data.speed, (Sec_yaw_angle) , 2000);  //����̨���ջ�����

	//Pitch����
   	TIM3->CCR1=limit_int16((-(Remote_data.RC_ch3))+140 , 169 , 111);                            
	
	 #endif


		/*��ӡ����*/
		pitch_real_jscope = -gimbal_output[1];    //jscope�۲����߱仯
		pitch_set_jscope  = -GIMBAL_AUTO_P_YAW_PID.SetValue; 
}


/*==================================��̨���ģʽ==============================*/
float pitch_data_filt=0.0f;                                                //�Ӿ�p�������˲�ֵ
float yaw_data_filt=0.0f;                                                //�Ӿ�y�������˲�ֵ
float pitch_Kalman_filt=0.0f;                                                //�Ӿ�p�Ῠ��������ֵ
first_order_low_filter_type_t pitch_low_filt  = {0.0f,0.0f,0.0f, 0.02642}; //һ�׵�ͨ�˲��� 0.01642
first_order_low_filter_type_t yaw_low_filt    = {0.0f,0.0f,0.0f, 0.05642}; //һ�׵�ͨ�˲��� 0.01642

sliding_mean_filter_type_t    pitch_mean_filt ;                            //�����˲���
extKalman_t                   pitch_kalman_filt;                           //�������˲���

static void Gimbal_Buff(void) 
{
	if(gimbal_work_mode != 2) //ģʽ�л�����
	{ 
		Gimbal_yaw_angle = 0;
		Last_Gimbal_yaw_angle = BMI160_Data.yaw_angle;//ʹ����̨����ʱˢ�³���һ���Ƕȣ���ֹ�л�ģʽ�Ƕȱ仯��
		Vision_Auto_Data.pitch_control_data = 0;
		Vision_Auto_Data.yaw_control_data   = 0;
		gimbal_work_mode = 2;
		Change_camera(3);                            //���ģʽ�£��л�����ͷΪ��ҵ�������		
		Sliding_Mean_Filter_Init(&pitch_mean_filt);  //��ʼ��P�Ử���˲���
		KalmanCreate(&pitch_kalman_filt,0.001,0.02);     //��ʼ�����˲�����Q=20 R=200����    //* @attention R�̶���QԽ�󣬴���Խ���β���ֵ��Q�������ֻ�ò���ֵ
                                                                                   //*		       	��֮��QԽС����Խ����ģ��Ԥ��ֵ��QΪ������ֻ��ģ��Ԥ��
		
	}
	 /*Ԥ����*/
   Gimbal_yaw_angle = BMI160_Data.yaw_angle-Last_Gimbal_yaw_angle;                                                               //��ȡ��̨�Զ����ʱ��������Y��Ƕ�

   /*��̨�Ӿ�������*/	//(y�����˿�������p�ỹû��)   //4.3
	 Vision_Auto_Data.pitch_control_data = Vision_Auto_Data.auto_pitch_angle*1.0f;  //1.93    Vision_Auto_Data.auto_pitch_sum - (Pitch_Motor_Data.actual_Position*360/1024)
	 Vision_Auto_Data.yaw_control_data   = Vision_Auto_Data.auto_yaw_angle*1.5f;    // - Vision_Auto_Data.auto_yaw_speed*14;    //auto_yaw_angle_kf��һ��������kalman������yaw�ĽǶȣ���Ϊ����Ŀ��ƣ����Բ��ü�ȥ�����ǵĽǶȣ�����ֱ�Ӽ�������

	 //�˲�p��
//   Data_Accelerated_Control(&Vision_Auto_Data.pitch_control_data , 3.6f);	//2.6                                                //б�¼��ٶ����ƺ���
//   Vision_Auto_Data.pitch_control_data = Sliding_Mean_Filter(&pitch_mean_filt , Vision_Auto_Data.pitch_control_data , 35);     //��ֵ�����˲������ͺ�	
//  	 pitch_data_filt = First_Order_Low_Filter(&pitch_low_filt , Vision_Auto_Data.pitch_control_data);                            //һ�׵�ͨ�˲�

	//	 pitch_Kalman_filt = pitch_data_filt;
//	 pitch_data_filt = KalmanFilter(&pitch_kalman_filt,pitch_data_filt);                                                         //�������˲�
//	 Kalman_Filter_Calc(&kalman_Filter, Vision_Auto_Data.auto_yaw_angle+Gimbal_yaw_angle, Vision_Auto_Data.auto_pitch_angle);    //���׿������˲�

	  //�˲�y��
	 yaw_data_filt = First_Order_Low_Filter(&yaw_low_filt , Vision_Auto_Data.yaw_control_data);                            //һ�׵�ͨ�˲�

	
   /*������ջ�����*/
	 gimbal_output[1] = Motor_Position_Speed_Control(&GIMBAL_AUTO_BUFF_S_YAW_PID ,
                                                   &GIMBAL_AUTO_BUFF_P_YAW_PID ,
                                                   0 , 
	                                                 Yaw_Motor_Data.speed , //BMI160_Data.Gyro_Z   Yaw_Motor_Data.speed
                                                	 (-yaw_data_filt) , 
	                                                 YAW_OUTPUT_LIMIT);//Yawλ�ÿ���
	
   gimbal_output[0] = Motor_Position_Speed_Control(&GIMBAL_AUTO_BUFF_S_PITCH_PID , 
	                                                 &GIMBAL_AUTO_BUFF_P_PITCH_PID ,
                                                   0 ,
                                                   Pitch_Motor_Data.speed ,
                         	                         Vision_Auto_Data.pitch_control_data,//Vision_Auto_Data.pitch_control_data,/*-(Pitch_Motor_Data.actual_Position*360/1024)*/ //(Remote_data.RC_ch3)//û�м�⵽Ŀ��ʱp�����
                                                   PITCH_OUTPUT_LIMIT); 
									 		 														 
	
	 /*����̨����*/
		#ifdef double_gimbal
		Sec_yaw_set_angle = loop_fp32_constrain((Yaw_different_angle - Sec_Yaw_different_angle) , -180 , 180);
	  gimbal_output[2] = Motor_Position_Speed_Control(&GIMBAL_DOUBLE_S_YAW_PID , &GIMBAL_DOUBLE_P_YAW_PID , 0 , Second_Yaw_Motor_Data.speed, (Sec_yaw_set_angle) , 2000);  //����̨���ջ�����
    #endif
	 
		/*��ӡ����*/
		pitch_real_jscope = -gimbal_output[1];    //jscope�۲����߱仯
		pitch_set_jscope = -GIMBAL_AUTO_P_YAW_PID.SetValue; 	
}


/*==================================��̨����==================================*/
static void Gimbal_Stop(void)
{
   gimbal_output[0] = 0;  //P
	 gimbal_output[1] = 0;  //Y
	 gimbal_output[2] = 0;
	 gimbal_output[3] = 0;
}


/******************�����㷨************************/

/*
* ���ܣ���̨�ĽǶ�����
* ���룺ң��ֵ�ṹ��
* �������
* ���������ƴ����Ŀ�����
*/
static void Gimbal_Angle_Limit(void)
{
	//pitch�Ƕ�����
	Remote_data.RC_ch3 = limit_float(Remote_data.RC_ch3, PITCH_ANGLE_LIMIT_UP, PITCH_ANGLE_LIMIT_DOWN);  
  //yaw�Ƕ����� 
//	Remote_data.RC_ch2 = limit_float(Remote_data.RC_ch2, YAW_ANGLE_LIMIT, -YAW_ANGLE_LIMIT);
}

/******************************************/



/*
* ���ܣ�Yaw��������λ�õĹ��У��
* ���룺��
* �������
* ��������̨�������������дγ������ü��Y��ֵ����������Y���������ʵֵ���м�Ϊ0
*/
static int16_t correctYawValue[5]={0};
static void Correct_Yaw_Zero(void)
{
	u8 i=0;
	u16 correctSum=0;
	
	for(i=0;i<5;i++)
	{
		correctYawValue[i-1]=correctYawValue[i];
	}
	correctYawValue[4] = !YAW_ZERO_FLAG;//Yaw_Zero_Value();
	
	for(i=0;i<5;i++)
	{
		correctSum+=correctYawValue[i];
	}
	
	correctSum/=5;
	
	if(correctSum==0)
	{
		Yaw_Motor_Data.actual_Position=0;
	}
}


/*
* ���ܣ���Yaw��������λ�õĹ��У��
* ���룺��
* �������
* ��������̨�������������дγ������ü��Y��ֵ����������Y���������ʵֵ���м�Ϊ0
*/
static int16_t correctSecYawValue[5]={0};
static void Correct_Sec_Yaw_Zero(void)
{
	u8 i=0;
	u16 correctSum=0;
	
	for(i=0;i<5;i++)
	{
		correctSecYawValue[i-1]=correctSecYawValue[i];
	}
	correctSecYawValue[4] = !SEC_YAW_ZERO_FLAG;//Yaw_Zero_Value();
	
	for(i=0;i<5;i++)
	{
		correctSum+=correctSecYawValue[i];
	}
	
	correctSum/=5;
	
	if(correctSum==0)
	{
		Second_Yaw_Motor_Data.actual_Position=0;
	}
}




/*
*���ܣ�ͨ�����յ�С����������Ÿı�pid�Ĳ���ֵ
*���룺������ţ�1���̽�  2������  3����ҵ��
*�������
*/

static void Change_camera(int camera_num)
{
 if(camera_num==1)
 {
  //Auto_Pitch
  GIMBAL_AUTO_P_PITCH_PID.kp=GIMBAL_AUTO_SHORT_P_PITCH_P;
	GIMBAL_AUTO_P_PITCH_PID.ki=GIMBAL_AUTO_SHORT_P_PITCH_I;
	GIMBAL_AUTO_P_PITCH_PID.kd=GIMBAL_AUTO_SHORT_P_PITCH_D;
	GIMBAL_AUTO_P_PITCH_PID.Error=0;
	GIMBAL_AUTO_P_PITCH_PID.iError=0;
	GIMBAL_AUTO_P_PITCH_PID.LastError=0;
	GIMBAL_AUTO_P_PITCH_PID.OutputValue=0;
	GIMBAL_AUTO_P_PITCH_PID.outputValue_Int32=0;
	GIMBAL_AUTO_P_PITCH_PID.LastOutput=0;
	GIMBAL_AUTO_P_PITCH_PID.SetValue=0;
	
	GIMBAL_AUTO_S_PITCH_PID.kp=GIMBAL_AUTO_SHORT_S_PITCH_P;
	GIMBAL_AUTO_S_PITCH_PID.ki=GIMBAL_AUTO_SHORT_S_PITCH_I;
	GIMBAL_AUTO_S_PITCH_PID.kd=GIMBAL_AUTO_SHORT_S_PITCH_D;
	GIMBAL_AUTO_S_PITCH_PID.Error=0;
	GIMBAL_AUTO_S_PITCH_PID.iError=0;	
	GIMBAL_AUTO_S_PITCH_PID.LastError=0;
	GIMBAL_AUTO_S_PITCH_PID.PrevError=0;
	GIMBAL_AUTO_S_PITCH_PID.OutputValue=0;
	GIMBAL_AUTO_S_PITCH_PID.outputValue_Int32=0;
	GIMBAL_AUTO_S_PITCH_PID.LastOutput=0;
	GIMBAL_AUTO_S_PITCH_PID.SetValue=0;
	
	//Auto_Yaw
	GIMBAL_AUTO_P_YAW_PID.kp=GIMBAL_AUTO_SHORT_P_YAW_P;
	GIMBAL_AUTO_P_YAW_PID.ki=GIMBAL_AUTO_SHORT_P_YAW_I;
	GIMBAL_AUTO_P_YAW_PID.kd=GIMBAL_AUTO_SHORT_P_YAW_D;
	GIMBAL_AUTO_P_YAW_PID.Error=0;
	GIMBAL_AUTO_P_YAW_PID.iError=0;	
	GIMBAL_AUTO_P_YAW_PID.LastError=0;
	GIMBAL_AUTO_P_YAW_PID.OutputValue=0;
	GIMBAL_AUTO_P_YAW_PID.outputValue_Int32=0;
	GIMBAL_AUTO_P_YAW_PID.LastOutput=0;
	GIMBAL_AUTO_P_YAW_PID.SetValue=0;
	
	GIMBAL_AUTO_S_YAW_PID.kp=GIMBAL_AUTO_SHORT_S_YAW_P;
	GIMBAL_AUTO_S_YAW_PID.ki=GIMBAL_AUTO_SHORT_S_YAW_I;
	GIMBAL_AUTO_S_YAW_PID.kd=GIMBAL_AUTO_SHORT_S_YAW_D;
	GIMBAL_AUTO_S_YAW_PID.Error=0;
	GIMBAL_AUTO_S_YAW_PID.iError=0;
	GIMBAL_AUTO_S_YAW_PID.LastError=0;
	GIMBAL_AUTO_S_YAW_PID.PrevError=0;
	GIMBAL_AUTO_S_YAW_PID.OutputValue=0;
	GIMBAL_AUTO_S_YAW_PID.outputValue_Int32=0;
	GIMBAL_AUTO_S_YAW_PID.LastOutput=0;
	GIMBAL_AUTO_S_YAW_PID.SetValue=0;
  }
  else if(camera_num==2)
	{
  //Auto_Pitch
  GIMBAL_AUTO_P_PITCH_PID.kp=GIMBAL_AUTO_LONG_P_PITCH_P;
	GIMBAL_AUTO_P_PITCH_PID.ki=GIMBAL_AUTO_LONG_P_PITCH_I;
	GIMBAL_AUTO_P_PITCH_PID.kd=GIMBAL_AUTO_LONG_P_PITCH_D;
	GIMBAL_AUTO_P_PITCH_PID.Error=0;
	GIMBAL_AUTO_P_PITCH_PID.iError=0;
	GIMBAL_AUTO_P_PITCH_PID.LastError=0;
	GIMBAL_AUTO_P_PITCH_PID.OutputValue=0;
	GIMBAL_AUTO_P_PITCH_PID.outputValue_Int32=0;
	GIMBAL_AUTO_P_PITCH_PID.LastOutput=0;
	GIMBAL_AUTO_P_PITCH_PID.SetValue=0;
	
	GIMBAL_AUTO_S_PITCH_PID.kp=GIMBAL_AUTO_LONG_S_PITCH_P;
	GIMBAL_AUTO_S_PITCH_PID.ki=GIMBAL_AUTO_LONG_S_PITCH_I;
	GIMBAL_AUTO_S_PITCH_PID.kd=GIMBAL_AUTO_LONG_S_PITCH_D;
	GIMBAL_AUTO_S_PITCH_PID.Error=0;
	GIMBAL_AUTO_S_PITCH_PID.iError=0;	
	GIMBAL_AUTO_S_PITCH_PID.LastError=0;
	GIMBAL_AUTO_S_PITCH_PID.PrevError=0;
	GIMBAL_AUTO_S_PITCH_PID.OutputValue=0;
	GIMBAL_AUTO_S_PITCH_PID.outputValue_Int32=0;
	GIMBAL_AUTO_S_PITCH_PID.LastOutput=0;
	GIMBAL_AUTO_S_PITCH_PID.SetValue=0;
	
	//Auto_Yaw
	GIMBAL_AUTO_P_YAW_PID.kp=GIMBAL_AUTO_LONG_P_YAW_P;
	GIMBAL_AUTO_P_YAW_PID.ki=GIMBAL_AUTO_LONG_P_YAW_I;
	GIMBAL_AUTO_P_YAW_PID.kd=GIMBAL_AUTO_LONG_P_YAW_D;
	GIMBAL_AUTO_P_YAW_PID.Error=0;
	GIMBAL_AUTO_P_YAW_PID.iError=0;	
	GIMBAL_AUTO_P_YAW_PID.LastError=0;
	GIMBAL_AUTO_P_YAW_PID.OutputValue=0;
	GIMBAL_AUTO_P_YAW_PID.outputValue_Int32=0;
	GIMBAL_AUTO_P_YAW_PID.LastOutput=0;
	GIMBAL_AUTO_P_YAW_PID.SetValue=0;
	
	GIMBAL_AUTO_S_YAW_PID.kp=GIMBAL_AUTO_LONG_S_YAW_P;
	GIMBAL_AUTO_S_YAW_PID.ki=GIMBAL_AUTO_LONG_S_YAW_I;
	GIMBAL_AUTO_S_YAW_PID.kd=GIMBAL_AUTO_LONG_S_YAW_D;
	GIMBAL_AUTO_S_YAW_PID.Error=0;
	GIMBAL_AUTO_S_YAW_PID.iError=0;
	GIMBAL_AUTO_S_YAW_PID.LastError=0;
	GIMBAL_AUTO_S_YAW_PID.PrevError=0;
	GIMBAL_AUTO_S_YAW_PID.OutputValue=0;
	GIMBAL_AUTO_S_YAW_PID.outputValue_Int32=0;
	GIMBAL_AUTO_S_YAW_PID.LastOutput=0;
	GIMBAL_AUTO_S_YAW_PID.SetValue=0;
  }
	else if(camera_num==3)
	{
  //Auto_Pitch
  GIMBAL_AUTO_P_PITCH_PID.kp=GIMBAL_AUTO_INDUSTRY_P_PITCH_P;
	GIMBAL_AUTO_P_PITCH_PID.ki=GIMBAL_AUTO_INDUSTRY_P_PITCH_I;
	GIMBAL_AUTO_P_PITCH_PID.kd=GIMBAL_AUTO_INDUSTRY_P_PITCH_D;
	GIMBAL_AUTO_P_PITCH_PID.Error=0;
	GIMBAL_AUTO_P_PITCH_PID.iError=0;
	GIMBAL_AUTO_P_PITCH_PID.LastError=0;
	GIMBAL_AUTO_P_PITCH_PID.OutputValue=0;
	GIMBAL_AUTO_P_PITCH_PID.outputValue_Int32=0;
	GIMBAL_AUTO_P_PITCH_PID.LastOutput=0;
	GIMBAL_AUTO_P_PITCH_PID.SetValue=0;
	
	GIMBAL_AUTO_S_PITCH_PID.kp=GIMBAL_AUTO_INDUSTRY_S_PITCH_P;
	GIMBAL_AUTO_S_PITCH_PID.ki=GIMBAL_AUTO_INDUSTRY_S_PITCH_I;
	GIMBAL_AUTO_S_PITCH_PID.kd=GIMBAL_AUTO_INDUSTRY_S_PITCH_D;
	GIMBAL_AUTO_S_PITCH_PID.Error=0;
	GIMBAL_AUTO_S_PITCH_PID.iError=0;	
	GIMBAL_AUTO_S_PITCH_PID.LastError=0;
	GIMBAL_AUTO_S_PITCH_PID.PrevError=0;
	GIMBAL_AUTO_S_PITCH_PID.OutputValue=0;
	GIMBAL_AUTO_S_PITCH_PID.outputValue_Int32=0;
	GIMBAL_AUTO_S_PITCH_PID.LastOutput=0;
	GIMBAL_AUTO_S_PITCH_PID.SetValue=0;
	
	//Auto_Yaw
	GIMBAL_AUTO_P_YAW_PID.kp=GIMBAL_AUTO_INDUSTRY_P_YAW_P;
	GIMBAL_AUTO_P_YAW_PID.ki=GIMBAL_AUTO_INDUSTRY_P_YAW_I;
	GIMBAL_AUTO_P_YAW_PID.kd=GIMBAL_AUTO_INDUSTRY_P_YAW_D;
	GIMBAL_AUTO_P_YAW_PID.Error=0;
	GIMBAL_AUTO_P_YAW_PID.iError=0;	
	GIMBAL_AUTO_P_YAW_PID.LastError=0;
	GIMBAL_AUTO_P_YAW_PID.OutputValue=0;
	GIMBAL_AUTO_P_YAW_PID.outputValue_Int32=0;
	GIMBAL_AUTO_P_YAW_PID.LastOutput=0;
	GIMBAL_AUTO_P_YAW_PID.SetValue=0;
	
	GIMBAL_AUTO_S_YAW_PID.kp=GIMBAL_AUTO_INDUSTRY_S_YAW_P;
	GIMBAL_AUTO_S_YAW_PID.ki=GIMBAL_AUTO_INDUSTRY_S_YAW_I;
	GIMBAL_AUTO_S_YAW_PID.kd=GIMBAL_AUTO_INDUSTRY_S_YAW_D;
	GIMBAL_AUTO_S_YAW_PID.Error=0;
	GIMBAL_AUTO_S_YAW_PID.iError=0;
	GIMBAL_AUTO_S_YAW_PID.LastError=0;
	GIMBAL_AUTO_S_YAW_PID.PrevError=0;
	GIMBAL_AUTO_S_YAW_PID.OutputValue=0;
	GIMBAL_AUTO_S_YAW_PID.outputValue_Int32=0;
	GIMBAL_AUTO_S_YAW_PID.LastOutput=0;
	GIMBAL_AUTO_S_YAW_PID.SetValue=0;		
	}
 }

 
 
 
 /*
*���ܣ������˫��̨�Լ����̵���ԽǶ�
*���룺1 ��Yaw����ԽǶ�()   2 ��yaw����ԽǶ�()  3 ��yaw����Ե��̲�ǣ�+-180��   4 ��yaw�������yaw���ǣ�+-180��
*�������
*/
 
 