/*==================================================*
*�ļ����ܣ�1.���̿�������
*         2.���ֵ���ģʽ�Ŀ����㷨
*         3.�������ƣ����ȼ���
*            
*��ע��ԓ�ļ���û�������ϵͳ�ķֵȼ���������
*           δ�ӳ������ݿ��Ʋ���
*==================================================*/

/*=====================*
*�ļ�����
*��飺
*�������ܣ�
*======================*/


#include "main.h"
#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "Remote_Task.h"
#include "Speed_Def.h"
#include "PID_Def.h"
#include "Mode_Def.h"
#include "Capacitor_control.h"


//==============��������
int chassis_heart = 0;             //������������
float Yaw_different_angle = 0.0f;  //Y������̵ĽǶȲ�
extern int16_t Sec_chassis_angle;  //����̨��Ե��̵Ĳ��
extern int8_t  Vision_flag;        //�Ӿ�ģʽ�л���־��0���رգ�1�����飬2���������أ�

int ChassisTask_water=0;
static int16_t chassis_input[4] =  {0};                              //���̵����������
static int16_t chassis_output[4] = {0};                              //���̵���������
static float Chassis_ch0=0.0f, Chassis_ch1=0.0f, Chassis_ch2=0.0f ;  //���̵���ܿ���



//==============�ṹ�塢ö�ٶ���
extern REFEREE_t REFEREE;
extern TaskHandle_t ChassisTask_Handler; // ջ��С
PowerLimit_t Chassis_PowerLimit;                         //���̹������ƽṹ��
Chassis_WorkStatus  chassis_workStatus = FOLLOWING;      //����Ĭ�Ϲ���ģʽΪ����

//==============�ڲ���������
static void Chassis_control(void);                                               //����ģʽ����
static void Chassis_Follow(void);                                                //���̸���
static void Chassis_Twist(void);																								 //����Ť��
static void Chassis_Rotation(void);                                              //����С����
static void Chassis_Independent(void);                                           //���̲�����
static void Chassis_Input(int16_t ch0, int16_t ch1, int16_t ch2);                //���̿������ݴ���
static void Chassis_to_Gimbal(void);                                             //���̷����ݵ���̨
static void Power_Init(void);                                                    //�������Ʋ�����ʼ��
static void Power_Real(void);                                                    //���ʲ�������
static void PowerLimitLoop(void);                                                //�ɹ����������ݴ���
static void Chassis_accelerated_Control(int16_t *ch0,int16_t *ch1,int16_t *ch2); //���ٶ����ƴ���
static void Keep_Driv_Track(int16_t *v0 ,int16_t *v1 ,int16_t *v2 ,int16_t *v3 ,int16_t SPEED_GAIN);// ��ֹ�˶��켣ʧ��



/**====��������====**/
void Chassis_Task(void *pvParameters)
{
	//����ʱ��
  vTaskDelay(CHASSIS_TASK_INIT_TIME);
	
	//�������Ʋ�����ʼ��
  Power_Init();                                                        
	
	while(1)
	{
		chassis_heart =! chassis_heart;                                       //������������(������debug�۲��Ƿ��������)
//		ChassisTask_water = uxTaskGetStackHighWaterMark(ChassisTask_Handler);
		LEDE1 = chassis_heart; 
		

	
#ifdef board_chassis 

			if((RC_Ctl.rc.s2==3 && RC_Ctl.rc.s1==2) || Vision_flag==1)
			{
				Yaw_different_angle = Sec_chassis_angle; //����̨����ϵ				
			}
			else
			{
		    Yaw_different_angle = Get_Yaw_Different_Angle(&Yaw_Motor_Data,3*19);  //�����������̨���								
			}
		
		  Chassis_control();	                                                  //���̿���
			
	   	Chassis_to_Gimbal();                                                  //can2���̷����ݵ���̨
			Chassis_control();	                                                  //���̿���

//		//��ŵ磨���ʱ���ܷŵ磬�ŵ�ʱ���ܳ�磩
    if(Remote_data.RC_ch1>600)
		{
		  GPIO_SetBits(GPIOA,GPIO_Pin_2);//����IO��ƽ�������ŵ翪��
//			TIM3->CCR2=0;
		}	
		else
		{
		  GPIO_ResetBits(GPIOA,GPIO_Pin_2);
			//���
//			TIM3->CCR2=abs_int16(limit_int16(Remote_data.RC_sw*100/660,30,0));   //���ݳ�����0-100,������97%��		
		}
#endif	
			
#ifdef board_gimbal
		   Yaw_different_angle = Get_Yaw_Different_Angle(&Yaw_Motor_Data,3*19);  //�����������̨���								
#endif
    
		
		//ϵͳ��ʱ
    vTaskDelay(CHASSIS_CONTROL_TIME_MS);
	}
}



/*���̷������ݵ���̨��yaw������������ֵ��yaw�����ٶ�ֵ��ң��ֵ��*/
static int send_sign = 0;              //˳����ң��ֵ

static void Chassis_to_Gimbal(void)
{ 
  /*���̷�ң�����ݵ���̨*/ 
        
	if(workStatus!=POWEROFF) //����ˢ������ 
	{  
		if(send_sign==0)
			{ 
			if(RC_Ctl.rc.s2 == MOUSE_CONTROL && RC_Ctl.rc.s1 == 3)
			  CAN2_Send_Data1_MK_To_Gimbal();
			else
				CAN2_Send_Data1_RC_To_Gimbal();
		  send_sign=1;
			}
		if(send_sign==1){   
			CAN2_Send_Data2_To_Gimbal();
		  send_sign=0;}
  }
 
} 
	



/**====�����˶�����====**/
static void Chassis_control(void)//����ģʽ����
{
	/*��ͬ����ģʽ�Ե�����������ͬ����*/
	if(INITIALIZE_flag == 1)
	{
		  #ifdef board_chassis
		  workStatus = WORKING;   //��ʼ����
		  #endif
		  
			if(chassis_workStatus == FOLLOWING)
			{
				Chassis_Follow();	//����
			}
			if(chassis_workStatus == TWIST_WAIST)
			{
				Chassis_Twist();  //Ť��
			}
			if(chassis_workStatus == ROTATION)
			{
				Chassis_Rotation();//С����
			}	
			if(chassis_workStatus == INDEPENDENT)
			{
				Chassis_Independent();//���̲�����
			}	
	
	}
	if(chassis_workStatus == STOP || INITIALIZE_flag == 0)
	{
		Chassis_Input(0,0,0);//ֹͣ
	}
	
}



/*====���̸���״̬����====*/
static void Chassis_Follow(void)
{
	Chassis_ch2 = Yaw_different_angle;//Angle_Limiting_Int32(Yaw_Motor_Data.actual_Position, 1, 8192);//�ٽǴ���   
	
	CHASSIS_ROTATE_FOLLOW_PID.SetValue = Chassis_ch2;       
	
	Chassis_ch2 = Location_Pid_Int32(&CHASSIS_ROTATE_FOLLOW_PID,0);	  
	
#ifdef chassis_exclusive_use	
	Chassis_Input(Remote_data.RC_ch0, Remote_data.RC_ch1, -(RC_Ctl.rc.ch2-1024));//����������У�ң�ؿ���ch2ת��
#else 
	Chassis_Input(Remote_data.RC_ch0, Remote_data.RC_ch1, Chassis_ch2);  //���̸��棬��̨��ǿ���ch2ת��
#endif
}	



/*===����Ť��״̬����=====*/
static int16_t TWIST_FLAG=0;  //0,��ת��1����ת
static int16_t Chassis_Twist_acc=0;//����Ť�����ٶ�����

static void Chassis_Twist(void)
{
	/*�ж�ת��*/
	if(Yaw_different_angle>20 && TWIST_FLAG==1)
	{
		TWIST_FLAG = 0;
	}
	else if(Yaw_different_angle<-20 && TWIST_FLAG==0)
	{
		TWIST_FLAG = 1;		
	}
	
	/*ת������*/
	 if(TWIST_FLAG == 0)//��Ƕȴ���30������ת
	 {
			//*�����̶�Ť��
		  if(Chassis_Twist_acc<=CHASSIS_TWIST_SPEED)
				Chassis_Twist_acc+=20;
			Chassis_ch2 = Chassis_Twist_acc;      
//			Chassis_ch2 = CHASSIS_TWIST_SPEED;      
			
			//*Ť���ƶ��㷨����̨Ϊ�������ᣬĿ��ֵ�ֽ⵽����������
			 if(Remote_data.RC_ch1||Remote_data.RC_ch0)
				{ 
					Chassis_ch1=cos(Yaw_different_angle/180*PI)*Remote_data.RC_ch1;//��Ҫ�û��ȵ�λ����
					Chassis_ch0=-sin(Yaw_different_angle/180*PI)*Remote_data.RC_ch1;
					if(Remote_data.RC_ch0)
					{
						Chassis_ch1+=sin(Yaw_different_angle/180*PI)*Remote_data.RC_ch0;
						Chassis_ch0+=cos(Yaw_different_angle/180*PI)*Remote_data.RC_ch0;
					}
				}
				else
				{
					Chassis_ch0 = 0;
					Chassis_ch1 = 0;
				}
		}
	 else if(TWIST_FLAG == 1)//�ҽǶ�С�ڸ�30������ת
	 {
			//*�����̶�Ť�� 
		  if(Chassis_Twist_acc>=-CHASSIS_TWIST_SPEED)
				Chassis_Twist_acc-=20;		 
			Chassis_ch2 = Chassis_Twist_acc;      
//			Chassis_ch2 = -CHASSIS_TWIST_SPEED;      	
			
			//*Ť���ƶ��㷨����̨Ϊ�������ᣬĿ��ֵ�ֽ⵽����������
			 if(Remote_data.RC_ch1||Remote_data.RC_ch0)
				{ 
					Chassis_ch1=cos(Yaw_different_angle/180*PI)*Remote_data.RC_ch1;//��Ҫ�û��ȵ�λ����
					Chassis_ch0=-sin(Yaw_different_angle/180*PI)*Remote_data.RC_ch1;
					if(Remote_data.RC_ch0)
					{
						Chassis_ch1+=sin(Yaw_different_angle/180*PI)*Remote_data.RC_ch0;
						Chassis_ch0+=cos(Yaw_different_angle/180*PI)*Remote_data.RC_ch0;
					}
				}	
				else
				{
					Chassis_ch0 = 0;
					Chassis_ch1 = 0;
				}				
	 }
   else
	 {
		 Chassis_ch0 = 0;
		 Chassis_ch1 = 0;
		 Chassis_ch2 = 0;
	 }		 
		
	/*����ֵ����*/
	Chassis_Input(Chassis_ch0*2, Chassis_ch1*2, Chassis_ch2); 
}


/*=====����С����״̬����======*/
static void Chassis_Rotation(void)
{  
	//*С������ת�ƶ��㷨����̨Ϊ�������ᣬĿ��ֵ�ֽ⵽����������
	 if(Remote_data.RC_ch1||Remote_data.RC_ch0)  //��ʱ���ƶ�
		{ 
			Chassis_ch2 = CHASSIS_ROTATION_MOVE_SPEED;//С�����ƶ�ʱ����
			
			Chassis_ch1=cos(Yaw_different_angle/180*PI)*Remote_data.RC_ch1;  //��Ҫ�û��ȵ�λ����
			Chassis_ch0=-sin(Yaw_different_angle/180*PI)*Remote_data.RC_ch1;
		  if(Remote_data.RC_ch0)
			{
				Chassis_ch1+=sin(Yaw_different_angle/180*PI)*Remote_data.RC_ch0;
				Chassis_ch0+=cos(Yaw_different_angle/180*PI)*Remote_data.RC_ch0;
			}
		}
		else  //ԭ��ת����
		{
	//*�����̶�ת��
	    Chassis_ch2 = CHASSIS_ROTATION_SPEED;//�������Ҹ��� 
			Chassis_ch0 = 0;
			Chassis_ch1 = 0;			
		}
		
	//*����ֵ����	
	Chassis_Input(Chassis_ch0*2, Chassis_ch1*2, Chassis_ch2); 
} 


/*======���̲�����״̬����========*/
static void Chassis_Independent(void) 
{
	//*�Ƶ׷����ƶ��㷨����̨Ϊ�������ᣬĿ��ֵ�ֽ⵽���������ᣨ��С����һ����
	 if(Remote_data.RC_ch1||Remote_data.RC_ch0)
		{ 
			Chassis_ch1=cos(Yaw_different_angle/180*PI)*Remote_data.RC_ch1;//��Ҫ�û��ȵ�λ����
			Chassis_ch0=-sin(Yaw_different_angle/180*PI)*Remote_data.RC_ch1;
		  if(Remote_data.RC_ch0)
			{
				Chassis_ch1+=sin(Yaw_different_angle/180*PI)*Remote_data.RC_ch0;
				Chassis_ch0+=cos(Yaw_different_angle/180*PI)*Remote_data.RC_ch0;
			}
		}
		else
		{
			Chassis_ch0=0;
			Chassis_ch1=0;
		}
		
	//*����ֵ����	
	Chassis_Input(Chassis_ch0, Chassis_ch1, 0);	
}


 

/*====�����������ݴ���====*/
static first_order_low_filter_type_t  Chassis_ForwardBack_LowFilt_Data = {0.0f,0.0f,0.0f,Chassis_Fir_Ord_Low_Fil_Param}; //��ͨǰ�������
static first_order_low_filter_type_t  Chassis_LeftRight_LowFilt_Data   = {0.0f,0.0f,0.0f,Chassis_Fir_Ord_Low_Fil_Param}; //��ͨ���ҿ�����
static int16_t filt_ch0=0,filt_ch1=0;
static int16_t SPEED_GAIN = 0;
int16_t  chassis_ch0_jscope = 0;
int16_t  chassis_ch1_jscope = 0;

static void Chassis_Input(int16_t ch0, int16_t ch1, int16_t ch2)
{	

/*������б�º�������*/	
	Chassis_accelerated_Control(&ch0,&ch1,&ch2);
	
/*������һ�׵�ͨ�˲���������*/
	filt_ch0 = First_Order_Low_Filter(&Chassis_LeftRight_LowFilt_Data   , ch0); //���ҿ�������ͨ�˲�
	filt_ch1 = First_Order_Low_Filter(&Chassis_ForwardBack_LowFilt_Data , ch1); //ǰ���������ͨ�˲�

//	filt_ch0 = ch0;
//	filt_ch1 = ch1;
	
	chassis_ch0_jscope = filt_ch0;
	chassis_ch1_jscope = filt_ch1;
	
/*���������ݵ���ģʽ����*/
	if(workStatus==POWEROFF || workStatus==INITIALIZE ) //�ص���ʼ���׶�
	{
		chassis_input[0] = 0;
		chassis_input[1] = 0;
		chassis_input[2] = 0;
		chassis_input[3] = 0;
	}
	else
	{
		if(chassis_workStatus==FOLLOWING)  //����
		{
			//�����˶�ѧ�ϳ�
			//������ʻ�ٶ�����
			chassis_input[0] = (-(filt_ch1  + ch2 - filt_ch0))*CHASSIS_SPEED_GAIN;
		  chassis_input[1] = (filt_ch1 - ch2 + filt_ch0)   *CHASSIS_SPEED_GAIN;
		  chassis_input[2] = (-(filt_ch1 + ch2 + filt_ch0))*CHASSIS_SPEED_GAIN;
		  chassis_input[3] = (filt_ch1 - ch2 - filt_ch0)   *CHASSIS_SPEED_GAIN;
			
			SPEED_GAIN = CHASSIS_SPEED_GAIN;
		}
		if(chassis_workStatus==ROTATION||chassis_workStatus==TWIST_WAIST||chassis_workStatus==INDEPENDENT)  //С����,Ť�������̲�����
		{
			//�����˶�ѧ�ϳ�
			//����ģʽ��ʻ�ٶ�����
			chassis_input[0] = (-(filt_ch1 + ch2 - filt_ch0))*TWIST_SPEED_GAIN;
		  chassis_input[1] = (filt_ch1 - ch2 + filt_ch0)   *TWIST_SPEED_GAIN;
		  chassis_input[2] = (-(filt_ch1 + ch2 + filt_ch0))*TWIST_SPEED_GAIN;
		  chassis_input[3] = (filt_ch1 - ch2 - filt_ch0)   *TWIST_SPEED_GAIN;
			
			SPEED_GAIN = TWIST_SPEED_GAIN;
		}
		if(chassis_workStatus==STOP)       //ֹͣ
		{
		  chassis_input[0] = 0;
		  chassis_input[1] = 0;
		  chassis_input[2] = 0;
		  chassis_input[3] = 0;			
		}
	}

	
/*���PID�ٶȱջ�����*/
	chassis_output[0]=Rmmotor_Speed_control(&CHASSIS_1_S_PID,chassis_input[0],Chassis_Motor[0].speed,M3508_MAX_OUTPUT_CURRENT);
	chassis_output[1]=Rmmotor_Speed_control(&CHASSIS_2_S_PID,chassis_input[1],Chassis_Motor[1].speed,M3508_MAX_OUTPUT_CURRENT);
	chassis_output[2]=Rmmotor_Speed_control(&CHASSIS_3_S_PID,chassis_input[2],Chassis_Motor[2].speed,M3508_MAX_OUTPUT_CURRENT);
	chassis_output[3]=Rmmotor_Speed_control(&CHASSIS_4_S_PID,chassis_input[3],Chassis_Motor[3].speed,M3508_MAX_OUTPUT_CURRENT);

	
/*�����������Ƶ��(test)*/
//	if(Chassis_Motor[0].speed>=0)
//	  Motor_power[2] = POWER_Data.Motor_3;
//  else
//	  Motor_power[2] = -POWER_Data.Motor_3;
//	
//	if(Chassis_Motor[1].speed>=0)
//	  Motor_power[0] = POWER_Data.Motor_1;
//  else
//	  Motor_power[0] = -POWER_Data.Motor_1;	
//	
//	if(Chassis_Motor[2].speed>=0)
//	  Motor_power[3] = POWER_Data.Motor_4;
//  else
//	  Motor_power[3] = -POWER_Data.Motor_4;	
//	
//	if(Chassis_Motor[3].speed>=0)
//	  Motor_power[1] = POWER_Data.Motor_2;
//  else
//	  Motor_power[1] = -POWER_Data.Motor_2;	
//	
//	chassis_output[0]=Rmmotor_Speed_control(&CHASSIS_1_P_PID,chassis_input[0], Motor_power[2]*1000,900);
//	chassis_output[1]=Rmmotor_Speed_control(&CHASSIS_2_P_PID,chassis_input[1], Motor_power[0]*1000,900);
//	chassis_output[2]=Rmmotor_Speed_control(&CHASSIS_3_P_PID,chassis_input[2], Motor_power[3]*1000,900);
//	chassis_output[3]=Rmmotor_Speed_control(&CHASSIS_4_P_PID,chassis_input[3], Motor_power[1]*1000,900);

	
/*���²���ϵͳ�������ݣ����ݺ͹��޶�Ҫ�õ���*/	
	Power_Real();         
	 
/*�������ݿ����߼�*/
#ifdef super_capacitor
  Capacitor_control_chassis(&Chassis_PowerLimit , 0 , chassis_output[0], chassis_output[1], chassis_output[2], chassis_output[3]);
#endif

/*���̹������ƴ���*/
#ifdef power_limit
{
	if(Chassis_PowerLimit.RemainPower[2]<PowerLimit_Thres && Chassis_PowerLimit.RemainPower[2]>5)  //����0��ֹ����ϵͳ���ݴ����
	{
    Chassis_PowerLimit.SumOutValue   = abs(chassis_output[0])+ abs(chassis_output[1])+ abs(chassis_output[2])+ abs(chassis_output[3]); //������������Ժ�
    Chassis_PowerLimit.LimitOutValue = Chassis_PowerLimit.RemainPower[2] * Chassis_PowerLimit.RemainPower[2] * PowerLimit_Param ;      //���幦��ƽ���ͳ˹�������ϵ��
		//�����ֵ���б�������
		chassis_output[0] = Chassis_PowerLimit.LimitOutValue * chassis_output[0] / Chassis_PowerLimit.SumOutValue;
		chassis_output[1] = Chassis_PowerLimit.LimitOutValue * chassis_output[1] / Chassis_PowerLimit.SumOutValue;
		chassis_output[2] = Chassis_PowerLimit.LimitOutValue * chassis_output[2] / Chassis_PowerLimit.SumOutValue;
		chassis_output[3] = Chassis_PowerLimit.LimitOutValue * chassis_output[3] / Chassis_PowerLimit.SumOutValue;
	}
	else if(Chassis_PowerLimit.RemainPower[2] <=5)// && Chassis_PowerLimit.RemainPower[2]!=0
  {
		chassis_output[0] = 0;
		chassis_output[1] = 0;
		chassis_output[2] = 0;
		chassis_output[3] = 0;		
	} 
}

/*��ֹ�����˶��켣ʧ�洦��*/	
	Keep_Driv_Track(&chassis_output[0] ,&chassis_output[1] ,&chassis_output[2] ,&chassis_output[3] ,14);

//{
//  PowerLimitLoop();
//	chassis_output[0]*=Chassis_PowerLimit.scale;
//	chassis_output[1]*=Chassis_PowerLimit.scale;
//	chassis_output[2]*=Chassis_PowerLimit.scale;
//	chassis_output[3]*=Chassis_PowerLimit.scale;
//}

#endif
 
	
/*��������͵����*/
#ifdef board_chassis  //ֻ�е��̰��������̹������ܷ���can

 #ifdef chassis_work
    if(INITIALIZE_flag==1)  //û�㶮Ϊɶ��ʼ��ʱ��������̵��ĸ���������ݣ��ٸ�y�ᵥ����ʱ�ᷢ����ȥ�����³�ʼ��ʱy�᲻ת��������������жϵ���û��ʼ�����ʱ���������̵��������
      CAN1_SendCommand_Chassis(chassis_output[0],chassis_output[1],chassis_output[2],chassis_output[3]);
 #endif

#endif
	
}


/*==���̼��ٶ�����б�º���==*/
static void Chassis_accelerated_Control(int16_t *ch0,int16_t *ch1,int16_t *ch2)				
{
	static int16_t last_ch[3] = {0,0,0};
	int16_t temp[3];
	
	temp[0] = *ch0 - last_ch[0];
	temp[1] = *ch1 - last_ch[1];
	temp[2] = *ch2 - last_ch[2];
	
	if(RC_Ctl.rc.s2 == RC_CONTROL)													//ң��ģʽ
	{
		if(abs_float(temp[0]) > TRANSLATION_ACCELERAD)
			*ch0 = last_ch[0] + temp[0]/abs_float(temp[0])*TRANSLATION_ACCELERAD;
		if(abs_float(temp[1]) > STRAIGHT_ACCELERAD)
			*ch1 = last_ch[1] + temp[1]/abs_float(temp[1])*STRAIGHT_ACCELERAD;
		
		if(chassis_workStatus==TWIST_WAIST||chassis_workStatus==ROTATION)         //Ť��ģʽ�²�����ת���ٶ�����
		{
			if(abs_float(temp[2]) > ROTATING_ACCELERAD)
				*ch2 = last_ch[2] + temp[2]/abs_float(temp[2])*ROTATING_ACCELERAD;
	  }
	}
	if(RC_Ctl.rc.s2 == MOUSE_CONTROL)												//����ģʽ
	{
		if(abs_float(temp[0]) > TRANSLATION_ACCELERAD)
			*ch0 = last_ch[0] + temp[0]/abs_float(temp[0])*TRANSLATION_ACCELERAD;
		if(abs_float(temp[1]) > STRAIGHT_ACCELERAD)
			*ch1 = last_ch[1] + temp[1]/abs_float(temp[1])*STRAIGHT_ACCELERAD;
		
		if(chassis_workStatus==TWIST_WAIST)                   //Ť��ģʽ�²�����ת���ٶ�����
		{
			if(abs_float(temp[2]) > ROTATING_ACCELERAD)
				*ch2 = last_ch[2] + temp[2]/abs_float(temp[2])*ROTATING_ACCELERAD;
	  }
	}
	last_ch[0] = *ch0;	
	last_ch[1] = *ch1;
	last_ch[2] = *ch2;
	
}



/*
*���ܣ����̷�ֹ�˶��켣ʧ��
*���룺�����ĸ��������ָ��
*�������������ĸ����
*/
static void Keep_Driv_Track(int16_t *v0 ,int16_t *v1 ,int16_t *v2 ,int16_t *v3 ,int16_t SPEED_GAIN)						
{
	static int16_t max_v=0;
	static float scale=1.0f;
	
	max_v = max_abs(*v0,max_abs(*v1,max_abs(*v2,*v3)));		// ȡ�ٶ���ֵ��������
	if(SPEED_GAIN == 0)
	{ *v0 = *v1 = *v2 = *v3 = 0;}
	else if(max_v > (SPEED_GAIN * 660) )
	{
		scale = max_v/(SPEED_GAIN * 660);
		*v0 = (int16_t)((*v0)/scale);
		*v1 = (int16_t)((*v1)/scale);
		*v2 = (int16_t)((*v2)/scale);
		*v3 = (int16_t)((*v3)/scale);
	}
}



/*****************************************************************���̹�������**************************************************************************************/
/*======����������ز�����ʼ��=======*/
static void Power_Init(void)
{
	Chassis_PowerLimit.Real_Power[2]=REFEREE.PowerHeat.chassis_power;    //ʵʱ����
	Chassis_PowerLimit.RemainPower[2]=REFEREE.PowerHeat.chassis_power_buffer; //���ʻ���
	
	Chassis_PowerLimit.Real_Power[1]=Chassis_PowerLimit.Real_Power[2];
	Chassis_PowerLimit.Real_Power[0]=Chassis_PowerLimit.Real_Power[1];
	
	Chassis_PowerLimit.RemainPower[1]=Chassis_PowerLimit.RemainPower[2];
	Chassis_PowerLimit.RemainPower[0]=Chassis_PowerLimit.RemainPower[1];
	
	REFEREE.PowerHeat.error=0;
	Chassis_PowerLimit.RemainPower[2] = Chassis_PowerLimit.RemainPower[1] = Chassis_PowerLimit.RemainPower[0] = 60; //���幦��
}

/*======��ֹ����ϵͳʧ��=======*/
static void Power_Real(void)
{
	if(REFEREE.PowerHeat.error!=1||Chassis_PowerLimit.RemainPower[2]>20)//��֤Ƶ��Ϊ50Hz
	{
		Chassis_PowerLimit.Real_Power[2]=REFEREE.PowerHeat.chassis_power;
		Chassis_PowerLimit.RemainPower[2]=REFEREE.PowerHeat.chassis_power_buffer;
	}
	else 
	{
		Chassis_PowerLimit.Real_Power[2]=(2*Chassis_PowerLimit.Real_Power[1]-Chassis_PowerLimit.Real_Power[0])*1.3f;//����֡��ʧ��ͨ��΢�ֹ�ϵԤ���ʱ���ʣ�Ϊ�˱�֤Ԥ�⹦��С��ʵ�ʹ��ʣ�ȡ��ȫϵ��1.3
		Chassis_PowerLimit.RemainPower[2]=(2*Chassis_PowerLimit.RemainPower[1]-Chassis_PowerLimit.RemainPower[0])/1.3f;
	}
	//���ݵ���
	Chassis_PowerLimit.Real_Power[1]=Chassis_PowerLimit.Real_Power[2];
	Chassis_PowerLimit.Real_Power[0]=Chassis_PowerLimit.Real_Power[1];
	
	Chassis_PowerLimit.RemainPower[1]=Chassis_PowerLimit.RemainPower[2];
	Chassis_PowerLimit.RemainPower[0]=Chassis_PowerLimit.RemainPower[1];
}

/*============================2019�����ɹ������ƣ�test��===============*/
static void PowerLimitLoop(void)
{
	float Power_p_error=0;
	
	Power_Real();//��ֹ��������ʧ�洦��
	
	if(Chassis_PowerLimit.RemainPower[2]>=50)  //���幦���㹻
	{
		Chassis_PowerLimit.scale = 1;
	}

	else if(Chassis_PowerLimit.RemainPower[2]>10&&Chassis_PowerLimit.RemainPower[2]<50)  //���幦�ʼ��٣��ٶȽ���
	{
		Power_p_error = 50 - Chassis_PowerLimit.RemainPower[2];
		Chassis_PowerLimit.scale = 1 - (float)(Power_p_error/40);	
	}
	else if(Chassis_PowerLimit.RemainPower[2]<=10||Chassis_PowerLimit.RemainPower[2]>0)  //���幦�ʲ�����ֱ�ӵ��̵�����Ϊ0
	{
		Chassis_PowerLimit.scale=0; 
	}
	else if(Chassis_PowerLimit.RemainPower[2]==0)//��ⲻ������ϵͳ���ǿ��Ե�����
	{
	  Chassis_PowerLimit.scale=0.7;       
	}

}


