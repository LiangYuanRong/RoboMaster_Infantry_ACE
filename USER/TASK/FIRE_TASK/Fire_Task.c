/*==================================================*
*�ļ����ܣ�1.������������ϵͳ������
*         2.����ϵͳ�Ŀ��ƺͷ���Ħ���ֵĿ���
*         3.�������ƣ����ȼ���
*             
*��ע��ԓ�ļ���û�������ϵͳ����������
*
*����ģʽ��1.������ģʽ�����õ������������ȼ��е������������Ħ���֣�����ȷ�ȸߡ���糵ʱ����Զ��ʱ�õ���
          2.������ģʽ��Ĭ���������������15m/s�ٶȣ���֤����Ƶ������ȷ�ȵ͡�2.5m��Χ�ڽ�ս�ã�
					3.��Ѫģʽ  ���ر��������ƣ���Ѫ����ȡ�������������Ƶ���п��ٴ���������������������ã�ע�����ٲ�Ҫ���ߣ���Ȼ������������
				
*==================================================*/

#include "main.h"
#include "Fire_Task.h"
#include "Speed_Def.h"
#include "PID_Def.h"
#include "Mode_Def.h"
#include "Remote_Task.h"



//================��������
int fire_heart=0;  //�����������

//================�ṹ�嶨��
Fire_WorkStatus     fire_workstatus    = STOP_FIRE;      //�䵯ģʽĬ��Ϊͣ��
Shoot_WorkStatus    shoot_workstatus   = LOW_SPEED;      //Ħ����ģʽ����ģʽ (û�������¹���������������޹أ�ֱ�Ӱ�����������)
Fire_task_t Fire_param; 
extern REFEREE_t REFEREE;             //����ϵͳ����
extern TaskHandle_t FireTask_Handler; // ջ��С

//REFEREE.RobotStatus.shooter_heat0_cooling_limit - REFEREE.PowerHeat.shooter_heat0  //��ǰ����������ȥʣ��������cooling_rate ��ȴ�ٶȣ�
//REFEREE.RobotStatus.shooter_heat0_speed_limit   //��ǰ��������

//================�ڲ���������
static void Fire_Control(void);    //����ϵͳ����
static void Fire_param_init(void); //������ʼ��

 
/**�������**/
void Fire_Task(void *pvParameters)
{
	//����ʱ��
  vTaskDelay(FIRE_TASK_INIT_TIME);
	
	while(1)
	{
		fire_heart=!fire_heart;           //�����������
		LEDE3 = fire_heart;
		 
		Fire_param_init();
		Fire_Control();
		
		//��������
    vTaskDelay(FIRE_CONTROL_TIME_MS);
	}
}




/**������ʼ��**/
static void Fire_param_init(void)
{
	Fire_param.GD_output=0;
	Fire_param.shoot_speed=0;
}


/**����ϵͳ����**/
int16_t fire_count=0;
static void Fire_Control(void)
{
	if(workStatus==POWEROFF || workStatus==INITIALIZE)   //��ʼ�������״̬ʱĦ����ֹͣ
	{
		PWM_Shoot_Left = PWM_Shoot_Right = SHOOT_STOP;
		CAN1_SendCommand_Gimbal_Fire(0,0,0);			
	}
	else                                                  //�������״̬��
	{		    					
		/*�������*/
		if(shoot_workstatus != STOP_SHOOT)                  //Ħ������������������������
		{
			/*=========����Ħ�����ٶ��趨===========*/
//			if(shoot_workstatus == LOW_SPEED)
//			{
//				if(fire_count<=300)   //��ʱ�ӵ�һ����ʱ����
//				{
//					PWM_Shoot_Left = ZERO_SHOOT_SPEED;
//					PWM_Shoot_Right = SHOOT_STOP;
//				  fire_count++; 
//				}
//				else if(fire_count>300)
//				{
//				  PWM_Shoot_Left = PWM_Shoot_Right = ZERO_SHOOT_SPEED;
//				}
//			}
			if(shoot_workstatus == LOW_SPEED)
			{
				//��������ٵȼ��仯
				if(REFEREE.RobotStatus.shooter_heat0_speed_limit==15) //�㼶
				{
					PWM_Shoot_Left = PWM_Shoot_Right = ZERO_SHOOT_SPEED;								
				}	
				if(REFEREE.RobotStatus.shooter_heat0_speed_limit==18) //һ��
				{
					PWM_Shoot_Left = PWM_Shoot_Right = ONE_SHOOT_SPEED;								
				}					
				if(REFEREE.RobotStatus.shooter_heat0_speed_limit==22) //����
				{
					PWM_Shoot_Left = PWM_Shoot_Right = TWO_SHOOT_SPEED;								
				}					
				if(REFEREE.RobotStatus.shooter_heat0_speed_limit==30) //����
				{
					PWM_Shoot_Left = PWM_Shoot_Right = THREE_SHOOT_SPEED;								
				}									
			}			
			
			/*=========�����������============*/
			if(fire_workstatus == STOP_FIRE)
			{
				Fire_param.GD_output = Rmmotor_Speed_control(&FIRE_S_PID ,0 ,Fire_GD_Motor.speed,M2006_MAX_OUTPUT_CURRENT);
			}
			else if(fire_workstatus == FIRE)
			{
				//��������
				Fire_param.GD_output = Rmmotor_Speed_control(&FIRE_S_PID ,-LOADING_SPEED ,Fire_GD_Motor.speed,M2006_MAX_OUTPUT_CURRENT);				
			}
			else if(fire_workstatus == BACK)
			{
        Fire_param.GD_output = Rmmotor_Speed_control(&FIRE_S_PID ,LOADING_SPEED ,Fire_GD_Motor.speed,M2006_MAX_OUTPUT_CURRENT);							
			}						
		}
		else                                                 //�ֶ��رշ���Ħ����
		{
		  PWM_Shoot_Left = PWM_Shoot_Right = SHOOT_STOP;
      Fire_param.GD_output = 0;	
      fire_count = 0;			
		}
	}
	
	  
#ifdef fire_work          //�����˻�ع���
	
	#ifdef board_chassis   //����ǵ��̰壬ֻ�������can�ź�  ����Ϊ���ͳ�ͻԭ��can1��������̨��������̰���һ�𷢣�
 				PWM_Shoot_Left = PWM_Shoot_Right = SHOOT_STOP;
	#endif
#else                     //�������ع�����Ħ����ֹͣ
			PWM_Shoot_Left = PWM_Shoot_Right = SHOOT_STOP;
	    fire_workstatus = STOP_FIRE;
#endif
	
}

