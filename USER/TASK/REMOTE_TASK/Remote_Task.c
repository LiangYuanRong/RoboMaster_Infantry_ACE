#include "main.h"
#include "maths.h"
#include "Remote_Task.h"
#include "remote.h"
#include "Speed_Def.h"
#include "Mode_Def.h"
#include "Gimbal_Task.h"
#include "Fire_Task.h"
#include "Chassis_Task.h"
#include "Capacitor_control.h"

/*****************************������λ˵����**********************************************************/
/* һ��ң��ģʽ��
         1.���̸���  ����������
				 2.Ť��ģʽ  ����������
				 3.����С���ݣ���������
				 4.���ģʽ  ���������У���������̨�������ƶ��������棩
				 5.����ģʽ  ����������
				 6.����ģʽ  ����������
		     7.����      ���ڳ�ʼ��������ҿ��ش��ϻ�������£����Ͻǲ��������Ϸ���������ֹͣ�䵯
				 8.�ػ�      ������				 
	 ��������ģʽ��
	       1.�����˶���WASD
				 2.��̨�˶������
				 3.���䣺    ���������㵥������ס����
				 4.���٣����ݷŵ磩��    ��סshift����Ϻ����
				 5.Ť��ģʽ�� F  ����һ�½��룬�ٰ�һ�·��أ�
				 6.����ģʽ�� ����Ҽ�  ����ס��
				 7.����ģʽ��  G  ����һ����̨��һ��ת�����굯�ٰ�һ�»�����
				 8.������ģʽ��C ����һ�ν��룩�����ģʽû���ˣ�֮ǰȫ��������ٴ�
				 9.������ģʽ��V ����һ�ν��룩
				 10.�˵�ģʽ�� Z  ����ס��
				 11.��̨ģʽ�� Ctrl ����ס��ֻ�ܿ�����̨�����̲�����
				 12.���ģʽ�� X ��һ�ν��룩
				 13.С����ģʽ��R����һ�ν��룩
                                                                                                    */
/****************************************************************************************************/
/*
   ���ּ�д��RC��remote contral   ң����
            MK: mouse key        ����
*/


/*ң���������Ĵ���ֵ*/
RC_Deal_data        Remote_data;         
/*��ջʣ��*/
extern TaskHandle_t RemoteTask_Handler; // ջ��С


/*
*���ܣ�ң������
*/
int mode_heart=0;    //ң����������
//int RemoteTask_water=0;

void Remote_Task(void *pvParameters)
{
	//����ʱ��
    vTaskDelay(REMOTE_TASK_INIT_TIME);
	
	while(1)
	{
		mode_heart=!mode_heart;       //ң����������
		LEDE0 = mode_heart;           //A�������
//		RemoteTask_water = uxTaskGetStackHighWaterMark(RemoteTask_Handler);//���ʣ�������ջ
		
		Select_Ctl_Mode();            //ң�����ݴ���
		//��������
    vTaskDelay(REMOTE_CONTROL_TIME_MS);
	}
}


	
		


/*
*���ܣ�ң��ѡ��ģʽ����
*/

static u8 control_mode=0;                  //�ñ���Ϊ��¼�ô�ң��ģʽ״̬��s2��
extern int8_t INITIALIZE_flag;             //���͸����̵ĳ�ʼ���ɹ���־
extern int gimbal_work_mode;               //��̨״̬�л������������־λ

void Select_Ctl_Mode()
{
	if(RC_Ctl.rc.s2 == RC_CONTROL)             //�ҿ��ش��ϣ�ң�ؿ���
	{
				 if(control_mode==2)                     //��һ��״̬Ϊs2���У����»�����״̬
				{ 
					chassis_workStatus = FOLLOWING;        //����Ĭ�Ϲ���ģʽΪ����
					Remote_reload();                       //ҡ��������			
				}	
				
				if(workStatus == POWEROFF)//֮ǰ��״̬Ϊ����
				{
					workStatus = INITIALIZE;               //״̬����Ϊ��ʼ��				
					Remote_reload();                       //ҡ��������
				}
				else if(workStatus == AUTOATTACK || workStatus == AUTOBUFF)//֮ǰ״̬Ϊ�Զ��������ģʽ
				{
					workStatus=WORKING;     //״̬����Ϊ����
				}
				else
				{
						if(workStatus != INITIALIZE)//״̬��Ϊ��ʼ��
						{
							RC_Data_Process();     //ң�������ݴ���
                  
						 /*�����л�ģʽ*/
									if(RC_Ctl.rc.s1 == RC_SW_UP)//s1�����ϣ�����
									{
										chassis_workStatus=ROTATION;//����С����ģʽ   ROTATION
									}
									if(RC_Ctl.rc.s1 == RC_SW_MID)//s1�����ϣ����м�       //ƽ��ң��״̬
									{
										chassis_workStatus=FOLLOWING;//���̸���ģʽ					
									}
									if(RC_Ctl.rc.s1 == RC_SW_DOWN)//s1�����ϣ�������
									{
										chassis_workStatus=TWIST_WAIST;//����Ť��ģʽ   TWIST_WAIST
									}
									
						/*����״̬*/
									/*���ֿ����䵯�����Ǹ����٣����ǵ����٣��м���ֹͣ����*/
									if(Remote_data.RC_sw >= 500)
									{
										fire_workstatus = FIRE;              //������  ����
										shoot_workstatus = HIGH_SPEED;       //Ħ����ģʽ����ģʽ 			
									} 
									if(Remote_data.RC_sw <= -500)
									{
										fire_workstatus = FIRE;              //������  ����	
										shoot_workstatus = LOW_SPEED;        //Ħ����ģʽ����ģʽ 				
									}
									else
									{
										fire_workstatus = STOP_FIRE;         //�����м�  ֹͣ����
										shoot_workstatus = LOW_SPEED;        //Ħ����ģʽ����ģʽ 							
									}	
									
							
						}
				}
				control_mode=1;         //s2��1		
	}
	if(RC_Ctl.rc.s2 == MOUSE_CONTROL)  //�ҿ��ش��У����̿���
	{
				 if(control_mode==1||control_mode==0)  //��һ��״̬Ϊs2����,���»�����״̬
				{
					if(control_mode==0)
							workStatus = POWEROFF;               //Ĭ�Ϲ���ģʽΪ����ģʽ
					chassis_workStatus = FOLLOWING;          //����Ĭ�Ϲ���ģʽΪ����
					gimbal_work_mode=0;                  
					
					Remote_reload();                         //ҡ��������
				}
				
				if(workStatus == POWEROFF)    //֮ǰ��״̬Ϊ�ص�
				{
					workStatus = INITIALIZE;    //״̬����Ϊ��ʼ��
					Remote_reload();            //ҡ��������					 
				}
				else
				{
							if(workStatus != INITIALIZE)//״̬��Ϊ��ʼ��
							{
									if(RC_Ctl.rc.s1 == 1)      //��������  ���ģʽ
									{ 
										RC_Data_Process();       //ң�������ݴ���
										workStatus = AUTOBUFF;   //���ģʽ 
										chassis_workStatus = INDEPENDENT; //���̲�����	
									
									}
									if(RC_Ctl.rc.s1 == 2)     //��������  �Զ����
									{
										RC_Data_Process();       //ң�������ݴ���					
										workStatus = AUTOATTACK; //�Զ����ģʽ
									
										/*����״̬*/
												/*���ֿ����䵯�����Ǹ����٣����ǵ����٣��м���ֹͣ����*/
												if(Remote_data.RC_sw >= 500)
												{
													fire_workstatus = FIRE;              //������  ����
													shoot_workstatus = HIGH_SPEED;       //Ħ����ģʽ����ģʽ 			
												} 
												if(Remote_data.RC_sw <= -500)
												{
													fire_workstatus = FIRE;              //������  ����	
													shoot_workstatus = LOW_SPEED;        //Ħ����ģʽ����ģʽ 				
												}
												else
												{
													fire_workstatus = STOP_FIRE;         //�����м�  ֹͣ����
													shoot_workstatus = LOW_SPEED;        //Ħ����ģʽ����ģʽ 							
												}	
									
									}
									if(RC_Ctl.rc.s1 == 3)      //��������   ����ģʽ
									{
										MK_Data_Process();       //�������
									}
												
							}
				}
				control_mode=2;          //s2��2
	}
	if(RC_Ctl.rc.s2 == ALL_STOP)       //�ҿ��ش��£�ֹͣ����
	{
		workStatus = POWEROFF;   //�������ģʽ
		INITIALIZE_flag = 0;     //��ʼ����־λ����
		control_mode=0;          //s2��0
		Remote_reload();         //ҡ��������
		
	}
	if(RC_Ctl.rc.s2 == 0)
	{
		workStatus = POWEROFF;   //�������ģʽ
		INITIALIZE_flag = 0;     //��ʼ����־λ����
		Remote_reload();         //ҡ��������
		
	}
	
}


/*
*���ܣ�ң�������ݴ���
*ȡֵ����Դ��remote�ļ����ң��ֵ��ȡ
*/
void RC_Data_Process(void)    
{
	/*ң�����˲�����*/
	
	//ң������ȡ��ֵ��364��1684֮�䣬�������Ϊ350��1700
	if((RC_Ctl.rc.ch0<350||RC_Ctl.rc.ch0>1700)||(RC_Ctl.rc.ch1<350||RC_Ctl.rc.ch1>1700)||(RC_Ctl.rc.ch2<350||RC_Ctl.rc.ch2>1700)||(RC_Ctl.rc.ch3<350||RC_Ctl.rc.ch3>1700))
	{
		RC_Ctl.rc.ch0 = RC_Ctl.rc.ch1 = RC_Ctl.rc.ch2 = RC_Ctl.rc.ch3 =1024;
	}
	//����ֵΪ10
		if(abs(RC_Ctl.rc.ch0 - 1024)<10)
			RC_Ctl.rc.ch0=1024;
		if(abs(RC_Ctl.rc.ch1 - 1024)<10)
			RC_Ctl.rc.ch1=1024;
		if(abs(RC_Ctl.rc.ch2 - 1024)<10)
			RC_Ctl.rc.ch2=1024;
		if(abs(RC_Ctl.rc.ch3 - 1024)<10)
			RC_Ctl.rc.ch3=1024;
	
	/*��ң����ֵ����ң����*/
		if(workStatus == WORKING || workStatus == AUTOATTACK || workStatus == AUTOBUFF)  //�ֶ�����
		{
			Remote_data.RC_ch0 = (RC_Ctl.rc.ch0 - 1024);
			Remote_data.RC_ch1 = (RC_Ctl.rc.ch1 - 1024); 
			Remote_data.RC_ch2 += (RC_Ctl.rc.ch2 - 1024)*RC_YAW_SPEED;   //Y��λ�û����ۼ�
			   Remote_data.RC_ch2 = loop_fp32_constrain(Remote_data.RC_ch2 , -180.0f , 180.0f);//ѭ���޷�
			Remote_data.RC_ch3 +=(RC_Ctl.rc.ch3 - 1024)*RC_PITCH_SPEED;   //P��λ�û����ۼ�
			Remote_data.RC_sw  = -(RC_Ctl.rc.sw - 1024);     //ң��������ֵ����ֵΪ1024������������ࡣ�������ֵΪ0
		}

		else  //����ģʽ����ʼ��ģʽ������ģʽ  �������ֶ�����
		{
			Remote_reload();                       //ҡ��������  
		}
	
}


/*
*���ܣ��������ݴ���
*ȡֵ����Դ��remote�ļ����ң��ֵ��ȡ
*       &RC_Ctl.mouse    ���
*       &RC_Ctl.key      ����
*/
Key_Press_Sigh Key_Press = {0};       //���̰������¼�¼

extern int8_t GIMBAL_SUPPLY_FLAG;     //����״̬��־λ(0:����״̬�����벹��״̬  1:��̨90��ת����  2:����ָ��λ�ã�������)
		   int8_t CHASSIS_SUPPLY_FLAG = 0;//���ڵ��̲���״̬ģʽ�л�
int8_t  Vision_flag = 0;              //�Ӿ�ģʽ�л���־��0���رգ�1�����飬2���������أ�
static int8_t  Shoot_flag = 0;        //������״̬�л���־
static int8_t  Independent_flag=0;    //���̶����л���־

static int16_t Speed_Straight_ACC = 0;       //ǰ���ƶ�����ģ��б�¼���
static int16_t Speed_Across_ACC = 0;         //����ƽ�Ƽ���ģ��б�¼���

void MK_Data_Process(void)
{
/**********����ģʽѡ��**********/
	
	//**F  Ť��ģʽ
		if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_F)     //F  Ť��ģʽ
		{
				if(Key_Press.KEY_F == 0)
				{
					if(chassis_workStatus == FOLLOWING)
					{	
					  chassis_workStatus=TWIST_WAIST;      //����Ť��ģʽ
					}
					else
					{
						chassis_workStatus = FOLLOWING;
					}
					Key_Press.KEY_F = 1;
				}
		}
		else
		{
			Key_Press.KEY_F = 0;			
		}
		
		


		//**R  С����ģʽ
		if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_R)     //R  С����ģʽ
		{
				if(Key_Press.KEY_R == 0)
				{
					if(chassis_workStatus == FOLLOWING)
					{	
						chassis_workStatus = ROTATION;
					}
					else
					{
						chassis_workStatus = FOLLOWING;
					}
					Key_Press.KEY_R = 1;
				}
		} 
		else
		{
			Key_Press.KEY_R = 0;			
		}



		//**Ctrl������rap����   ��̨ģʽ
		if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_CTRL) //Ctrl  ��̨ģʽ����ס��
	  {
			Independent_flag = 1;
			chassis_workStatus = INDEPENDENT;
	  }
		else if(Independent_flag==1)
		{
			Independent_flag=0;
			chassis_workStatus = FOLLOWING;
		}
	 
		

		//**G  ����ģʽ
		
#ifdef board_gimbal//��̨�岹���߼�		
		if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_G)    //G  ����ģʽ
		{
				if(Key_Press.KEY_G == 0)            //����
				{
					if(workStatus != REPLENISHMEN)  //��������
					{
						workStatus=REPLENISHMEN;
						chassis_workStatus=STOP; 
					}
					else if(workStatus == REPLENISHMEN && GIMBAL_SUPPLY_FLAG==2) //�˳����������ڲ���ģʽ�����ѵ���Ŀ��Ƕȣ�
					{
             GIMBAL_SUPPLY_FLAG = 3;
					}
					Key_Press.KEY_G = 1;
				}
		}
		else
		{
			if(GIMBAL_SUPPLY_FLAG == 4)
			{
				GIMBAL_SUPPLY_FLAG = 0;   //�ɿ������ظ�ԭ״̬
				workStatus = WORKING;
				chassis_workStatus = FOLLOWING;
			}
			Key_Press.KEY_G = 0;			
		}
		
#else //���̰岹���߼�
   	if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_G)    //G  ����ģʽ
		{
				if(Key_Press.KEY_G == 0)            //����
				{
						chassis_workStatus=STOP; 
						CHASSIS_SUPPLY_FLAG = 1;
						GIMBAL_SUPPLY_FLAG = 1;
			  		Key_Press.KEY_G = 1;
				}
		}
		else
		{
			if(CHASSIS_SUPPLY_FLAG==1 && (GIMBAL_SUPPLY_FLAG == 4 || GIMBAL_SUPPLY_FLAG == 0))  //��Ϊ����Ƶ��ԭ����ʱ��̨δ���ü�����4�ı�־λ�ͱ����0�������������Լ����ж�
			{
				GIMBAL_SUPPLY_FLAG = 0;   //��̨�����������
				CHASSIS_SUPPLY_FLAG = 0;  //���̽������״̬
				chassis_workStatus = FOLLOWING;
			}
			Key_Press.KEY_G = 0;			
		}	
#endif
		
	
		
	 //**C  ������ģʽ
		if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_C)    //C		������ģʽ
		{
				if(Key_Press.KEY_C == 0)
			{
          if(shoot_workstatus == LOW_SPEED)
					{
						shoot_workstatus=HIGH_SPEED;
					}
					Key_Press.KEY_C = 1;
				}
		}
		else
		{
			Key_Press.KEY_C  = 0;
		}		
	
		
	 //**V  ������ģʽ
		if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_V)    //V		������ģʽ
		{
				if(Key_Press.KEY_V == 0)
			{
          if(shoot_workstatus == HIGH_SPEED)
					{
						shoot_workstatus=LOW_SPEED;
					}
					Key_Press.KEY_V = 1;
				}
		}
		else
		{
			Key_Press.KEY_V  = 0;
		}	
	
		

		
	//**X  ���ģʽ
#ifdef board_gimbal	//��̨����ģʽ
		if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_X)    
		{
				if(Key_Press.KEY_X == 0)
				{
					if(workStatus != AUTOBUFF)
					{
						workStatus = AUTOBUFF;
					}
					else
					{
						workStatus = WORKING;  //���ģʽ�����������������
					}
					Key_Press.KEY_X = 1;
				}
		}
		else
		{
			Key_Press.KEY_X = 0;			
		}
#else  //���̰���ģʽ
   	if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_X)    
		{        
			if(Key_Press.KEY_X == 0)
			{
				 if(Vision_flag==0)
				 {
						chassis_workStatus = STOP; 
						Vision_flag = 2;
				 }
				 else if(Vision_flag==2)
				 {
					 	Vision_flag = 0;
	     			chassis_workStatus = FOLLOWING;
				 }					 
			}
			Key_Press.KEY_X = 1;
		}
		else
		{
			Key_Press.KEY_X = 0;			
		}		
#endif 
		
		
	//**����Ҽ�   �Զ����	
		if(RC_Ctl.mouse.press_r==1)                 //����Ҽ�  �Զ��������ס��
		{
			Vision_flag = 1;            
			workStatus=AUTOATTACK;
		}
		else if(Vision_flag==1)
		{
			Vision_flag = 0;
			workStatus=WORKING;
		}	

		
		
	//**������   ����	
		if(RC_Ctl.mouse.press_l==1)                  //������  ���𣨰�ס��
		{
			Shoot_flag = 1;
			fire_workstatus=FIRE;
		}
		else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_Z) //Z   �˵�ģʽ(�̰����ɣ��������յ��)
		{
			Shoot_flag = 1; 
			fire_workstatus = BACK;			
		}
		else if(Shoot_flag==1) 
		{
			Shoot_flag = 0;
			fire_workstatus=STOP_FIRE;
		}	

	//**��shift    ʹ�ó������
  			if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT)   //Shift    ���ݷŵ� 	 
				{  		
  			   	super_cap_data.Cap_Out_Flag=1;	  //�ŵ��־1
            super_cap_data.Cap_Charge_Flag=0;	//����־0		
				}
				else
				{
						super_cap_data.Cap_Out_Flag=0;    //�ŵ��־0
					  super_cap_data.Cap_Charge_Flag=1; //����־1
				}

 /**********ǰ�������ƶ�**********/
    //ǰ��� 
		if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_W)           //W  ǰ��
		 {
				if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT)   //Shift    �����ƶ�   	 
				{
					Remote_data.RC_ch1 = HIGH_FORWARD_BACK_SPEED; 			 				
				}
				else
				{
					if(Speed_Straight_ACC <= NORMAL_FORWARD_BACK_SPEED)
						  Speed_Straight_ACC++;
					Remote_data.RC_ch1 = Speed_Straight_ACC; 			 
				}
		 }
		 else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_S)      //S  ����
		 {			 
				if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT)   //Shift    �����ƶ�   	 
				{
					Remote_data.RC_ch1 = -HIGH_FORWARD_BACK_SPEED; 			 				
				}
				else
				{
					if(Speed_Straight_ACC <= NORMAL_FORWARD_BACK_SPEED)
						  Speed_Straight_ACC++;
					Remote_data.RC_ch1 = -Speed_Straight_ACC;			 
				}
		 }
		 else
		 {
			 Speed_Straight_ACC=100;
			 Remote_data.RC_ch1 = 0; 				 			 
		 }
		 //�����
		 if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_A)            //A  ��ƽ��
		 {
				if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT)   //Shift    �����ƶ�   	 
				{
					Remote_data.RC_ch0 = -HIGH_LEFT_RIGHT_SPEED; 			 				
				}
				else
				{
					if(Speed_Across_ACC <= NORMAL_LEFT_RIGHT_SPEED)
						  Speed_Across_ACC++;
					Remote_data.RC_ch0 = -Speed_Across_ACC;					 
				}			 
		 }
		 else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_D)            //D  ��ƽ��
		 {
				if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT)   //Shift    �����ƶ�   	 
				{
					Remote_data.RC_ch0 = HIGH_LEFT_RIGHT_SPEED; 			 				
				}
				else
				{
					if(Speed_Across_ACC <= NORMAL_LEFT_RIGHT_SPEED)
						  Speed_Across_ACC++;
					Remote_data.RC_ch0 = Speed_Across_ACC;			 
				}				 
		 }
		 else
		 {
			 Speed_Across_ACC = 100;
			 Remote_data.RC_ch0 = 0; 				 			 			 			 
		 }

		 
/**********��������̨**********/
		 if(workStatus == WORKING || workStatus == AUTOATTACK)    //�ֶ�����
		{
			Remote_data.RC_ch2 += (RC_Ctl.mouse.x * MOUSE_YAW_SPEED);     //Y��λ�û����ۼ�  
			Remote_data.RC_ch3 -= (RC_Ctl.mouse.y * MOUSE_PITCH_SPEED);   //P��λ�û����ۼ� 
		}
//		else if(workStatus == AUTOATTACK)  //�Զ���׼
//		{
//			Remote_data.RC_ch2 = 0;                         //�Զ���׼ʱȡ���ֶ�����Y��
//			Remote_data.RC_ch3 = 0;                         //�Զ���׼ʱȡ���ֶ�����P��		
//		}
		else  //����ģʽ����ʼ��ģʽ������ģʽ  �������ֶ�����
		{
//			Remote_data.RC_ch2 = 0;   
//			Remote_data.RC_ch3 = 0;   
		}
	 
		
}



/*
*ҡ��������
*/
void Remote_reload(void)
{
		Remote_data.RC_ch0 = 0;
		Remote_data.RC_ch1 = 0;
		Remote_data.RC_ch2 = 0;   
		Remote_data.RC_ch3 = 0;   
		Remote_data.RC_sw  = 0;   
}





