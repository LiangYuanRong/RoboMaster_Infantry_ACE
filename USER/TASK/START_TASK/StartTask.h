#ifndef __STARTTASK_H
#define __STARTTASK_H

void Start_Task(void);


/*************ϵͳ��һЩģʽö��*****************/

/*��������״̬*/
typedef enum 
{
	INITIALIZE,   //��ʼ��״̬
	WORKING,      //�ֶ�״̬
	AUTOATTACK,   //����״̬
	AUTOBUFF,     //���״̬
	REPLENISHMEN, //����״̬
	DOUBLE_GIMBAL,//˫��̨״̬������̨���飬����̨������ 
	POWEROFF,     //����״̬
	
}WorkStatus;



/*�ⲿʹ������*/ 
extern WorkStatus          workStatus;               //Ĭ�Ϲ���ģʽΪ����ģʽ



#endif
