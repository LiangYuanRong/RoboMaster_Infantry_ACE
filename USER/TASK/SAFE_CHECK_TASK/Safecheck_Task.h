#ifndef __SAFECHECK_TASK
#define __SAFECHECK_TASK

#define CHECK_CONTROL_TIME_MS 20

typedef struct
{	
	//remote
	int8_t  RC_Connected;
	int8_t  RC_Receive_Flag;
	int16_t RC_Error;
	
	//can1
	//can2
	
} DogTypeDef;

/*外部使用声明*/
extern DogTypeDef Safecheck_dog;


/*全局函数声明*/
void Safecheck_Task(void *pvParameters);   


#endif
