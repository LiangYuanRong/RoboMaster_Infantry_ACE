#include "main.h"
#include "Gyro_Task.h"
#include "kalman_filter.h"

/*����*/
int gyro_heart=0;
/*��ջʣ��*/
extern TaskHandle_t GyroTask_Handler; // ջ��С
int GyroTask_water=0;


/*
*����������
*/
void Gyro_Task(void *pvParameters)   //��ʱ��������������񣬻����bug
{
  vTaskDelay(GYRO_TASK_INIT_TIME);

	while(1)
	{

    vTaskDelay(GYRO_CONTROL_TIME_MS);
	}
}

