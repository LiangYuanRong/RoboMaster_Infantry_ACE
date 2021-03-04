#include "main.h"
#include "Gyro_Task.h"
#include "kalman_filter.h"

/*心跳*/
int gyro_heart=0;
/*堆栈剩余*/
extern TaskHandle_t GyroTask_Handler; // 栈大小
int GyroTask_water=0;


/*
*陀螺仪任务
*/
void Gyro_Task(void *pvParameters)   //暂时不单独用这个任务，会出现bug
{
  vTaskDelay(GYRO_TASK_INIT_TIME);

	while(1)
	{

    vTaskDelay(GYRO_CONTROL_TIME_MS);
	}
}

