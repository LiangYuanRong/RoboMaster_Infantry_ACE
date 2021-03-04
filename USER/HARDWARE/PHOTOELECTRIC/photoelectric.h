#ifndef __PHOTOELECTRIC_H
#define __PHOTOELECTRIC_H

void Photoelectric_GPIO_Init(void);
void Yaw_Zero_GPIO_Init(void);     //零点校准传感器
void Yaw_Two_Zero_GPIO_Init(void);

u8 Yaw_Zero_Value(void);
u8 Sec_Yaw_Zero_Value(void);   //检测Y轴是否到位   到中值返回0，否则返回1

#endif
