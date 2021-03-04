#ifndef __CANISR_H
#define __CANISR_H
#include "stm32f4xx.h"

extern int8_t INITIALIZE_flag;  //在云台任务中定义的变量，用于云台完成初始化后发送给底盘

/*=============can1电机通信===============*/
void CAN1_SendCommand_Chassis(int16_t ESC_201,int16_t ESC_202,int16_t ESC_203,int16_t ESC_204);
void CAN1_SendCommand_Gimbal_Fire(int16_t ESC_205,int16_t ESC_206,int16_t ESC_207);

void Can1_Chassis_sendText_to_Gimbal_one(void);
void Can1_Chassis_sendText_to_Gimbal_two(void);
void Can1_Chassis_sendText_to_Gimbal_three(void);

void CAN1_SendCommand_Encoder_Read(void);


/*=============can2通信===============*/
void CAN2_Send_Yaw_Data_To_Chassis(u8 INITIALIZE_flag , int16_t gimbal_output , u8 YAW_ZERO_FLAG , u8 GIMBAL_SUPPLY_FLAG , int16_t Sec_chassis_angle); //云台can2发送到底盘
void CAN1_SendCommand_Gimbal_Pitch(int16_t ESC_201,int16_t ESC_204);           //P轴电机是201,副云台yaw2006是204
void CAN2_SendCommand_Chassis_Fire(int16_t ESC_207);                           //供弹电机207

void CAN2_Send_Data1_RC_To_Gimbal(void);
void CAN2_Send_Data1_MK_To_Gimbal(void);
void CAN2_Send_Data2_To_Gimbal(void);

#endif
