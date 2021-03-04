#ifndef _CAN1_H
#define _CAN1_H

void CAN1_Configuration(uint8_t SJW, uint8_t BS1, uint8_t BS2, uint8_t Prescaler, uint8_t mode);

void CAN1_SendCommand_Chassis(int16_t ESC_201,int16_t ESC_202,int16_t ESC_203,int16_t ESC_204);
void CAN1_SendCommand_Gimbal(int16_t control_205,int16_t control_206, int16_t control_207, int16_t control_208);

#endif

