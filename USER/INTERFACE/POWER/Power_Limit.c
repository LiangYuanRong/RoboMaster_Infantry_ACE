#include "Power_Limit.h"
#include "BMI160.h"

//void Chassis_Power_Limit(POWER_Data *power_data, )
//{
//	#ifdef power_limit
//	if (chassis_power_limit == NULL)
//	{
//			return;                            
//	}
//	
//	int32_t Mergeoutput=0;
//	int16_t LimtOutValue=0;
//	
//	if(PowerLimit.RemainPower[2]<50&&PowerLimit.RemainPower[2]>5)
//	{
//		
//	Mergeoutput=int16_t_abs(chassis_power_limit->chassis_motor[0].give_current)+int16_t_abs(chassis_power_limit->chassis_motor[1].give_current)
//							+int16_t_abs(chassis_power_limit->chassis_motor[2].give_current)+int16_t_abs(chassis_power_limit->chassis_motor[3].give_current);
//	
//	LimtOutValue=PowerLimit.RemainPower[2]*PowerLimit.RemainPower[2]*6;
//	
//	chassis_power_limit->chassis_motor[0].give_current=LimtOutValue*chassis_power_limit->chassis_motor[0].give_current/Mergeoutput;
//	chassis_power_limit->chassis_motor[1].give_current=LimtOutValue*chassis_power_limit->chassis_motor[1].give_current/Mergeoutput;
//	chassis_power_limit->chassis_motor[2].give_current=LimtOutValue*chassis_power_limit->chassis_motor[2].give_current/Mergeoutput;
//	chassis_power_limit->chassis_motor[3].give_current=LimtOutValue*chassis_power_limit->chassis_motor[3].give_current/Mergeoutput;
//	}
//	else if(PowerLimit.RemainPower[2]<5)
//	{
//	chassis_power_limit->chassis_motor[0].give_current=0;
//	chassis_power_limit->chassis_motor[1].give_current=0;
//	chassis_power_limit->chassis_motor[2].give_current=0;
//	chassis_power_limit->chassis_motor[3].give_current=0;
//	}
//  #endif
//}
