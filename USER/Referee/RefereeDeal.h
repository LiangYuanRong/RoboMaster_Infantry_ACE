#ifndef __RefereeDeal_H_
#define __RefereeDeal_H_
#include "stm32f4xx.h"



/*比赛状态数据*/
typedef __packed struct 
{   
	uint8_t game_type : 4;   
	uint8_t game_progress : 4;   
	uint16_t stage_remain_time; 
	uint8_t error;
}ext_game_status_t; 

/*血量数据*/
typedef __packed struct
{   
	uint16_t red_1_robot_HP;    
	uint16_t red_2_robot_HP;   
	uint16_t red_3_robot_HP;   
	uint16_t red_4_robot_HP;    
	uint16_t red_5_robot_HP;  
  uint16_t red_7_robot_HP;   
	uint16_t red_outpost_HP;  
	uint16_t red_base_HP;   
	uint16_t blue_1_robot_HP;    
	uint16_t blue_2_robot_HP;  
	uint16_t blue_3_robot_HP;    
	uint16_t blue_4_robot_HP;  
	uint16_t blue_5_robot_HP;  
	uint16_t blue_7_robot_HP;  
	uint16_t blue_outpost_HP;   
	uint16_t blue_base_HP; 
	uint8_t error;
} ext_game_robot_HP_t; 

/*飞镖发射状态*/
typedef __packed struct { 
	uint8_t dart_belong;  
	uint16_t stage_remaining_time; 
	uint8_t error;	
} ext_dart_status_t; 

/*场地事件数据*/
typedef __packed struct{
	uint32_t Parking : 2;
	uint32_t EnergyAgency : 2;
	uint32_t Shield : 1;
	uint32_t other : 27;
	uint8_t error;
} ext_event_data_t; 

/*补给站动作标识*/
typedef __packed struct {   
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;    
	uint8_t supply_projectile_step;  
	uint8_t supply_projectile_num; 
	uint8_t error;
} ext_supply_projectile_action_t;

/*飞镖发射口数据*/
typedef __packed struct {   
	uint8_t dart_remaining_time;   
	uint8_t error;
} ext_dart_remaining_time_t; 

/*机器人状态数据*/
typedef __packed struct {   
	uint8_t robot_id;   
	uint8_t robot_level;   
	uint16_t remain_HP;  
	uint16_t max_HP;  
	uint16_t shooter_heat0_cooling_rate;  
	uint16_t shooter_heat0_cooling_limit;  
	uint16_t shooter_heat1_cooling_rate;  
	uint16_t shooter_heat1_cooling_limit;  
	uint8_t shooter_heat0_speed_limit;  
	uint8_t shooter_heat1_speed_limit;  
	uint8_t max_chassis_power;   
	uint8_t mains_power_gimbal_output : 1;  
	uint8_t mains_power_chassis_output : 1;  
	uint8_t mains_power_shooter_output : 1; 
	uint8_t error;
} ext_game_robot_status_t; 

/*功率热量数据*/
typedef __packed struct {   
	uint16_t chassis_volt;  
	uint16_t chassis_current; 
	float chassis_power;  
	uint16_t chassis_power_buffer;  
	uint16_t shooter_heat0;   
	uint16_t shooter_heat1;   
	uint16_t mobile_shooter_heat2; 
	uint8_t error;
} ext_power_heat_data_t; 

/*机器人增益数据*/
typedef __packed struct {  
	uint8_t EnrichingBloodBuff : 1;
	uint8_t	MuzzleCdBuff : 1;
	uint8_t DefenseBuff : 1;
	uint8_t AttackBuff : 1;
	uint8_t other : 4;
	uint8_t error;
}ext_buff_t; 

/*飞机数据*/
typedef __packed struct {   
	uint16_t energy_point;   
	uint8_t attack_time; 
	uint8_t error;
} ext_aerial_robot_energy_t;

/*伤害状态*/
typedef __packed struct {   
	uint8_t armor_id : 4;   
	uint8_t hurt_type : 4; 
	uint8_t error;
} ext_robot_hurt_t; 

/*实时射击信息*/
typedef __packed struct {   
	uint8_t bullet_type;   
	uint8_t bullet_freq;    
	float bullet_speed; 
	uint8_t error;
} ext_shoot_data_t; 

/*子弹剩余发射数*/
typedef __packed struct { 
	uint16_t bullet_remaining_num; 
	uint8_t error;
} ext_bullet_remaining_t; 

/*RFID状态*/
typedef __packed struct { 
	uint32_t BaseLand : 1;
	uint32_t HighLand : 1;
	uint32_t Energy : 1;
	uint32_t OverSlope : 1;
	uint32_t Sentry : 1;
	uint32_t Resources : 1;
	uint32_t BloodPoint : 1;
	uint32_t BloodCard : 1;
	uint32_t other : 24;
	uint8_t error;
} ext_rfid_status_t; 

/*裁判系统数据*/
typedef __packed struct
{
	u8 RefereeData[256];
	u8 RealData[45];
	int16_t DataLen;
	int16_t RealLen;
	int16_t Cmd_ID;
	u8 RECEIVE_FLAG;
	ext_game_status_t GameStatus;
	ext_game_robot_HP_t RobotHP;
	ext_dart_status_t DartStatus; 
	ext_event_data_t EventData;
	ext_supply_projectile_action_t SupplyAction;
	ext_dart_remaining_time_t RemainingTime;
	ext_game_robot_status_t RobotStatus;
	ext_power_heat_data_t PowerHeat;
	ext_buff_t Buff;
	ext_aerial_robot_energy_t AerialEnergy;
	ext_robot_hurt_t RobotHurt;
	ext_shoot_data_t ShootData; 
	ext_bullet_remaining_t BulletNum;
	ext_rfid_status_t RFIDStatus;
}REFEREE_t;


/*float数组重组*/
typedef union    
{  
	struct
   {
   u8 LB;  
   u8 MLB;  
   u8 MHB;  
   u8 HB;   
	}float_byte;
   float value; 
}Float_t;

void RefereeDataDeal(REFEREE_t*referee);
REFEREE_t*Return_Referee_Point(void);
#endif
