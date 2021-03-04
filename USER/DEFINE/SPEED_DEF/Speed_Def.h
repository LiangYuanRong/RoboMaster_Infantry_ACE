#ifndef __SPEEDDEF_H
#define __SPEEDDEF_H

/*****底盘电机移动速度设定*****/ 
#define M3508_MAX_OUTPUT_CURRENT  5000  //m3508电机最大电流输出  
#define M2006_MAX_OUTPUT_CURRENT  9500   //m2006电机最大电流输出

#define NORMAL_FORWARD_BACK_SPEED 	300   //键盘普通直行速度
#define NORMAL_LEFT_RIGHT_SPEED   	300   //键盘普通平移速度

#define HIGH_FORWARD_BACK_SPEED 	600     //键盘加速直行速度
#define HIGH_LEFT_RIGHT_SPEED   	600     //键盘加速平移速度

#define CHASSIS_SPEED_GAIN        12.0f    //正常行驶速度增益
#define TWIST_SPEED_GAIN          2.0f     //扭腰行驶速度增益

#define CHASSIS_ROTATION_SPEED    2000      //小陀螺的旋转速度 
#define CHASSIS_ROTATION_MOVE_SPEED  1700   //小陀螺移动时为防止轨迹失真减转速 
#define CHASSIS_TWIST_SPEED       1600      //扭腰速度


/*****键盘鼠标遥控速度设置******/ 
#define MOUSE_YAW_SPEED 			0.021    //鼠标yaw轴速度增益
#define MOUSE_PITCH_SPEED 		0.08     //鼠标pitch轴速度增益0.13
#define RC_YAW_SPEED          0.0026   //遥控器yaw轴速度增益
#define RC_PITCH_SPEED        0.0026   //遥控器pitch轴速度增益 0.0026


/*****发弹系统速度设定*******/ //零级：15m/s   一级：18m/s  二级：22m/s  三级：30m/s
#define THREE_SHOOT_SPEED     1226       //30摩擦轮高速pwm
#define TWO_SHOOT_SPEED       1193       //22摩擦轮高速pwm
#define ONE_SHOOT_SPEED       1172       //18摩擦轮高速pwm
#define ZERO_SHOOT_SPEED      1165       //15摩擦轮低速pwm    1160-10.7m/s   1180-17.3m/s   1200-20.4m/s   1220-25m/s   1240
#define SHOOT_STOP            1000       //0摩擦轮停止pwm
#define LOADING_SPEED         -3000      //供弹电机速度

/*****最大底盘功率***********/  //零级：50w   一级：60w   二级：70w   三级：100w
#define ZERO_CHASSIS_POWER  50
#define ONE_CHASSIS_POWER   60
#define TWO_CHASSIS_POWER   70
#define THREE_CHASSIS_POWER 100


/*****运动加速度限制*******/
#define STRAIGHT_ACCELERAD        3.5f      //直行底盘加速度限制
#define TRANSLATION_ACCELERAD     5.5f      //平移底盘加速度限制
#define ROTATING_ACCELERAD        19.0f     //旋转底盘加速度限制
#define GIMBAL_PITCH_ACCELERAD    2         //云台俯仰加速度限制
#define GIMBAL_YAW_ACCELERAD      2         //云台偏航加速度限制
#define GIMBAL_AUTO_YAW_ACCELERAD 1         //云台自动偏航加速度限制


/*************初始化云台速度******************/
#define YAW_INIT_SPEED 150


/*************功率限制参数**********************/
#define PowerLimit_Param  6.5f         //功率限制输出参数
#define PowerLimit_Thres  55.0f        //功率限制阈值（缓冲功率剩余量）50.0f


/****控制量低通滤波参数**********/
#define Chassis_Fir_Ord_Low_Fil_Param       0.0110f  //0.0097f越小越平稳，灵敏度越低；越高输出不稳，但灵敏度更高
#define Gimbal_Pitch_Fir_Ord_Low_Fil_Param  0.0864f
#define Gimbal_Yaw_Fir_Ord_Low_Fil_Param    0.215f
#define Gimbal_Remote_Fir_Ord_Low_Fil_Param 0.123f

/*************pitch和yaw输出量限制************/
#define YAW_OUTPUT_LIMIT        11000 
#define YAW_INIT_OUTPUT_LIMIT   8000
#define PITCH_OUTPUT_LIMIT      8000 
#define PITCH_INIT_OUTPUT_LIMIT 5000

/*************减速电机启动电流补偿（快速启动）**********/
#define GEAR_MOTOR_START_CURRENT   300
#define GEAR_MOTOR_REDUCE_CURRENT  1.21






#endif
