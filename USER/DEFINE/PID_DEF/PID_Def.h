#ifndef __PIDDEF_H
#define __PIDDEF_H

/************底盘位置环pid参数设置************************/
#define CHASSIS_P_1_P   2.0f												
#define CHASSIS_P_1_I 	0.0f												
#define CHASSIS_P_1_D 	0.0f												

#define CHASSIS_P_2_P 	2.0f												
#define CHASSIS_P_2_I 	0.0f													
#define CHASSIS_P_2_D 	0.0f							

#define CHASSIS_P_3_P 	2.0f												
#define CHASSIS_P_3_I 	0.0f													
#define CHASSIS_P_3_D 	0.0f								

#define CHASSIS_P_4_P 	2.0f													
#define CHASSIS_P_4_I   0.0f												
#define CHASSIS_P_4_D 	0.0f	

/************底盘速度环pid参数设置************************/
#define CHASSIS_S_1_P   8.5f												
#define CHASSIS_S_1_I 	0.0f												
#define CHASSIS_S_1_D 	0.2f												

#define CHASSIS_S_2_P 	8.5f												
#define CHASSIS_S_2_I 	0.0f													
#define CHASSIS_S_2_D 	0.2f							

#define CHASSIS_S_3_P 	8.5f												
#define CHASSIS_S_3_I 	0.0f													
#define CHASSIS_S_3_D 	0.2f								

#define CHASSIS_S_4_P 	8.5f													
#define CHASSIS_S_4_I   0.0f												
#define CHASSIS_S_4_D 	0.2f	

#define CHASSIS_MOVE_FOLLOW_P 0.2  //底盘运动跟随PID   0.015    //不用这个
#define CHASSIS_MOVE_FOLLOW_I 0
#define CHASSIS_MOVE_FOLLOW_D 0.297

#define CHASSIS_ROTATE_FOLLOW_P 9.0  //底盘静止跟随PID   8.0
#define CHASSIS_ROTATE_FOLLOW_I 0.0   //0.01
#define CHASSIS_ROTATE_FOLLOW_D 0.0   //5.02   10.02

/***********云台P,Y轴速度环位置环参数定义**********/  
//P电机电调反馈速度环
//#define GIMBAL_P_PITCH_P 58.0f   //P位置环  53   410    160       58
//#define GIMBAL_P_PITCH_I 0.0      //8f      
//#define GIMBAL_P_PITCH_D 100.0    //0.0f       360       100     100

//#define GIMBAL_S_PITCH_P 9.5f     //P速度环(不要加i)       1.5     9.5
//#define GIMBAL_S_PITCH_I 0.0f      
//#define GIMBAL_S_PITCH_D 0.0f      

//P陀螺仪角速度环
#define GIMBAL_P_PITCH_P 25.0f    //P位置环  53   410    160       58
#define GIMBAL_P_PITCH_I 0.0      //8f      
#define GIMBAL_P_PITCH_D 0.0    //0.0f       360       100     100

#define GIMBAL_S_PITCH_P 17.5f     //P速度环(不要加i)       1.5     9.5
#define GIMBAL_S_PITCH_I 0.0f      
#define GIMBAL_S_PITCH_D 0.0f   

//外接陀螺仪
#define GIMBAL_P_YAW_P 130.0f     //Y位置环    150     62    130
#define GIMBAL_P_YAW_I 0.0f       
#define GIMBAL_P_YAW_D 100.0f     //           0      100   100

#define GIMBAL_S_YAW_P 9.0f      //Y速度环     8      10     9
#define GIMBAL_S_YAW_I 0.0f   
#define GIMBAL_S_YAW_D 0.0f      //2                   0

/*炮台模式PID*/
//外接陀螺仪
#define GIMBAL_INDEPENDENT_P_YAW_P 21.0f      //Y位置环
#define GIMBAL_INDEPENDENT_P_YAW_I 0.0f    
#define GIMBAL_INDEPENDENT_P_YAW_D 100.2f   

#define GIMBAL_INDEPENDENT_S_YAW_P 10.5f      //Y速度环
#define GIMBAL_INDEPENDENT_S_YAW_I 0.0f   
#define GIMBAL_INDEPENDENT_S_YAW_D 0.0f 


/*副云台速度角度环pid*/
#define GIMBAL_DOUBLE_P_PITCH_P 2.0f     //P位置环  53   410      2.0
#define GIMBAL_DOUBLE_P_PITCH_I 0.0      //8f      
#define GIMBAL_DOUBLE_P_PITCH_D 0.0      //0.0f       360         0

#define GIMBAL_DOUBLE_S_PITCH_P 8.5f     //P速度环(不要加i)
#define GIMBAL_DOUBLE_S_PITCH_I 0.0f      
#define GIMBAL_DOUBLE_S_PITCH_D 0.0f      

#define GIMBAL_DOUBLE_P_YAW_P 20.0f      //Y位置环  52      25      14
#define GIMBAL_DOUBLE_P_YAW_I 0.10f      //                        0.1
#define GIMBAL_DOUBLE_P_YAW_D 10.0f     //         100     100    102

#define GIMBAL_DOUBLE_S_YAW_P 8.2f       //Y速度环  2.8     3.8     7.2
#define GIMBAL_DOUBLE_S_YAW_I 0.0f   
#define GIMBAL_DOUBLE_S_YAW_D 0.0f    



/**********拨弹电机速度环pid参数定义**********/
#define FIRE_S_P 12.0f                  //供弹电机M2006速度环
#define FIRE_S_I 0.0f
#define FIRE_S_D 0.0f 



/*********视觉PY轴数据pid参数定义***************/

/**后面视觉需要三套pid，自瞄短焦，自瞄工业，打符工业**/

//短焦摄像头4mm
#define GIMBAL_AUTO_SHORT_P_PITCH_P 30.5f     //P自动位置环   7.5
#define GIMBAL_AUTO_SHORT_P_PITCH_I 0.0f      
#define GIMBAL_AUTO_SHORT_P_PITCH_D 0.1f      

#define GIMBAL_AUTO_SHORT_S_PITCH_P 5.5f     //P自动速度环   13.5
#define GIMBAL_AUTO_SHORT_S_PITCH_I 0.0f      
#define GIMBAL_AUTO_SHORT_S_PITCH_D 0.0f      

#define GIMBAL_AUTO_SHORT_P_YAW_P 100.0f       //Y自动位置环   150
#define GIMBAL_AUTO_SHORT_P_YAW_I 0.0f        
#define GIMBAL_AUTO_SHORT_P_YAW_D 0.0f        

#define GIMBAL_AUTO_SHORT_S_YAW_P 5.6f       //Y自动速度环    7.6
#define GIMBAL_AUTO_SHORT_S_YAW_I 0.0f        
#define GIMBAL_AUTO_SHORT_S_YAW_D 0.0f        

//长焦摄像头8mm
#define GIMBAL_AUTO_LONG_P_PITCH_P 92.5f     //P自动位置环   7.5
#define GIMBAL_AUTO_LONG_P_PITCH_I 0.0f      
#define GIMBAL_AUTO_LONG_P_PITCH_D 0.1f      

#define GIMBAL_AUTO_LONG_S_PITCH_P 6.5f     //P自动速度环   13.5
#define GIMBAL_AUTO_LONG_S_PITCH_I 0.0f      
#define GIMBAL_AUTO_LONG_S_PITCH_D 0.0f      

#define GIMBAL_AUTO_LONG_P_YAW_P 32.0f       //Y自动位置环
#define GIMBAL_AUTO_LONG_P_YAW_I 0.0f        
#define GIMBAL_AUTO_LONG_P_YAW_D 0.2f        

#define GIMBAL_AUTO_LONG_S_YAW_P 23.0f       //Y自动速度环
#define GIMBAL_AUTO_LONG_S_YAW_I 0.0f        
#define GIMBAL_AUTO_LONG_S_YAW_D 0.0f        

//工业摄像头
#define GIMBAL_AUTO_INDUSTRY_P_PITCH_P 120.0f    //P自动位置环  530.0f
#define GIMBAL_AUTO_INDUSTRY_P_PITCH_I 0.0f      
#define GIMBAL_AUTO_INDUSTRY_P_PITCH_D 0.0f    //200.0f   

#define GIMBAL_AUTO_INDUSTRY_S_PITCH_P 2.1f     //P自动速度环
#define GIMBAL_AUTO_INDUSTRY_S_PITCH_I 0.0f      
#define GIMBAL_AUTO_INDUSTRY_S_PITCH_D 0.0f      

#define GIMBAL_AUTO_INDUSTRY_P_YAW_P 160.0f       //Y自动位置环 170
#define GIMBAL_AUTO_INDUSTRY_P_YAW_I 0.0f        
#define GIMBAL_AUTO_INDUSTRY_P_YAW_D 0.0f        

#define GIMBAL_AUTO_INDUSTRY_S_YAW_P 11.0f       //Y自动速度环 12
#define GIMBAL_AUTO_INDUSTRY_S_YAW_I 0.0f        
#define GIMBAL_AUTO_INDUSTRY_S_YAW_D 0.0f        


//打符PID
#define GIMBAL_AUTO_BUFF_P_PITCH_P 18.5f    //P自动位置环  110.0f   84.0    35.5      13.5
#define GIMBAL_AUTO_BUFF_P_PITCH_I 0.0f      
#define GIMBAL_AUTO_BUFF_P_PITCH_D 0.0f    //200.0f                0

#define GIMBAL_AUTO_BUFF_S_PITCH_P 30.5f     //P自动速度环  3.5      3.8      14.2     30.1
#define GIMBAL_AUTO_BUFF_S_PITCH_I 0.0f      
#define GIMBAL_AUTO_BUFF_S_PITCH_D 0.0f      

#define GIMBAL_AUTO_BUFF_P_YAW_P 25.3f       //Y自动位置环  80   54.3       25.3
#define GIMBAL_AUTO_BUFF_P_YAW_I 0.0f        
#define GIMBAL_AUTO_BUFF_P_YAW_D 0.0f        

#define GIMBAL_AUTO_BUFF_S_YAW_P 13.7f       //Y自动速度环  12   14.4       13.7
#define GIMBAL_AUTO_BUFF_S_YAW_I 0.0f        
#define GIMBAL_AUTO_BUFF_S_YAW_D 0.0f    






#endif
