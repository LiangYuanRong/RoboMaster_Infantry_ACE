#include "kalman_filter.h"


/*********************************************************************一阶卡尔曼***********************************************************************/
/**    
  * @author  Liu heng
  * 一阶卡尔曼滤波器来自RoboMaster论坛  
  *   一维卡尔曼滤波器                     
  *   使用时先定义一个kalman指针，然后调用kalmanCreate()创建一个滤波器 
  *   每次读取到传感器数据后即可调用KalmanFilter()来对数据进行滤波
  *          使用示例                                             
  *          extKalman_t p;                  //定义一个卡尔曼滤波器结构体                                                 
  *          float SersorData;             //需要进行滤波的数据                                          
  *          KalmanCreate(&p,20,200);      //初始化该滤波器的Q=20 R=200参数                                                  
  *          while(1)                                                                
  *          {                                                                            
  *             SersorData = sersor();                     //获取数据                                           
  *             SersorData = KalmanFilter(&p,SersorData);  //对数据进行滤波                                                                            
  *          }                                                                            
  */


/**
  * @name   kalmanCreate
  * @brief  创建一个卡尔曼滤波器
  * @param  p:  滤波器
  *         T_Q:系统噪声协方差
  *         T_R:测量噪声协方差
  *         
  * @retval none
  * @attention R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
  *		       	反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
  */ 
void KalmanCreate(extKalman_t *p,float T_Q,float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
	p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}

/**
  * @name   KalmanFilter
  * @brief  卡尔曼滤波器
  * @param  p:  滤波器
  *         dat:待滤波数据
  * @retval 滤波后的数据
  * @attention Z(k)是系统输入,即测量值   X(k|k)是卡尔曼滤波后的值,即最终输出
  *            A=1 B=0 H=1 I=1  W(K)  V(k)是高斯白噪声,叠加在测量值上了,可以不用管
  *            以下是卡尔曼的5个核心公式
  *            一阶H'即为它本身,否则为转置矩阵
  */

float KalmanFilter(extKalman_t* p,float dat)
{
    p->X_mid =p->A*p->X_last;                     //百度对应公式(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    p->P_mid = p->A*p->P_last+p->Q;               //百度对应公式(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
    p->kg = p->P_mid/(p->P_mid+p->R);             //百度对应公式(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //百度对应公式(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;                //百度对应公式(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
    p->P_last = p->P_now;                         //状态更新
    p->X_last = p->X_now;
    return p->X_now;							  //输出预测结果x(k|k)
}







/*******************************************************************二阶卡尔曼***************************************************************/
/*
*功能：视觉数据卡尔曼初始化
*输入：
*/
kalman_filter_init_t kalman_Filter_Init={0};
kalman_filter_t kalman_Filter={0};
extern float len;

void Kalman_Filter_Init(kalman_filter_t *F, kalman_filter_init_t *I)
{
	mat_init(&F->xhat,4,1,(float *)I->xhat_data);//4*1，特征值(k-1)
	mat_init(&F->xhatminus,4,1,(float *)I->xhatminus_data);//4*1，特征值(k)
	mat_init(&F->z,2,1,(float *)I->z_data);//2*1		测量值
	mat_init(&F->Pminus,4,4,(float *)I->Pminus_data);//4*4 新的协方差矩阵
	mat_init(&F->k,4,2,(float *)I->K_data);//4*2	kalman增益
	mat_init(&F->p,4,4,(float *)I->P_data);//4*4  协方差矩阵
	mat_init(&F->AT,4,4,(float *)I->AT_data);//4*4	状态方程
	mat_init(&F->HT,4,2,(float *)I->HT_data);//4*2	
	mat_init(&F->A,4,4,(float *)I->A_data);//4*4
	mat_init(&F->H,2,4,(float *)I->H_data);//2*4
	mat_init(&F->Q,4,4,(float *)I->Q_data);//4*4
	mat_init(&F->R,2,2,(float *)I->R_data);//2*2
	mat_trans(&F->A, &F->AT);
	mat_trans(&F->H, &F->HT);
}


/*
*名称：视觉数据卡尔曼处理
*功能：对小电脑传回的数据进行卡尔曼滤波处理
*输入：1、视觉卡尔曼数据结构体  2、回传角度  3、回传角速度
*输出：auto_yaw_angle_kf    auto_pitch_angle_kf   auto_yaw_speed
*/
void Kalman_Filter_Calc(kalman_filter_t *F, float signal1, float signal2)
{
  float TEMP_data441[16]= {0,0,0,0,
												   0,0,0,0,
													 0,0,0,0,
													 0,0,0,0};
	float TEMP_data442[16]= {0,0,0,0,
												   0,0,0,0,
													 0,0,0,0,
													 0,0,0,0};
  float TEMP_data21[2] = {0, 0};
	float TEMP_data41[16]= {1,0,0,0,
													0,1,0,0,
													0,0,1,0,
													0,0,0,1};
	float TEMP_data24[8]= {0,0,0,0,
												 0,0,0,0};
	float TEMP_data22[4]= {0,0,0,0};														
	float TEMP_data222[4]= {0,0,0,0};	
	float TEMP_data42[8]= {0,0,0,0,
												 0,0,0,0};
	float TEMP_data212[2] = {0, 0};
	float TEMP_data412[4]=  {0,0,0,0};
	
  mat TEMP441,TEMP442,TEMP21,TEMP41,TEMP24,TEMP22,TEMP222,TEMP42,TEMP212,TEMP412;

  mat_init(&TEMP441,4,4,(float *)TEMP_data441);
  mat_init(&TEMP442,4,4,(float *)TEMP_data442);
	mat_init(&TEMP21,2,1,(float *)TEMP_data21);													
	mat_init(&TEMP41,4,4,(float *)TEMP_data41);	
	mat_init(&TEMP24,2,4,(float *)TEMP_data24);	
	mat_init(&TEMP22,2,2,(float *)TEMP_data22);	
	mat_init(&TEMP222,2,2,(float *)TEMP_data222);
	mat_init(&TEMP42,4,2,(float *)TEMP_data42);
	mat_init(&TEMP212,2,1,(float *)TEMP_data212);
	mat_init(&TEMP412,4,1,(float *)TEMP_data412);
	
  F->z.pData[0] = signal1;
  F->z.pData[1] = signal2;							
	
	
  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus);//4*4 ,4*1 -->4*1

			
	//2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->p, &F->Pminus);	//44 44 44
  mat_mult(&F->Pminus, &F->AT, &TEMP441);//44 44 44
  mat_add(&TEMP441, &F->Q, &F->Pminus);//44 44 44


  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&F->H, &F->Pminus, &TEMP24);//24 44 24
  mat_mult(&TEMP24, &F->HT, &TEMP22);//24 42 22
  mat_add(&TEMP22, &F->R, &TEMP222);//22 22 22

 
  mat_inv(&TEMP222, &TEMP22);//42  44 这里应该是2*2 的逆为  2*2 
  mat_mult(&F->Pminus, &F->HT, &TEMP42);//44 42  42
  mat_mult(&TEMP42, &TEMP22, &F->k);//42 44  


  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP21);		//24  41  21
  mat_sub(&F->z, &TEMP21, &TEMP212);					//21  21  21
  mat_mult(&F->k, &TEMP212, &TEMP412);         //42  41  21  -> 42 21  41
  mat_add(&F->xhatminus, &TEMP412, &F->xhat);  //41  21  41  ->41 41 41
  

	//5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->k, &F->H, &TEMP441);							//42  24  44
  mat_sub(&TEMP41, &TEMP441,&TEMP442 );							//44  44  44
  mat_mult(&TEMP442, &F->Pminus, &F->p);					//44  44  44
	
	
	F->filtered_value[0] = F->xhat.pData[0];
  F->filtered_value[1] = F->xhat.pData[1];
	F->filtered_value[2] = F->xhat.pData[2];
	F->filtered_value[3] = F->xhat.pData[3];

  
	Vision_Auto_Data.auto_yaw_angle_kf=F->xhat.pData[0];
	Vision_Auto_Data.auto_pitch_angle_kf=F->xhat.pData[1];
	Vision_Auto_Data.auto_yaw_speed=F->xhat.pData[2];

}
