#include "filter.h"


float K1 = 0.02;
float angle, angle_dot;  // 角度和角速度
float Q_angle = 0.001;   // 过程噪声的协方差
float Q_gyro = 0.003;    // 0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
float R_angle = 0.5;     // 测量噪声的协方差 既测量偏差
float dt = 0.005;        // 注意：dt的取值为kalman滤波器采样时间                
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] = { 0,0,0,0 };
float PP[2][2] = { { 1, 0 },
                   { 0, 1 } };



/**************************************************************************
函数功能：简易卡尔曼滤波
入口参数：加速度、角速度
返回  值：无
主要对协方差Q和R的取值进行设计（Q_angle, Q_gyro, R_angle），
R取值越小，滤波响应和收敛越迅速；
Q取值越小，抑制滤除噪声的能力越强。
因此，具体取值也需要反复实际调试进行权衡确定。
**************************************************************************/
void Kalman_Filter(float Accel, float Gyro)
{
	//1、卡尔曼第一个公式(状态预测):X(k|k-1)=AX(k-1|k-1)+BU(k)
	angle += (Gyro - Q_bias) * dt; //先验估计 (角速度积分就是角度)
	
	//2、卡尔曼第二个公式(计算误差协方差)：AP(k-1|k-1)A' + Q
	Pdot[0] = Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3] = Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;

	Angle_err = Accel - angle;	//zk-先验估计

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];

	E = R_angle + C_0 * PCt_0;

	//3、卡尔曼第三个公式(计算卡尔曼增益):Kg(k) = P(k|k-1)H' / (HP(k|k-1)H' + R)
	//R --->系统测量噪声协方差
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	//5、卡尔曼第五个公式(更新误差协方差)：P(k|k) = (1 - Kg(k)H) P(k|k-1)
	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	//4、卡尔曼第四个公式(修正估计):X(k|k) = X(k|k-1) + Kg(k)(Z(k) - HX(k|k-1))
	angle += K_0 * Angle_err;	 //后验估计
	Q_bias += K_1 * Angle_err;	 //后验估计
	angle_dot = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
}



/**************************************************************************
函数功能：PI改善型互补滤波
入口参数：加速度、角速度
返回  值：无
**************************************************************************/
void Improve_Complementary_Filter(float angle_m, float gyro_m)
{
	
}




