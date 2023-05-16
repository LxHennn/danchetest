#include "kalman.h"
/****************************************
调节Q和R
越信任模型，Q就越小
越信任测量值，R就越小
*****************************************/
inline void KalmanFilter_Angle(float In_GPS, float angle, K_Filter_A *K_data)
{
    K_data->Angle =angle; /* 先验估计 */
	
    K_data->Pdot[0] = K_data->Q_angle;

    K_data->PP[0][0] += K_data->Pdot[0] * K_data->dt; /* 误差协方差 */

    K_data->Angle_err = In_GPS - K_data->Angle; /* 测量值误差 */

    K_data->PCt_0 = K_data->PP[0][0]; /* 计算卡尔曼增益 */

    K_data->E = K_data->R_angle +  K_data->PCt_0;

    K_data->K_0 = K_data->PCt_0 / K_data->E;

    K_data->T_0 = K_data->PCt_0;

    K_data->PP[0][0] -= K_data->K_0 * K_data->T_0; /* 更新协方差 */

    K_data->Angle += K_data->K_0 * K_data->Angle_err; /* 后验修正 */
   }

K_Filter_A Kalman = 
    {
	 .dt = 0.1,
	 .Q_angle = 1,
	 .R_angle = 1,
	 .Pdot[0] = 0,
	 .PP[0][0] =0.270156,
	};		

	