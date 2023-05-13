#ifndef KALMAN_H__
#define KALMAN_H__

typedef struct
{
    
    float Angle;      /* 滤波后的角度 */
    float Gyro;       /* 利用角度微分后的角速度 */
    float dt;         /* 检测周期 */
    float Angle_err;  /* 后验修正时的角度差 */
    float Q_bias;     /* 陀螺仪0漂噪声的协方差 *//* 本应是矩阵，这里将其简化 */
    float Q_angle;    /* 系统过程噪声的协方差 *//* 本应是矩阵，这里将其简化 */
    float Q_gyro;     /* 系统过程噪声的协方差 *//* 本应是矩阵，这里将其简化 */
    float R_angle;    /* 角度测量噪声的协方差 *//* 本应是矩阵，这里将其简化 */
    float C_0;        /* 初始值要赋 1 */
    float Pdot[4];
    float PP[2][2];   /* (0,0) 和 (1.1) 要赋 1 */
    float PCt_0;
    float PCt_1;
    float E;
    float K_0; 
    float K_1;
    float T_0;
    float T_1;

}K_Filter_A;


		
extern void KalmanFilter_Angle(float In_GPS, float In_gyro, K_Filter_A *K_data);
extern K_Filter_A Kalman;
#endif 