#ifndef KALMAN_H__
#define KALMAN_H__

typedef struct
{
    
    float Angle;      /* �˲���ĽǶ� */
    float Gyro;       /* ���ýǶ�΢�ֺ�Ľ��ٶ� */
    float dt;         /* ������� */
    float Angle_err;  /* ��������ʱ�ĽǶȲ� */
    float Q_bias;     /* ������0Ư������Э���� *//* ��Ӧ�Ǿ������ｫ��� */
    float Q_angle;    /* ϵͳ����������Э���� *//* ��Ӧ�Ǿ������ｫ��� */
    float Q_gyro;     /* ϵͳ����������Э���� *//* ��Ӧ�Ǿ������ｫ��� */
    float R_angle;    /* �ǶȲ���������Э���� *//* ��Ӧ�Ǿ������ｫ��� */
    float C_0;        /* ��ʼֵҪ�� 1 */
    float Pdot[4];
    float PP[2][2];   /* (0,0) �� (1.1) Ҫ�� 1 */
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