#include "pid.h"

//pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
// 位置式PID控制
float Position1(float Encoder,float Target)
{
    float pwm = 0;
    pos1.ek = Target - Encoder; // 计算当前误差
    //pos1.ek_sum += pos1.ek;      //求出偏差的积分
		if(pos1.ek_sum<=pos1.ek_sumlimit&&pos1.ek_sum>=-pos1.ek_sumlimit)
			pos1.ek_sum += pos1.ek;
		else if(pos1.ek_sum<=-pos1.ek_sumlimit)
		{
			pos1.ek_sum=-pos1.ek_sumlimit;
			if(pos1.ek>0)
				pos1.ek_sum += pos1.ek;
		}
		else if(pos1.ek_sum>=pos1.ek_sumlimit)
		{
			pos1.ek_sum=pos1.ek_sumlimit;
			if(pos1.ek<0)
				pos1.ek_sum += pos1.ek;
		}
    pwm = pos1.kp*pos1.ek + pos1.ki*pos1.ek_sum+pos1.kd*(pos1.ek-pos1.ek_1);   //位置式pos1控制器
    pos1.ek_1 = pos1.ek;   //保存上一次偏差
    if(pwm > pos1.limit)
    {
      pwm =  pos1.limit;
    }
    else if(pwm < -pos1.limit)
    {
      pwm =  -pos1.limit;
    }
    return pwm;
}

float Position2(float Encoder,float Target)
{
    float pwm = 0;
    pos2.ek = Target - Encoder; // 计算当前误差
    pos2.ek_sum += pos2.ek;      //求出偏差的积分
    pwm = pos2.kp*pos2.ek + pos2.ki*pos2.ek_sum+pos2.kd*(pos2.ek-pos2.ek_1);   //位置式pos1控制器
    pos2.ek_1 = pos2.ek;   //保存上一次偏差
    if(pwm > pos2.limit)
    {
      pwm =  pos2.limit;
    }
    else if(pwm < -pos2.limit)
    {
      pwm =  -pos2.limit;
    }
    return pwm;
}

float Position3(float Encoder,float Target)
{
    float pwm = 0;
    pos3.ek = Target - Encoder; // 计算当前误差
    //pos3.ek_sum += pos3.ek;      //求出偏差的积分
	  if(pos3.ek_sum<pos3.ek_sumlimit&&pos3.ek_sum>-pos3.ek_sumlimit)
			pos3.ek_sum += pos3.ek;
		else if(pos3.ek_sum<=-pos3.ek_sumlimit)
		{
			pos3.ek_sum=-pos3.ek_sumlimit;
			if(pos3.ek>0)
				pos3.ek_sum += pos3.ek;
		}
		else if(pos3.ek_sum>=pos3.ek_sumlimit)
		{
			pos3.ek_sum=pos3.ek_sumlimit;
			if(pos3.ek<0)
				pos3.ek_sum += pos3.ek;
		}
    pwm = pos3.kp*pos3.ek + pos3.ki*pos3.ek_sum+pos3.kd*(pos3.ek-pos3.ek_1);   //位置式pos1控制器
    pos3.ek_1 = pos3.ek;   //保存上一次偏差
    if(pwm > pos3.limit)
    {
      pwm =  pos3.limit;
    }
    else if(pwm < -pos3.limit)
    {
      pwm =  -pos3.limit;
    }
    return pwm;
}

float Position4(float Encoder,float Target)
{
    float pwm = 0;
    pos4.ek = Target - Encoder; // 计算当前误差
    pos4.ek_sum += pos4.ek;      //求出偏差的积分
    pwm = pos4.kp*pos4.ek + pos4.ki*pos4.ek_sum+pos4.kd*(pos4.ek-pos4.ek_1);   //位置式pos1控制器
    pos4.ek_1 = pos4.ek;   //保存上一次偏差
    if(pwm > pos4.limit)
      pwm =  pos4.limit;
    else if(pwm < -pos4.limit)
      pwm =  -pos4.limit;
    return pwm;
}

float Position5(float Encoder,float Target)
{
    float pwm = 0;
    pos5.ek = Target - Encoder; // 计算当前误差
    pos5.ek_sum += pos5.ek;      //求出偏差的积分
    pwm = pos5.kp*pos5.ek + pos5.ki*pos5.ek_sum+pos5.kd*(pos5.ek-pos5.ek_1);   //位置式pos1控制器
    pos5.ek_1 = pos5.ek;   //保存上一次偏差
    if(pwm > pos5.limit)
      pwm =  pos5.limit;
    else if(pwm < -pos5.limit)
      pwm =  -pos5.limit;
    return pwm;
}

float Position6(float Encoder,float Target)
{
    float pwm = 0;
    pos6.ek = Target - Encoder; // 计算当前误差
    pos6.ek_sum += pos6.ek;      //求出偏差的积分
    pwm = pos6.kp*pos6.ek + pos6.ki*pos6.ek_sum+pos6.kd*(pos6.ek-pos6.ek_1);   //位置式pos1控制器
    pos6.ek_1 = pos6.ek;   //保存上一次偏差
    if(pwm > pos6.limit)
      pwm =  pos6.limit;
    else if(pwm < -pos6.limit)
      pwm =  -pos6.limit;
    return pwm;
}

void pos1_Init()
{
    pos1.kp = 19;//21
    pos1.ki = 0;
    pos1.kd = 0;
    pos1.limit = 10000;
    pos1.ek = 0;
    pos1.ek_1 = 0;
    pos1.ek_sum = 0;
		pos1.ek_sumlimit=700000;
}

void pos2_Init()
{
    pos2.kp = 50;//48
    pos2.ki = 0;
    pos2.kd = 8;//5
    pos2.limit = 100000;
    pos2.ek = 0;
    pos2.ek_1 = 0;
    pos2.ek_sum = 0;
}

void pos3_Init()
{
    pos3.kp = 0.2;//0.22
    pos3.ki = 0.0003;//0.0005
    pos3.kd = 0;
    pos3.limit = 180;
    pos3.ek = 0;
    pos3.ek_1 = 0;
    pos3.ek_sum = 0;
		pos3.ek_sumlimit=2000;
}

void pos4_Init(void)
{
    pos4.kp =0.29;
    pos4.ki =0;
    pos4.kd = 0.013;
    pos4.limit =0;
    pos4.ek = 0;
    pos4.ek_1 = 0;
    pos4.ek_sum = 0;
}

void pos5_Init(void)
{
    pos5.kp = 0.1;
    pos5.ki = 0;
    pos5.kd = 0.01;
    pos5.limit = 5000;
    pos5.ek = 0;
    pos5.ek_1 = 0;
    pos5.ek_sum = 0;
}

void pos6_Init(void)
{
    pos6.kp = 600;
    pos6.ki = 5;//2
    pos6.kd = 0;	
    pos6.limit = 10000;
    pos6.ek = 0;
    pos6.ek_1 = 0;
    pos6.ek_sum = 0;
}