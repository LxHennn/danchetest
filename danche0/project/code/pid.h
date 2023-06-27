#ifndef _PID_H_h_
#define _PID_H_h_


typedef struct pospid
{ 
  float kp;
  float ki;
  float kd;
  float ek;     //当前误差
  float ek_1;   //上一次误差
  float ek_sum; //误差总和
  float limit;  //限幅
	float ek_sumlimit;
}pos;

extern pos pos1,pos2,pos3,pos4,pos5,pos6;

float Position1(float Encoder,float Target);
float Position2(float Encoder,float Target);
float Position3(float Encoder,float Target);
float Position4(float Encoder,float Target);
float Position5(float Encoder,float Target);
float Position6(float Encoder,float Target);
void pos1_Init();
void pos2_Init();
void pos3_Init();
void pos4_Init();
void pos5_Init();
void pos6_Init();


#endif