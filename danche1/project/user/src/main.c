/*********************************************************************************************************************
 * RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）是一个基于官�? SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 RT1064DVL6A 开源库的一部分
 *
 * RT1064DVL6A 开源库 是免费软�?
 * 您可以根据自由软件基金会发布�? GPL（GNU General Public License，即 GNU通用公共许可证）的条�?
 * �? GPL 的第3版（�? GPL3.0）或（您选择的）任何后来的版本，重新发布�?/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参�? GPL
 *
 * 您应该在收到本开源库的同时收到一�? GPL 的副�?
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明�?
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版�?
 * 许可申明英文版在 libraries/doc 文件夹下�? GPL3_permission_statement.txt 文件�?
 * 许可证副本在 libraries 文件夹下 即该文件夹下�? LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明�?
 *
 * 文件名称          main
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环�?          IAR 8.32.4 or MDK 5.33
 * 适用平台          RT1064DVL6A
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作�?                备注
 * 2022-09-21        SeekFree            first version
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "pid.h"
#include "ptf.h"
#include "math.h"
#include "kalman.h"
#include "turn.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一�? 关闭上面所有打开的文�?
// 第二�? project->clean  等待下方进度条走�?

// 本例程是开源库移植用空工程

#define PWM_CH1 (PWM2_MODULE3_CHB_D3)
#define DIR_CH1 (D1)

#define KEY1 				(C4)//(C31)
#define KEY2        (D4)//(C27) 
#define KEY3				(C26)
#define KEY4				(C31)

#define FLASH_SECTION_INDEX (127) // 存储数据用的扇区 倒数第一个扇形
#define FLASH_PAGE_INDEX (FLASH_PAGE_3)

#define ENCODER1_TIM (QTIMER3_ENCODER2)			 //(QTIMER1_ENCODER2)                                 // 编码器定时器
#define ENCODER1_PLUS (QTIMER3_ENCODER2_CH1_B18) //(QTIMER1_ENCODER2_CH1_C2)                          // 编码器计数端�?
#define ENCODER1_DIR (QTIMER3_ENCODER2_CH2_B19)	 //(QTIMER1_ENCODER2_CH2_C24)												 // 编码器方向采值端�?

#define PWM_CH2 (PWM2_MODULE3_CHA_D2)
#define DIR_CH2 (D0)

#define ENCODER_QUADDEC (QTIMER2_ENCODER1)			 // 正交编码器对应使用的编码器接口 这里使用QTIMER1的ENCODER1
#define ENCODER_QUADDEC_A (QTIMER2_ENCODER1_CH1_C3)	 // A 相对应的引脚
#define ENCODER_QUADDEC_B (QTIMER2_ENCODER1_CH2_C25) // B 相对应的引脚

#define SERVO_MOTOR_PWM (PWM4_MODULE2_CHA_C30) // 定义主板上舵机对应引脚
#define SERVO_MOTOR_FREQ (50)				   // 定义主板上舵机频率	50-300

#define SERVO_MOTOR_DUTY(x)  ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_MOTOR_FREQ) * (0.5 + (float)(x) / 90.0))

#define g (9.724)

pos pos1, pos2, pos3, pos4, pos5, pos6;

uint8 data_buffer[4];
uint8 data_len;

float e=1.6;
float angle_z =118, gyro_z;

int i=0,j=1, p=0,q=1,flag_1 = 0, t = 0,flagd=0;
double angle_now=0;
int flag1;
int num;
double caidian[100];

float duoji=0;
float L = 0.2, d = 0.07, v=0.8; // L是前后轮触地点距离，d是重心距离后轮的距离，V是前进速度
float ax;
float temp;
float angle;
void servobalance()
{
	ax=pos4.kp*(angle-e)-pos4.kd*icm20602_gyro_transition(icm20602_gyro_x);
	temp=-ax/v/v*d;
	if(temp<-1) temp=-1;
  else if(temp>1) temp=1;
	duoji=atan((L/d)*tan(asin(temp)))*180/PI;
	if(duoji>40)duoji=40;
	else if(duoji<-40)duoji=-40;
	pwm_set_duty(SERVO_MOTOR_PWM,SERVO_MOTOR_DUTY(93-duoji));
}

int main(void)
{
	clock_init(SYSTEM_CLOCK_600M); // 不可删除
	debug_init();				   // 调试端口初始化

	// 此处编写用户代码 例如外设初始化代码等
	icm20602_init();
	gps_init();
	
  flash_init();
	
	wireless_uart_init();
	exti_init(KEY1, EXTI_TRIGGER_LOW);
	exti_init(KEY2, EXTI_TRIGGER_LOW);
	exti_init(KEY3, EXTI_TRIGGER_LOW);
	exti_init(KEY4, EXTI_TRIGGER_LOW);

	pit_ms_init(PIT_CH0, 1);
	pit_ms_init(PIT_CH1, 5);
	pit_ms_init(PIT_CH2, 10);
	pit_ms_init(PIT_CH3, 200);
 
	//lingpiao();
	
	pos1_Init();
	pos2_Init();
	pos3_Init();
	pos4_Init();
	pos5_Init();
	pos6_Init();

	pwm_init(PWM_CH1, 1000, 0);
	pwm_init(PWM_CH2, 1000, 0);
	pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, SERVO_MOTOR_DUTY(93));
	gpio_init(DIR_CH1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
	gpio_init(DIR_CH2, GPO, GPIO_HIGH, GPO_PUSH_PULL);
	
	encoder_dir_init(ENCODER1_TIM, ENCODER1_PLUS, ENCODER1_DIR);
	encoder_dir_init(ENCODER_QUADDEC, ENCODER_QUADDEC_A, ENCODER_QUADDEC_B);

	if (flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX)) // 判断是否有数据
	{
		flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX); // 擦除这一数据
	}
	flash_buffer_clear();
	
	
	interrupt_set_priority(PIT_IRQn, 1);
	interrupt_disable(PIT_IRQn);
	//interrupt_enable(PIT_IRQn);
  
	while (1)
	{	
		data_len = wireless_uart_read_buff(data_buffer, 1);                    // 查看是否有消息 默认缓冲区是 WIRELESS_UART_BUFFER_SIZE 总共 64 字节
		if(data_len != 0)                                                       // 收到了消息 读取函数会返回实际读取到的数据个数
		{
			if(data_buffer[0]=='A')
			{
				if (gps_tau1201_flag)
				{
					gps_tau1201_flag = 0;
					if (!gps_data_parse()) // 开始解析数据
					{
						caidian[i]= gps_tau1201.latitude;	 	 // 获取纬度
					  caidian[j]= gps_tau1201.longitude; 	 // 获取经度				
						
						for(int c=0;c<30;c++)
				    ptf("%lf%lf%d%d",caidian[i],caidian[j],i,j);
				    i+=2;
				    j+=2;
						//flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
						}
					}
			}

			if(data_buffer[0]=='S')
			{
				interrupt_enable (PIT_IRQn);
			}
			if(data_buffer[0]=='D')
				flagd=1;
			if(data_buffer[0]=='F')
				flagd=0;
			if(data_buffer[0]=='B')
			{
				i=0;
        j=1;
			}
			memset(data_buffer, 0, 4);
		}
		}
	}
extern float accr;
extern float err[3];
extern int lp;

float w, v1,v2;
float PWM, suibiande;

float encoder0,encoder1;
float encoder2;
extern Flilter gyrox, gyroy, gyroz;
extern Flilter accx, accy, accz;

float anglez0;
void pit_handler0(void)
{
	icm20602_get_acc();
	icm20602_get_gyro();
	//lingpiao();

	//if(lp>100)
	//{
		suibiande = Low_Flilter1(icm20602_gyro_transition(icm20602_gyro_x), &gyrox);
		PWM = Position1(suibiande,w)-encoder0/1000*290;
		if(PWM>10000)PWM=10000;
		else if(PWM<-10000)PWM=-10000;
		if (PWM > 0)
		{
			gpio_set_level(DIR_CH1, 0);
			pwm_set_duty(PWM_CH1, PWM);
		}
		else
		{
			gpio_set_level(DIR_CH1, 1);
			pwm_set_duty(PWM_CH1, -PWM);
		}
	//}
	
	gyro_z = icm20602_gyro_transition(icm20602_gyro_z);
	angle_z += ((gyro_z + 0.128405816) * 0.001);
	if (angle_z > 360)
		angle_z -= 360;
	if (angle_z < 0)
		angle_z += 360;
}

float dajiao;
int dajiaoflag;
float angle_err;

float angle0,run=1.5;
int time;
void pit_handler1(void)
{
	Angle_acc_cal();
	angle = 0.99f * (angle - Low_Flilter2(icm20602_gyro_transition(icm20602_gyro_x) - err[0], &gyrox) / 200) + 0.01f * accr;
	
	w = -Position2(angle,e+v1);
	
 	angle_err=Kalman.Angle-angle_now;
	if(angle_err<-180)angle_err+=360;
	else if(angle_err>180)angle_err-=360;
	dajiao=0.2*angle_err;

	if(flag1==1)
	{
		if(dajiao>0)      //左转
		{
			if(dajiao>6)
				dajiao=6;
			if(dajiao>3)
			{
				LEFT(dajiao);
				dajiaoflag=1;
			}
			else if (dajiaoflag)
				LEFTBACK();
		}
		else if(dajiao<0)
		{
			if(dajiao<-6)
				dajiao=-6;	
			if(dajiao<-3)
			{
				RIGHT(dajiao);
				dajiaoflag=1;
			}
			else if(dajiaoflag)
				RIGHTBACK();
		}
	}

//	if(time>1000&&time<2000)
//		//LEFT(10);
//		RIGHT(-10);
//	else if(time>2000)
//		//LEFTBACK();
//		RIGHTBACK();
//	time++;
	
//	if(encoder0<1&&encoder0>-1)
//	{
//		pos1.ek_sumlimit=100000;
//		pos3.ek_sumlimit=2000;
//	}
//	else
//	{
//		pos1.ek_sumlimit=700000;
//		pos3.ek_sumlimit=7000;
//	}
}

void pit_handler2(void)
{
	encoder0 = (float)encoder_get_count(ENCODER1_TIM);
	encoder0 = 0.3 * encoder0 + 0.7 * encoder1;
	encoder1 = encoder0;
	encoder_clear_count(ENCODER1_TIM);

	v1 = -Position3(encoder0, 0);

	encoder2 = (float)encoder_get_count(ENCODER_QUADDEC);
	encoder_clear_count(ENCODER_QUADDEC);
	
//ptf("%f",encoder2);

	if(flagd)
	{
		v2 = Position6(encoder2, run);
		pwm_set_duty(PWM_CH2, v2);
	}
	else 
		pwm_set_duty(PWM_CH2,0);

	if(encoder2>run+1.5)
		pos6.ek_sum=0;
	//servobalance();
	//ptf("%f%f%f%f%f",duoji,ax,angle,suibiande,icm20602_gyro_transition(icm20602_gyro_x));
	//ptf("%f%f%f",PWM,pos6.ek_sum,encoder2);
	
	if (angle > 15 || angle < -15)
		flagd=0;
}

float distance;
void pit_handler3(void)
{
	angle_z -= 0.030309;
	if (gps_tau1201_flag)
	{
		gps_tau1201_flag = 0;
		if (!gps_data_parse()) // 开始解析数据
		{
			KalmanFilter_Angle(gps_tau1201.direction, angle_z, &Kalman);
      angle_now=get_two_points_azimuth (gps_tau1201.latitude,gps_tau1201.longitude, caidian[i], caidian[j]);			
			distance=get_two_points_distance (gps_tau1201.latitude,gps_tau1201.longitude, caidian[i], caidian[j]);
			ptf("%f%f%lf%f%f%lf%lf%lf%lf%lf%d%d",angle_z,Kalman.Angle,angle_now,duoji,dajiao,distance,gps_tau1201.latitude,gps_tau1201.longitude,caidian[i],caidian[j],i,j);
			//ptf("%lf%lf", angle_now, Kalman.Angle);
			if(distance<5.3)
				flag1=1;
			if(distance<2)
			{
				i+=2;
			  j+=2;
			}
		}
	}
}

int key1;
void key1_exti_handler(void)
{
	if (exti_flag_get(KEY1))
	{
		if(key1==500)
		{
			if (gps_tau1201_flag)
			{
				gps_tau1201_flag = 0;
				if (!gps_data_parse()) // 开始解析数据
				{
					caidian[i]= gps_tau1201.latitude;	 // 获取纬度
					caidian[j]= gps_tau1201.longitude; 	 // 获取经度				
					
					for(int c=0;c<30;c++)
					ptf("%lf%lf%d%d",caidian[i],caidian[j],i,j);
					i+=2;
					j+=2;
					//flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
				}
			}
			key1=0;
		}
		else
		{
			key1++;
			system_delay_ms(1);
		}
		exti_flag_clear(KEY1);
	}
	else
		key1=0;
}

int key2;
void key2_exti_handler(void)
{
	if (exti_flag_get(KEY2))
	{
		if(key2==500)
		{
			interrupt_enable (PIT_IRQn);
			key2=0;
		}
		else
		{
			key2++;
			system_delay_ms(1);
		}
		exti_flag_clear(KEY2);
	}
	else
		key2=0;
}

int key3;
void key3_exti_handler(void)
{
	if (exti_flag_get(KEY3))
	{
		if(key3==500)
		{
			flagd=1;
			key3=0;
		}
		else
		{
			key3++;
			system_delay_ms(1);
		}
		exti_flag_clear(KEY3);
	}
	else
		key3=0;
}

int key4;
void key4_exti_handler(void)
{
	if (exti_flag_get(KEY4))
	{
		if(key4==500)
		{
			i=0;
			j=1;
			key4=0;
		}
		else
		{
			key4++;
			system_delay_ms(1);
		}
		exti_flag_clear(KEY4);
	}
	else
		key4=0;
}