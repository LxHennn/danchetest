#include "turn.h"
#include "zf_common_headfile.h"

#define PWM_CH1             		(PWM2_MODULE3_CHB_D3)
#define PWM_CH2                 (PWM2_MODULE3_CHA_D2)
#define SERVO_MOTOR_PWM         (PWM4_MODULE2_CHA_C30)
#define SERVO_MOTOR_FREQ        (50)
#define SERVO_MOTOR_DUTY(x)     ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

void LEFT(double dajiao)
{
	if(duoji<dajiao)
	{
		//e=-2-duoji/10;
		e=3.6;
		duoji+=0.1;
	}
	pwm_set_duty(SERVO_MOTOR_PWM,SERVO_MOTOR_DUTY(93-duoji));
}	

void LEFTBACK(void)
{
	if(duoji>0)
	{
		//e=-2-duoji/20;
		e=1.6;
		duoji-=0.1;
	}
	else e=1.6;
	pwm_set_duty(SERVO_MOTOR_PWM,SERVO_MOTOR_DUTY(93-duoji));
}

void RIGHT(double dajiao)
{
	if(duoji>dajiao)
	{
		//e=-3+duoji/10;
		e=0.6;
		duoji-=0.1;
	}
	pwm_set_duty(SERVO_MOTOR_PWM,SERVO_MOTOR_DUTY(93-duoji));
}

void RIGHTBACK(void)
{
	if(duoji<0)
	{
		//e=-2+duoji/10;
		e=1.6;
		duoji+=0.1;
	}
	else e=1.6;
	pwm_set_duty(SERVO_MOTOR_PWM,SERVO_MOTOR_DUTY(93-duoji));
}

void stop(void)
{
	pwm_set_duty(PWM_CH1,0);
	pwm_set_duty(PWM_CH2,0);
	interrupt_global_disable();
}