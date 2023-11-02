#include "control.h"
#include "pid.h"
#include "visual communication.h"
extern TIM_HandleTypeDef htim4;
extern PID_TypeDef Pid[2];
extern Visual_Data_ MY_;
uint16_t MidPoint_X=128;
uint16_t MidPoint_Y=75;
uint8_t follow_sign=0;
uint16_t sign_num=0,bbb_num;
float follow_yaw=1900,follow_pitch=1700,yaw_pid,pitch_pid;
void Follow_Red(void)
{
	Pid_config(0,0.0028,0,0);
	Pid_config(1,0.0028,0,0);
	if(follow_sign==0)
	{
		sign_num++;
		if(sign_num>=2000)
		{
			follow_sign=1;
			sign_num=0;
		}
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, follow_yaw);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, follow_pitch);
	}
	if(follow_sign==1)
	{
		if(MidPoint_X-MY_.YAW<=0||MidPoint_X-MY_.YAW>=0)
		{
			yaw_pid=pid_calculate(&Pid[0],MY_.YAW,MidPoint_X);
		}
		else 
		{
			yaw_pid=0;
		}
		if(MidPoint_Y-MY_.PITCH<=0||MidPoint_Y-MY_.PITCH>=0)
		{
			pitch_pid=pid_calculate(&Pid[1],MY_.PITCH,MidPoint_Y);
		}
		else 
		{
			pitch_pid=0;
		}
		if(MY_.YAW==0&&MY_.PITCH==0)
		{
			yaw_pid=0;
			pitch_pid=0;
		}
		follow_yaw+=yaw_pid;
		follow_pitch-=pitch_pid;
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, follow_yaw);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, follow_pitch);
		bbb_num++;
		if(bbb_num>=2000)
		{
			bbb_num=2000;
			if(MY_.YAW!=0&&MY_.PITCH!=0)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);//bbb
			}
			else 
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);//bbb
			}
		}
	}
}


void Follow2(void)
{
	Pid_config(0,1,0.01,0);
	Pid_config(1,1,0,0);
	Pid_config(2,1,0.01,0);
	Pid_config(3,1,0,0);
	
}





