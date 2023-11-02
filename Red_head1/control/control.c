#include "control.h"
#include "pid.h"
#include "gpio.h"
#include "visual communication.h"
extern PID_TypeDef Pid[4];
extern Visual_Data_ A_,B_,C_,D_,MY_;
extern float Eular[3];
extern TIM_HandleTypeDef htim3,htim8;
float yaw_frist,yaw;
uint16_t TIM8_Pwm,TIM3_Pwm;
uint8_t sign_gyro=1;
//void gyro_control(void)
//{
//	if(sign_gyro==1)
//	{
//		yaw_frist=Eular[2];
//		sign_gyro=0;
//	}
//	yaw=Eular[2]-yaw_frist;
//	if(yaw<=-60)
//	{
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 1250);
//	}
//	else if(yaw>=60)
//	{
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 1750);
//	}
//	else
//	{
//		TIM8_Pwm=(yaw+60)*4.2+1250;
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, TIM8_Pwm);
//	}
//	if(Eular[0]<=-30)
//	{
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1750);
//	}
//	else if(Eular[0]>=90)
//	{
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1250);
//	}
//	else
//	{
//		TIM3_Pwm=1750-(Eular[0]+30)*4.2;
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, TIM3_Pwm);
//	}
//}
/*右减下加*/
uint16_t back_yaw=1366,back_pitch=1678;//左上复位
void Back_Origin(void)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, back_pitch);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, back_yaw);
	
}

uint16_t yaw0=1467;
uint16_t pitch0=1577;
uint16_t yaw1=1277;
uint16_t pitch1=1573;
uint16_t yaw2=1271;
uint16_t pitch2=1772;
uint16_t yaw3=1465;
uint16_t pitch3=1778;
uint16_t yaw1_c,pitch1_c,yaw2_c,pitch2_c,yaw3_c,pitch3_c,yaw4_c,pitch4_c;
uint8_t key_2_sign=0,key2_speed=1;
uint16_t key_2_signnum=0;
void Key_2(void)
{
	if(key_2_sign==0)
	{
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, yaw0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pitch0);
		key_2_signnum++;
		if(key_2_signnum>=2000)
		{
			key_2_sign=1;
			key_2_signnum=0;
			yaw1_c=yaw0;
			pitch1_c=pitch0;
		}
	}
	else if(key_2_sign==1)
	{
		key_2_signnum++;
		if(key_2_signnum>=20)
		{
			yaw1_c-=key2_speed;
			key_2_signnum=0;
		}
		if(yaw1_c<=yaw1)
		{
			key_2_signnum=0;
			key_2_sign=2;
			yaw2_c=yaw1;
			pitch2_c=pitch1;
		}
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, yaw1_c);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pitch1_c);
	}
	else if(key_2_sign==2)
	{
		key_2_signnum++;
		if(key_2_signnum>=20)
		{
			pitch2_c+=key2_speed;
			key_2_signnum=0;
		}
		if(pitch2_c>=pitch2)
		{
			key_2_signnum=0;
			key_2_sign=3;
			yaw3_c=yaw2;
			pitch3_c=pitch2;
		}
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, yaw2_c);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pitch2_c);
	}
	else if(key_2_sign==3)
	{
		key_2_signnum++;
		if(key_2_signnum>=20)
		{
			yaw3_c+=key2_speed;
			key_2_signnum=0;
		}
		if(yaw3_c>=yaw3)
		{
			key_2_signnum=0;
			key_2_sign=4;
			yaw4_c=yaw3;
			pitch4_c=pitch3;
		}
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, yaw3_c);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pitch3_c);
	}
	else if(key_2_sign==4)
	{
		key_2_signnum++;
		if(key_2_signnum>=20)
		{
			pitch4_c-=key2_speed;
			key_2_signnum=0;
		}
		if(pitch4_c<=pitch0)
		{
			key_2_signnum=0;
			key_2_sign=4;
		}
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, yaw4_c);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pitch4_c);
	}
}

uint8_t Follow_SIGN=0;
uint8_t follow_sign=0,follow1_first=0;
float follow_yaw,follow_pitch,yaw_pid,pitch_pid;
float follow_yaw_start=1366,follow_pitch_start=1669;
uint16_t follow_num=0;
void Black_Follow(void)
{
	Follow_SIGN=1;
	Pid_config(0,0.00043,0,0);
	Pid_config(1,0.00043,0,0);
	if(follow_sign==0)
	{
		follow_num++;
		follow_yaw=follow_yaw_start;
		follow_pitch=follow_pitch_start;
		if(follow_num>=1000)
		{
			follow_sign=1;
			follow_num=0;
		}
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
	}
	else if(follow_sign==1)
	{
		yaw_pid=pid_calculate(&Pid[0],MY_.YAW,A_.YAW);
		pitch_pid=pid_calculate(&Pid[0],MY_.PITCH,A_.PITCH);
		follow_yaw-=yaw_pid;
		follow_pitch+=pitch_pid;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
			if(yaw_pid>=-0.0002&&pitch_pid>=-0.0002)
			{
				follow_sign=2;
			}
	}
	else if(follow_sign==2)
	{
		yaw_pid=pid_calculate(&Pid[0],MY_.YAW,B_.YAW);
		pitch_pid=pid_calculate(&Pid[0],MY_.PITCH,B_.PITCH);
		follow_yaw-=yaw_pid;
		follow_pitch+=pitch_pid;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
		if(yaw_pid<=0.0003&&pitch_pid<=0.0003)
		{
			follow_sign=3;
		}
	}
	else if(follow_sign==3)
	{
		yaw_pid=pid_calculate(&Pid[0],MY_.YAW,C_.YAW);
		pitch_pid=pid_calculate(&Pid[0],MY_.PITCH,C_.PITCH);
		follow_yaw-=yaw_pid;
		follow_pitch+=pitch_pid;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
		if(yaw_pid<=0.0002&&pitch_pid<=0.0002)
		{
			follow_sign=4;
		}
	}
	else if(follow_sign==4)
	{
		yaw_pid=pid_calculate(&Pid[0],MY_.YAW,D_.YAW);
		pitch_pid=pid_calculate(&Pid[0],MY_.PITCH,D_.PITCH);
		follow_yaw-=yaw_pid;
		follow_pitch+=pitch_pid;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
		if(yaw_pid>=-0.00025&&pitch_pid<=0.00025)
		{
			follow_sign=5;
		}
	}
	else if(follow_sign==5)
	{
		yaw_pid=pid_calculate(&Pid[0],MY_.YAW,A_.YAW);
		pitch_pid=pid_calculate(&Pid[0],MY_.PITCH,A_.PITCH);
		follow_yaw-=yaw_pid;
		follow_pitch+=pitch_pid;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
		if(yaw_pid>=-0.0003&&pitch_pid>=-0.0003)
		{
			follow_sign=2;
		}
	}
}


/*******************************************************************************/
Visual_Data_ follow2[4],tt,xx[2],follow2_rel[4];
float x__;
uint8_t X_num=0;

void Find(void)
{
	follow2[0]=A_;
	follow2[1]=B_;
	follow2[2]=C_;
	follow2[3]=D_;
	x__=(float)((follow2[0].YAW+follow2[1].YAW+follow2[2].YAW+follow2[3].YAW)/4);
	for(int i=0;i<4;i++)
	{
		if(follow2[i].YAW<x__)
		{
			xx[X_num]=follow2[i];
			X_num++;
		}
		if(X_num==2)
		{
			X_num=0;
		}
	}
	if(xx[0].PITCH<xx[1].PITCH)
	{
		follow2_rel[0]=xx[0];
		follow2_rel[3]=xx[1];
	}
	else 
	{
		follow2_rel[0]=xx[1];
		follow2_rel[3]=xx[0];
	}
	for(int i=0;i<4;i++)
	{
		if(follow2[i].YAW>x__)
		{
			xx[X_num]=follow2[i];
			X_num++;
		}
		if(X_num==2)
		{
			X_num=0;
		}
	}
	if(xx[0].PITCH>xx[1].PITCH)
	{
		follow2_rel[1]=xx[0];
		follow2_rel[2]=xx[1];
	}
	else 
	{
		follow2_rel[2]=xx[1];
		follow2_rel[1]=xx[0];
	}
}
//Visual_Data_ follow_section[12];
//void Section(void)
//{
//	follow_section[0].YAW=((follow2_rel[0].YAW+follow2_rel[1].YAW)/2+follow2_rel[0].YAW)/2;
//	follow_section[0].PITCH=((follow2_rel[0].PITCH+follow2_rel[1].PITCH)/2+follow2_rel[0].PITCH)/2+2;
//	follow_section[1].YAW=(follow2_rel[0].YAW+follow2_rel[1].YAW)/2;
//	follow_section[1].PITCH=(follow2_rel[0].PITCH+follow2_rel[1].PITCH)/2;
//	follow_section[2].YAW=((follow2_rel[0].YAW+follow2_rel[1].YAW)/2+follow2_rel[1].YAW)/2;
//	follow_section[2].PITCH=((follow2_rel[0].PITCH+follow2_rel[1].PITCH)/2+follow2_rel[1].PITCH)/2;
//	follow_section[3].YAW=follow2_rel[1].YAW-6;
//	follow_section[3].PITCH=follow2_rel[1].PITCH;
//	
//	follow_section[4].YAW=(follow2_rel[1].YAW+follow2_rel[2].YAW)/2;
//	follow_section[4].PITCH=(follow2_rel[1].PITCH+follow2_rel[2].PITCH)/2-3;
//	follow_section[5].YAW=follow2_rel[2].YAW;
//	follow_section[5].PITCH=follow2_rel[2].PITCH-5;
//	
//	follow_section[6].YAW=((follow2_rel[2].YAW+follow2_rel[3].YAW)/2+follow2_rel[2].YAW)/2;
//	follow_section[6].PITCH=((follow2_rel[2].PITCH+follow2_rel[3].PITCH)/2+follow2_rel[2].PITCH)/2;
//	follow_section[7].YAW=(follow2_rel[2].YAW+follow2_rel[3].YAW)/2;
//	follow_section[7].PITCH=(follow2_rel[2].PITCH+follow2_rel[3].PITCH)/2;
//	follow_section[8].YAW=((follow2_rel[2].YAW+follow2_rel[3].YAW)/2+follow2_rel[3].YAW)/2;
//	follow_section[8].PITCH=((follow2_rel[2].PITCH+follow2_rel[3].PITCH)/2+follow2_rel[3].PITCH)/2;
//	follow_section[9].YAW=follow2_rel[3].YAW;
//	follow_section[9].PITCH=follow2_rel[3].PITCH;
//	
//	follow_section[10].YAW=(follow2_rel[3].YAW+follow2_rel[0].YAW)/2;
//	follow_section[10].PITCH=(follow2_rel[3].PITCH+follow2_rel[0].PITCH)/2;
//	follow_section[11].YAW=follow2_rel[0].YAW;
//	follow_section[11].PITCH=follow2_rel[0].PITCH+5;
//}

float follow2_yaw_start=1400,follow2_pitch_start=1680;
//uint8_t F2=0;
//void Black_Follow2(void)
//{
//	Follow_SIGN=2;
//	Find();
//	Section();
//	Pid_config(2,0.0006,0,0);
//	Pid_config(3,0.0006,0,0);
//	
//	if(follow_sign==0)
//	{
//		follow_num++;
//		follow_yaw=follow2_yaw_start;
//		follow_pitch=follow2_pitch_start;
//		if(follow_num>=1000)
//		{
//			follow_sign=1;
//			follow_num=0;
//		}

//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
//	}
//	else if(follow_sign==1)
//	{
//		if(F2==1)
//		{
//			Pid_config(2,0.0006,0,0);
//			Pid_config(3,0.0006,0,0);
//		}
//		yaw_pid=pid_calculate(&Pid[2],MY_.YAW,follow_section[11].YAW);
//		pitch_pid=pid_calculate(&Pid[3],MY_.PITCH,follow_section[11].PITCH);
//		follow_yaw-=yaw_pid;
//		follow_pitch+=pitch_pid;
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
//		if(yaw_pid<=0.0003&&pitch_pid>=-0.0003)
//		{
//			follow_sign=2;
//		}
//	}
//	else if(follow_sign==2)
//	{
//		Pid_config(2,0.0006,0,0);
//		Pid_config(3,0.00055,0,0);
//		yaw_pid=pid_calculate(&Pid[2],MY_.YAW,follow_section[0].YAW);
//		pitch_pid=pid_calculate(&Pid[3],MY_.PITCH,(follow_section[0].PITCH));
//		follow_yaw-=yaw_pid;
//		follow_pitch+=pitch_pid;
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
//		if(yaw_pid<=0.0005&&pitch_pid<=0.0005)
//		{
//			follow_sign=3;
//		}
//	}
//	else if(follow_sign==3)
//	{
//		Pid_config(2,0.0006,0,0);
//		Pid_config(3,0.00055,0,0);
//		yaw_pid=pid_calculate(&Pid[2],MY_.YAW,follow_section[1].YAW);
//		pitch_pid=pid_calculate(&Pid[3],MY_.PITCH,(follow_section[1].PITCH));
//		follow_yaw-=yaw_pid;
//		follow_pitch+=pitch_pid;
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
//		if(yaw_pid<=0.0004&&pitch_pid<=0.0004)
//		{
//			follow_sign=4;
//		}
//	}
//	else if(follow_sign==4)
//	{
//		Pid_config(2,0.0006,0,0);
//		Pid_config(3,0.00055,0,0);
//		yaw_pid=pid_calculate(&Pid[2],MY_.YAW,follow_section[2].YAW);
//		pitch_pid=pid_calculate(&Pid[3],MY_.PITCH,(follow_section[2].PITCH));
//		follow_yaw-=yaw_pid;
//		follow_pitch+=pitch_pid;
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
//		if(yaw_pid<=0.0004&&pitch_pid<=0.0004)
//		{
//			follow_sign=5;
//		}
//	}
//	else if(follow_sign==5)
//	{
//		Pid_config(2,0.0004,0,0);
//		Pid_config(3,0.00035,0,0);
//		yaw_pid=pid_calculate(&Pid[2],MY_.YAW,follow_section[3].YAW);
//		pitch_pid=pid_calculate(&Pid[3],MY_.PITCH,(follow_section[3].PITCH));
//		follow_yaw-=yaw_pid;
//		follow_pitch+=pitch_pid;
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
//		if(yaw_pid<=0.0004&&pitch_pid<=0.0004)
//		{
//			follow_sign=6;
//		}
//	}
//	else if(follow_sign==6)
//	{
//		Pid_config(2,0.00062,0,0);
//		Pid_config(3,0.00025,0,0);
//		yaw_pid=pid_calculate(&Pid[2],MY_.YAW,follow_section[4].YAW);
//		pitch_pid=pid_calculate(&Pid[3],MY_.PITCH,(follow_section[4].PITCH));
//		follow_yaw-=yaw_pid;
//		follow_pitch+=pitch_pid;
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
//		if(yaw_pid>=-0.0004&&pitch_pid<=0.0004)
//		{
//			follow_sign=7;
//		}
//	}
//	else if(follow_sign==7)
//	{
//		Pid_config(2,0.00025,0,0);
//		Pid_config(3,0.00035,0,0);
//		yaw_pid=pid_calculate(&Pid[2],MY_.YAW,follow_section[5].YAW);
//		pitch_pid=pid_calculate(&Pid[3],MY_.PITCH,(follow_section[5].PITCH));
//		follow_yaw-=yaw_pid;
//		follow_pitch+=pitch_pid;
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
////		if(yaw_pid>=-0.0004&&pitch_pid<=0.0004)
////		{
////			follow_sign=8;
////		}
//	}
//}

uint8_t follow_sign2;
uint16_t follow2_num;
void Black_Follow2(void)
{
	Find();
	Follow_SIGN=1;
	Pid_config(2,0.0004,0,0);
	Pid_config(3,0.0004,0,0);
	
	if(follow_sign2==0)
	{
		follow2_num++;
		follow_yaw=follow2_yaw_start;
		follow_pitch=follow2_pitch_start;
		if(follow2_num>=1000)
		{
			follow_sign2=1;
			follow2_num=0;
		}
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
	}
	else if(follow_sign2==1)
	{
		yaw_pid=pid_calculate(&Pid[2],MY_.YAW,(follow2_rel[0].YAW-2));
		pitch_pid=pid_calculate(&Pid[3],MY_.PITCH,follow2_rel[0].PITCH+6);
		follow_yaw-=yaw_pid;
		follow_pitch+=pitch_pid;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
		
//		if(yaw_pid<=0.00020&&pitch_pid>=-0.00020)
//		{
//			follow_sign2=2;
//		}
		follow2_num++;
		if(follow2_num>=8000)
		{
			follow2_num=0;
			follow_sign2=2;
		}
	}
	else if(follow_sign2==2)
	{
		Pid_config(2,0.0002,0,0);
		Pid_config(3,0.0002,0,0);
		yaw_pid=pid_calculate(&Pid[2],MY_.YAW,follow2_rel[1].YAW-3);
		pitch_pid=pid_calculate(&Pid[3],MY_.PITCH,follow2_rel[1].PITCH-3);
		follow_yaw-=yaw_pid;
		follow_pitch+=pitch_pid;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
		if(yaw_pid<=0.00020&&pitch_pid<=0.00020)
		{
			follow_sign2=3;
		}
	 }
	else if(follow_sign2==3)
	{
		Pid_config(2,0.00043,0,0);
		Pid_config(3,0.00043,0,0);
		yaw_pid=pid_calculate(&Pid[2],MY_.YAW,follow2_rel[2].YAW+2);
		pitch_pid=pid_calculate(&Pid[3],MY_.PITCH,follow2_rel[2].PITCH-2);
		follow_yaw-=yaw_pid;
		follow_pitch+=pitch_pid;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
		if(yaw_pid>=-0.0004&&pitch_pid<=0.0004)
		{
			follow_sign2=4;
		}
	 }
	else if(follow_sign2==4)
	{
		Pid_config(2,0.0002,0,0);
		Pid_config(3,0.00025,0,0);
		yaw_pid=pid_calculate(&Pid[2],MY_.YAW,follow2_rel[3].YAW+2);
		pitch_pid=pid_calculate(&Pid[3],MY_.PITCH,follow2_rel[3].PITCH+2);
		follow_yaw-=yaw_pid;
		follow_pitch+=pitch_pid;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
		if(yaw_pid>=-0.0004&&pitch_pid<=-0.0004)
		{
			follow_sign2=5;
		}
	 }
	else if(follow_sign2==5)
	{
		Pid_config(2,0.00043,0,0);
		Pid_config(3,0.00043,0,0);
		yaw_pid=pid_calculate(&Pid[2],MY_.YAW,follow2_rel[0].YAW);
		pitch_pid=pid_calculate(&Pid[3],MY_.PITCH,follow2_rel[0].PITCH);
		follow_yaw-=yaw_pid;
		follow_pitch+=pitch_pid;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
		if(yaw_pid>=-0.0004&&pitch_pid<=0.0004)
		{
			follow_sign2=6;
		}
	 }
}






extern uint8_t KEY0,KEY1,KEY2,KEY3,KEY_RED;
void Find_Point(void)
{
	if(KEY0==1)
	{
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, yaw3);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pitch3);
	}
	else if(KEY1==1)
	{
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, yaw0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pitch0);
	}
	else if(KEY2==1)
	{
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, yaw1);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pitch1);
	}
	else if(KEY3==1)
	{
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, yaw2);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pitch2);
	}

}

extern uint8_t KEY0,KEY1,KEY2,KEY3,KEY3_2,KEY3_Stop;
uint8_t sign_key3=0;
void Key_3(void)
{
//	if(follow_sign==6)
//	{
//		KEY3_2=1;
//		KEY0=KEY1=KEY2=KEY3=KEY_RED=0;
//		sign_key3=1;
//	}
//	else
//	{
		KEY3=1;
		KEY0=KEY1=KEY2=KEY3_2=KEY_RED=0;
//	}
}

//extern Visual_Data_ MID_1,A_1,B_1,C_1,D_1,MY_1;
//void Back_Origin_V(void)
//{
//	Pid_config(0,0.0005,0,0);
//	Pid_config(1,0.0005,0,0);
//	yaw_pid=pid_calculate(&Pid[0],MY_.YAW,MID_1.YAW);
//	pitch_pid=pid_calculate(&Pid[1],MY_.PITCH,MID_1.PITCH);
//	follow_yaw+=yaw_pid;
//	follow_pitch-=pitch_pid;
//	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
//	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
//}



void Black_Follow3(void)
{
	Find();
	Follow_SIGN=1;
	Pid_config(2,0.0004,0,0);
	Pid_config(3,0.0004,0,0);
	
	if(follow_sign2==0)
	{
		follow2_num++;
		follow_yaw=follow2_yaw_start;
		follow_pitch=follow2_pitch_start;
		if(follow2_num>=1000)
		{
			follow_sign2=1;
			follow2_num=0;
		}
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
	}
	else if(follow_sign2==1)
	{
		yaw_pid=pid_calculate(&Pid[2],MY_.YAW,(follow2_rel[0].YAW-2));
		pitch_pid=pid_calculate(&Pid[3],MY_.PITCH,follow2_rel[0].PITCH+6);
		follow_yaw-=yaw_pid;
		follow_pitch+=pitch_pid;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
		
//		if(yaw_pid<=0.00020&&pitch_pid>=-0.00020)
//		{
//			follow_sign2=2;
//		}
		follow2_num++;
		if(follow2_num>=10000)
		{
			follow2_num=0;
			follow_sign2=2;
		}
	}
	else if(follow_sign2==2)
	{
		Pid_config(2,0.0004,0,0);
		Pid_config(3,0.00053,0,0);
		yaw_pid=pid_calculate(&Pid[2],MY_.YAW,follow2_rel[1].YAW-3);
		pitch_pid=pid_calculate(&Pid[3],MY_.PITCH,follow2_rel[1].PITCH-3);
		follow_yaw-=yaw_pid;
		follow_pitch+=pitch_pid;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
		if(yaw_pid<=0.00040&&pitch_pid<=0.00040)
		{
			follow_sign2=3;
		}
	 }
	else if(follow_sign2==3)
	{
		Pid_config(2,0.00043,0,0);
		Pid_config(3,0.00043,0,0);
		yaw_pid=pid_calculate(&Pid[2],MY_.YAW,follow2_rel[2].YAW+2);
		pitch_pid=pid_calculate(&Pid[3],MY_.PITCH,follow2_rel[2].PITCH-2);
		follow_yaw-=yaw_pid;
		follow_pitch+=pitch_pid;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
		if(yaw_pid>=-0.0004&&pitch_pid<=0.0004)
		{
			follow_sign2=4;
		}
	 }
	else if(follow_sign2==4)
	{
		Pid_config(2,0.0004,0,0);
		Pid_config(3,0.00055,0,0);
		yaw_pid=pid_calculate(&Pid[2],MY_.YAW,follow2_rel[3].YAW+2);
		pitch_pid=pid_calculate(&Pid[3],MY_.PITCH,follow2_rel[3].PITCH+2);
		follow_yaw-=yaw_pid;
		follow_pitch+=pitch_pid;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
		if(yaw_pid>=-0.0004&&pitch_pid>=-0.0004)
		{
			follow_sign2=5;
		}
	 }
	else if(follow_sign2==5)
	{
		Pid_config(2,0.00047,0,0);
		Pid_config(3,0.00043,0,0);
		yaw_pid=pid_calculate(&Pid[2],MY_.YAW,follow2_rel[0].YAW);
		pitch_pid=pid_calculate(&Pid[3],MY_.PITCH,follow2_rel[0].PITCH);
		follow_yaw-=yaw_pid;
		follow_pitch+=pitch_pid;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, follow_yaw);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, follow_pitch);
		if(yaw_pid<=0.0002&&pitch_pid>=-0.0002)
		{
			follow_sign2=2;
		}
	 }
}












