#include "Ano_MotorCtrl.h"
#include "Ano_Math.h"
#include "ANO_RC.h"
#include "Drv_icm20602.h"
#include "Drv_spl06.h"
#include "Ano_Imu.h"
#include "Drv_pwm_out.h"
#include "Ano_MotionCal.h"
#include "Ano_Filter.h"
#include "Ano_Navigate.h"

/*
四轴：
      机头
   m2     m1
     \   /
      \ /
      / \
     /   \
   m3     m4
      屁股
*/
s16 motor[MOTORSNUM];
s16 motor_step[MOTORSNUM];
//float motor_lpf[MOTORSNUM];

static u16 motor_prepara_cnt;
_mc_st mc;
u16 IDLING;//10*Ano_Parame.set.idle_speed_pwm  //200
void Motor_Ctrl_Task(u8 dT_ms)
{
	u8 i;
	
//	if(flag.taking_off)
//	{
//		flag.motor_preparation = 1;
//		motor_prepara_cnt = 0;			
//	}
	
	if(flag.unlock_sta)
	{		
		IDLING = 10*LIMIT(Ano_Parame.set.idle_speed_pwm,0,30);
		
		if(flag.motor_preparation == 0)
		{
			motor_prepara_cnt += dT_ms;
			
			if(flag.motor_preparation == 0)
			{			
				if(motor_prepara_cnt<250)
				{
					motor[m1] = IDLING;
				}
				else if(motor_prepara_cnt<500)
				{
					motor[m2] = IDLING;
				}
				else if(motor_prepara_cnt<750)
				{
					motor[m3] = IDLING;
				}	
				else if(motor_prepara_cnt<1000)
				{	
					motor[m4] = IDLING;
				}
#if (MOTORSNUM >= 6)
				else if(motor_prepara_cnt<1250)
				{
					motor[m5] = IDLING;
				}
				else if(motor_prepara_cnt<1500)
				{
					motor[m6] = IDLING;
				}
#endif				
				else
				{
					flag.motor_preparation = 1;
					motor_prepara_cnt = 0;
				}
			}
			
		}	
	}
	else
	{
		flag.motor_preparation = 0;
	}
	

			
	if(flag.motor_preparation == 1)
	{	
		
		
#if (MOTORSNUM == 4)
/*
四轴：
      机头
   m2     m1
     \   /
      \ /
      / \
     /   \
   m3     m4
*/    

		motor_step[m1] = mc.ct_val_thr  +mc.ct_val_yaw -mc.ct_val_rol +mc.ct_val_pit;
		motor_step[m2] = mc.ct_val_thr  -mc.ct_val_yaw +mc.ct_val_rol +mc.ct_val_pit;
		motor_step[m3] = mc.ct_val_thr  +mc.ct_val_yaw +mc.ct_val_rol -mc.ct_val_pit;
		motor_step[m4] = mc.ct_val_thr  -mc.ct_val_yaw -mc.ct_val_rol -mc.ct_val_pit;
#elif (MOTORSNUM == 6)	
/*
六轴：
			机头
     m2   m1
      \  /
 m3----------m6
      /  \
     m4   m5
*/
//六轴  sqrt(3)/2 = 0.866f

		motor_step[m1] = mc.ct_val_thr -mc.ct_val_rol *0.866f +mc.ct_val_pit +mc.ct_val_yaw;
		motor_step[m2] = mc.ct_val_thr +mc.ct_val_rol *0.866f +mc.ct_val_pit -mc.ct_val_yaw;
		motor_step[m3] = mc.ct_val_thr +mc.ct_val_rol *0.866f                +mc.ct_val_yaw;
		motor_step[m4] = mc.ct_val_thr +mc.ct_val_rol *0.866f -mc.ct_val_pit -mc.ct_val_yaw;	
		motor_step[m5] = mc.ct_val_thr -mc.ct_val_rol *0.866f -mc.ct_val_pit +mc.ct_val_yaw;
		motor_step[m6] = mc.ct_val_thr -mc.ct_val_rol *0.866f                -mc.ct_val_yaw;			


#endif		
	
		for(i=0;i<MOTORSNUM;i++)
		{	
			motor_step[i] = LIMIT(motor_step[i],IDLING,1000);
//			motor_lpf[i] += 0.5f *(motor_step[i] - motor_lpf[i]) ;		
			
		}
		


	}
	
	for(i=0;i<MOTORSNUM;i++)
	{
		if(flag.unlock_sta)
		{
			if(flag.motor_preparation == 1)
			{
				motor[i] = LIMIT(motor_step[i],IDLING,999);
			}
	
		}
		else
		{		
			motor[i] = 0;
		}	

	}

	//配置输出
//	for(u8 i =0;i<4;i++)
//	{
		SetPwm(motor);
//	}

//#define Cali_Set_ESC
#ifdef Cali_Set_ESC
	//配置输出
	for(u8 i =0;i<4;i++)
	{
		motor[i] = CH_N[CH_THR]+500;
		SetPwm(motor);
	}
	
#endif


}



