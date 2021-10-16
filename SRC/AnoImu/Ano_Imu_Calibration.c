#include "Ano_Imu_Calibration.h"
#include "Ano_Filter.h"
#include "Ano_Math.h"
#include "string.h"



_ano_imu_cali_st st_imu_cali;
//
static u16 acc_sum_cnt,gyr_sum_cnt;
static float acc_sum_av[3],gyr_sum_av[3];
//
static float acc_tmp[3],gyr_tmp[3];
static float acc_delta_tmp[3],gyr_delta_tmp[3];
static float acc_delta_length,gyr_delta_length;
static float hold_time_ms[2];
//设定参数：
#define ACC_CKS_VAL  25.0f
#define GYR_CKS_VAL  3.0f
#define CALI_AV_N 1000   //
/*检测加速度计静止，用于获取数据*/
void AccGyrStableCheck_Services(float dT_s,float acc_cmss_in[3],float gyr_dps_in[3])
{
	//==
	for(u8 i=0;i<3;i++)
	{
		//
		LPF_1_(2.0f,dT_s,(acc_cmss_in[i]),acc_tmp[i]);
		LPF_1_(2.0f,dT_s,(gyr_dps_in[i] ),gyr_tmp[i]);
		//
		acc_delta_tmp[i] = acc_cmss_in[i] - acc_tmp[i];
		gyr_delta_tmp[i] = gyr_dps_in[i]  - gyr_tmp[i];
	}
	//
	acc_delta_length = my_3_norm(acc_delta_tmp[0],acc_delta_tmp[1],acc_delta_tmp[2]);
	gyr_delta_length = my_3_norm(gyr_delta_tmp[0],gyr_delta_tmp[1],gyr_delta_tmp[2]);
	//==
	//检测acc是否稳定
	if(acc_delta_length>ACC_CKS_VAL)
	{
		hold_time_ms[0] = 0;
		st_imu_cali.acc_stable = 0;
	}
	else
	{
		if(hold_time_ms[0]<200)
		{
			hold_time_ms[0] += 1e3f *(dT_s);
		}
		else
		{
			st_imu_cali.acc_stable = 1;
		}
	}
	//检测gyr是否稳定
	if(gyr_delta_length>GYR_CKS_VAL)
	{
		hold_time_ms[1] = 0;
		st_imu_cali.gyr_stable = 0;
	}
	else
	{
		if(hold_time_ms[1]<200)
		{
			hold_time_ms[1] += 1e3f *(dT_s);
		}
		else
		{
			st_imu_cali.gyr_stable = 1;
		}
	}
}
/*陀螺仪AV_N个数据平均值*/
u8 GetGyrAvValue(u8 en,vec3_f gyr_raw_in,vec3_f gyr_av_out)
{

	if(en == 1)
	{
		if(gyr_sum_cnt<CALI_AV_N)
		{
			//
			gyr_sum_cnt++;
			//
			for(u8 i=0;i<3;i++)
			{
				gyr_sum_av[i] += gyr_raw_in[i]/CALI_AV_N;
			}
			//
			return 0;
		}
		else
		{
			//
			gyr_sum_cnt = 0;
			//
//			test_addr = gyr_av_out;
			//memcpy(gyr_av_out,gyr_sum_av,12);
			
			//输出结果
			for(u8 i=0;i<3;i++)
			{
				//
				*(gyr_av_out+i) = gyr_sum_av[i];
				//
				gyr_sum_av[i] = 0;
			}
			return 1;
		}
	}
	else
	{
		gyr_sum_cnt = 0;
		for(u8 i=0;i<3;i++)
		{
			gyr_sum_av[i] = 0;
		}
		//
		return 0;
	}
}
/*加速度计AV_N个数据平均值*/
u8 GetAccAvValue(u8 en,vec3_f acc_raw_in,vec3_f acc_av_out)
{

	if(en == 1)
	{
		if(acc_sum_cnt<CALI_AV_N)
		{
			//
			acc_sum_cnt++;
			//
			for(u8 i=0;i<3;i++)
			{
				acc_sum_av[i] += acc_raw_in[i]/CALI_AV_N;
			}
			//
			return 0;
		}
		else
		{
			//
			acc_sum_cnt = 0;
			//输出结果
			for(u8 i=0;i<3;i++)
			{
				//
				*(acc_av_out+i) = 	acc_sum_av[i];
				//
				acc_sum_av[i] = 0;
			}
			return 1;
		}
	}
	else
	{
		acc_sum_cnt = 0;
		for(u8 i=0;i<3;i++)
		{
			acc_sum_av[i] = 0;
		}
		//
		return 0;
	}
}

