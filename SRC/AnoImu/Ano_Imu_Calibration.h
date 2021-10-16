#ifndef _ANO_IMU_CALIBRATION_H_
#define _ANO_IMU_CALIBRATION_H_


//==����
#include "include.h"


//==����
typedef struct
{
	//
	u8 acc_stable;
	u8 gyr_stable;	
	//
	u8 acc_cali_on;
	u8 gyr_cali_on;
	//

	
}_ano_imu_cali_st;
//==��������
extern _ano_imu_cali_st st_imu_cali;

//==��������

//static


//public
void AccGyrStableCheck_Services(float dT_s,float acc_cmss_in[3],float gyr_dps_in[3]);
u8 GetGyrAvValue(u8 en,vec3_f gyr_raw_in,vec3_f gyr_av_out);
u8 GetAccAvValue(u8 en,vec3_f acc_raw_in,vec3_f acc_av_out);

	
#endif


