#ifndef _ANO_UWB_H_
#define _ANO_UWB_H_

//==����

//
#include "Ano_Filter.h"
#include "Ano_Math.h"
#include "Ano_Imu.h"
#include "Ano_FcData.h"
//==����
typedef struct
{
	u8 init_ok;
	u8 online;
	
	float ref_dir[2];
	float raw_data_loc[3];
	float raw_data_vel[3];	
	float w_dis_cm[3];
	float w_vel_cmps[3];

}_uwb_data_st;


//==��������
extern _uwb_data_st uwb_data;


//==��������
void Ano_UWB_Get_Byte(u8 data);
void Ano_UWB_Get_Data_Task(u8 dT_ms);
void Ano_UWB_Data_Calcu_Task(u8 dT_ms);





#endif





