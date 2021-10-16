#ifndef __ANO_OPMV_LINETRACKING_CTRL_H
#define __ANO_OPMV_LINETRACKING_CTRL_H

//==����
//#include "sysconfig.h"
#include "Ano_FcData.h"

//==����
typedef struct
{
	//
	u8 target_loss;	
	//
	s16 opmv_pos;
	s16 r2pixel_val;

	//
	float decou_pos_pixel;
	float ground_pos_err_h_cm;
	float ground_pos_err_d_h_cmps;
	

	//
	float exp_velocity_h_cmps[2];
	float exp_yaw_pal_dps;
}_ano_opmv_lt_ctrl_st;

//==��������
extern _ano_opmv_lt_ctrl_st ano_opmv_lt_ctrl;

//==��������

//static
static void ANO_LTracking_Decoupling(u8 *dT_ms,float rol_degs,float pit_degs);
static void ANO_LTracking_Calcu(u8 *dT_ms,s32 relative_height_cm);
//public
void ANO_LTracking_Task(u8 dT_ms);
void ANO_LTracking_Ctrl(u8 *dT_ms,u8 en);




#endif
