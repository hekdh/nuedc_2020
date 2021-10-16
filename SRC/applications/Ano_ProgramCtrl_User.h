#ifndef __ANO_PROGRAMCTRL_USER_H
#define __ANO_PROGRAMCTRL_USER_H

//==����

#include "Ano_FcData.h"

//==����
typedef struct
{
	//
	float vel_cmps_set_h[2];
	float vel_cmps_set_w[2];
	float vel_cmps_set_ref[2];
	//
	float vel_cmps_set_z;
	float pal_dps_set;
}_pc_user_st;
extern _pc_user_st pc_user;

//==��������


//==��������

//static


//public
void Program_Ctrl_User_Set_HXYcmps(float hx_vel_cmps,float hy_vel_cmps);
void Program_Ctrl_User_Set_Zcmps(float z_vel_cmps);
void Program_Ctrl_User_Set_YAWdps(float yaw_pal_dps);



#endif
