#ifndef _DRV_BSP_H_
#define _DRV_BSP_H_
#include "include.h"
//
enum 
{
	ch_1_rol=0,
	ch_2_pit,
	ch_3_thr,
	ch_4_yaw,
	ch_5_aux1,
	ch_6_aux2,
	ch_7_aux3,
	ch_8_aux4,
	ch_9_aux5,	
	ch_10_aux6,	
};
typedef struct
{
	s16 ch_[10]; //	

}__attribute__ ((__packed__)) _rc_ch_st;
typedef union 
{
	u8 byte_data[20];
	_rc_ch_st st_data;
}_rc_ch_un;
typedef struct
{
	u8 sig_mode; //0==null,1==ppm,2==sbus
	//
	s16 ppm_ch[9];
	//
	s16 sbus_ch[16];
	u8 sbus_flag;
	//
	u16 signal_fre;
	u8 no_signal;
	u8 fail_safe;
	_rc_ch_un rc_ch;
	u16 signal_cnt_tmp;
	u8 rc_in_mode_tmp;
} _rc_input_st;

//==Êý¾ÝÉùÃ÷
extern _rc_input_st rc_in;


void DrvRcInputInit(void);
void DrvRcInputTask(float dT_s);
void DrvPpmGetOneCh(u16 data);
void DrvSbusGetOneByte(u8 data);
#endif
