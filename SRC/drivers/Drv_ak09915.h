#ifndef _AK09915_H_
#define	_AK09915_H_
#include "stm32f4xx.h"


typedef struct 
{
	u8 st1;
	s16 x;
	s16 y;
	s16 z;
	u8 tmps;
	u8 st2;
}__attribute__ ((__packed__)) _ak09915_st;// __packed 

typedef union 
{
	u8 u8_buffer[9];
	_ak09915_st st_data;
}_ak09915_data_un;

extern _ak09915_data_un ak09915Buf;

u8 DrvAK09915Check(void);
//
void DrvAk09915Init(void);
//
void DrvAk09915Read(void);

#endif

