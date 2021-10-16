#ifndef _INIT_H_
#define _INIT_H_

#include "stm32f4xx.h"

u8 All_Init(void);
void DrvGpioCsPinCtrlBmi088Acc(u8 ena);
void DrvGpioCsPinCtrlBmi088Gyr(u8 ena);

extern u8 Init_Finish;
extern u8 of_init_type;
#endif
