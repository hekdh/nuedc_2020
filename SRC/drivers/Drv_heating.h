#ifndef _HEATING_H_
#define _HEATING_H_

#include "stm32f4xx.h"

void Drv_HeatingInit(void);
void Drv_HeatingSet(u8 val);
void Thermostatic_Ctrl_Task(u8 dT_ms);

#endif
