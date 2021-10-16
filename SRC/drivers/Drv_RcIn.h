#ifndef _DRV_RCIN_H_
#define _DRV_RCIN_H_
#include "include.h"
//
void DrvRcSbusInit(void);
void Sbus_IRQH(void);
void DrvRcPpmInit(void);
void PPM_IRQH(void);
#endif
