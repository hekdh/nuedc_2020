#ifndef _DRV_SPI_H_
#define _DRV_SPI_H_
#include "stm32f4xx.h"

void Drv_SPI2_init(void);
u8 Drv_SPI2_RW(u8 dat);
void Drv_SPI2_Transmit(uint8_t *pData, uint16_t Size);
void Drv_SPI2_Receive(uint8_t *pData, uint16_t Size);
#endif
