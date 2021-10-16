#ifndef _DRV_BMI088_H_
#define _DRV_BMI088_H_
#include "include.h"
//
typedef struct 
{
	s16 s16_acc[3];
	s16 s16_gyr[3];
	
}__attribute__ ((__packed__)) _bmi_s16_st;// __packed 

typedef union 
{
	u8 u8_buffer[12];
	_bmi_s16_st st_data;
}_bmi_data_un;

typedef struct
{
	_bmi_data_un un_ins;
	s16 temperature_100;

}_bmi_data_st;
//
extern _bmi_data_st st_bmi_data;
//
void DrvGpioSenser088CsPinInit(void);
u8 DrvBmi088Init(void);
void DrvBmi088ReadServices(void);
void DrvBmi088TemperatureRead(void);
void DrvBmi088AccelerationRead(void);

#endif
