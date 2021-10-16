#include "stm32f4xx.h"
#include "Ano_FlightCtrl.h"
static void UltraSonic_Data_Analysis(u8 *buf_data,u8 len);
void UltraSonic_Byte_Get(u8 bytedata);
//void ultras_tesk();
typedef struct
{
	u8 offline;
	u16 distance;
}_UltraSonic_data_st;
//Êý¾ÝÉùÃ÷
extern _UltraSonic_data_st uls;
void ct_state_task(void);
u8 ct_state_task1(void);
void ct_state_task2(void);
void ct_state_task3(void);
void ultras_tesk(void);
u16 ultra_pid(u16 ep_distance,u16 distance);
u16 filter(u16 NEW_distance,float k);
void ult_distance_hold(float ep_alt,float alt_data);
u8 filter_common(u8 nowData,float a);
u8 Median_sliding_filter(u8 value);
u8 Average_sliding_filter(u8 value);

float pitch_hold_pid(float ep_distance,float distance,float php_p,float php_i,float php_d);