#include "stm32f4xx.h"
#include "Ano_FlightCtrl.h"
#include "Drv_OpenMV.h"
u8 ct_state_task1(void);
void ct_state_task2(void);
u16 get_yaw();
void task_1();
void task_2();

float PID_realize(float ActualSpeed,float SetSpeed);


