/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONFIG_H
#define __CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/***************����******************/
#define ANGLE_TO_RADIAN 0.01745329f //*0.01745 = /57.3	�Ƕ�ת����
/***********************************************/

#define ANO_DT_USE_NRF24l01
/***********************************************/



#define SP_EST_DRAG 1.0f
#define BARO_WIND_COMP 0.10f


/***********************************************/
//==
//GYR_ACC_FILTER �������·�Χ�ο�
//500KV���� 0.15f~0.2f
//500~2000KV 0.2f~0.3f
//2000kv���� 0.3f-0.5f
//==
//FINAL_P �������·�Χ�ο�
//500KV���� 0.4f����
//500~2000KV 0.4f~0.3f
//2000kv���� 0.3f-0.2f
//==
#define GYR_ACC_FILTER 0.22f //�����Ǽ��ٶȼ��˲�ϵ��
#define FINAL_P 			 0.35f  //������������ϵ��

#define MOTOR_ESC_TYPE 1  //2��ĳЩ��ˢ�����ɲ���ĵ��(��������)  1����ˢ�������ɲ���ĵ��(Ĭ��)��
#define MOTORSNUM 4

#define BAT_LOW_VOTAGE 3250    //mV
#define FLOAW_MAX_HEIGHT  450
#define FLOW_ROLL_CONDITION 8 //   0-12

#define APP_ROLL_CH CH_PIT //app����

#define MAX_ANGLE     25.0f
//#define MAX_ANGLE_ROL 25.0f //�Ƕ�
//#define MAX_ANGLE_PIT 25.0f //�Ƕ�

#define MAX_SPEED_ROL 200  //�Ƕ�ÿ��
#define MAX_SPEED_PIT 200  //�Ƕ�ÿ��
#define MAX_SPEED_YAW 250  //�Ƕ�ÿ��

#define MAX_ROLLING_SPEED 1600  //�Ƕ�ÿ��

#define MAX_SPEED 500 //���ˮƽ�ٶȣ�����ÿ�� cm/s

#define MAX_Z_SPEED_UP 350 //����ÿ�� cm/s
#define MAX_Z_SPEED_DW 250 //����ÿ�� cm/s

#define MAX_EXP_XY_ACC   500 //����ÿƽ���� cm/ss
#define MAX_EXP_Z_ACC    600 //����ÿƽ���� cm/ss

#define CTRL_1_INTE_LIM 250 //���ٶȻ������޷� �����

#define ANGULAR_VELOCITY_PID_INTE_D_LIM 300/FINAL_P  
#define X_PROPORTION_X_Y 1.0f //proportion
#define ROLL_ANGLE_KP 10.0f   //�����Ƕ�kp

#define MAX_THR_SET    85  //������Űٷֱ� %
#define THR_INTE_LIM_SET   70  //���Ż��ְٷֱ� % 

//#define MAX_THR       MAX_THR_SET/FINAL_P   
#define THR_INTE_LIM   THR_INTE_LIM_SET/FINAL_P  

#define THR_START      35  //����������ٷֱ� %

#define LAND_ACC              500  //��½���ٶȼ��
#define LAND_ACC_DELTA        300  //��½���ٶȱ仯�����


#define BARO_FIX -0                          //��ѹ�ٶȻ����������ֵ/CM����
//#define AUTO_TAKE_OFF_HEIGHT 50  //�Զ���ɸ߶�
//#define HEIGHT_FIX           0               //��ѹ�߶ȹ̶�����/CM����



#endif


