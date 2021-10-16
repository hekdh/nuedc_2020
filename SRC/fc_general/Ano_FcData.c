#include "Ano_FcData.h"
#include "Ano_Parameter.h"

_switch_st switchs;
// _save_st save;
_flag flag;
_fc_sta_var_st fc_stv;
_sensor_hd_check_st sens_hd_check;
_yaw_ctrl yaw_ctrl; 
_time_data time_data;

void data_save(void)
{
	para_sta.save_en = !flag.unlock_sta;
	para_sta.save_trig = 1;
}


void Para_Data_Init()
{

	Ano_Parame_Read();
}
void data_init(void)
{
	yaw_ctrl.flag=0;
	time_data.start_turn_flag=0;
	time_data.take_off_flag=0;
	yaw_ctrl.ct=0;
	time_data.start_move_flag=0;
}