#ifndef __serial_task_H
#define __serial_task_H
#include "main.h"

extern uint8_t view_rxdata;
extern uint8_t screen_rxdata;
extern uint8_t color,last_color;
extern float blob_float_data[2];
extern uint16_t blob_data[2];

extern float last_blob_x,last_blob_y;
extern float color_all_pos[3][2];
extern float color_all_pos_last[3][2];
extern uint8_t view_rxbuf2[12];
void cal_qrcode(void);

extern uint8_t serial2_choose;

extern uint8_t red_area,green_area,blue_area,last_red_area,last_green_area,last_blue_area;

extern uint8_t ahand_flag,behind_flag,left_flag,right_flag,turn_left_flag,turn_right_flag,stop_flag,run_circle_flag,method2_flag,position_flag,
	             grab_red_flag,grab_green_flag,grab_blue_flag,
							 grab_position1_flag,grab_position2_flag,grab_position3_flag,arm_up_flag,arm_down_flag,arm_up_reset_flag,upload_flag,
							grab_to_own1_flag,start_flag,set_material_flag,turn_self_zero_flag;
extern uint16_t qrcode_data[2];
extern uint16_t qrcode_data[2];
void cal_blob_area(void);
uint16_t cal_data(uint8_t low,uint8_t high);
void show_qrocde(int16_t num1,int16_t num2);



void view_task(void);
void screen_task(void);
void data_reset(void);

#endif
