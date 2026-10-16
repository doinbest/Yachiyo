#include "stm32f4xx.h"                  // Device header
#include "serial.h"
#include "delay.h"
uint8_t view_rxdata;
uint8_t view_rxbuf[5];
uint16_t blob_data[2];
float blob_float_data[2];
uint8_t last_color,color_temp,color;
float last_blob_x,last_blob_y;
uint8_t view_rxbuf2[255];
float color_all_pos[3][2];
float color_all_pos_last[3][2];

uint16_t qrcode_data[2];
uint8_t qrcode_buf[32];
uint8_t serial2_choose;

uint16_t cal_data(uint8_t low,uint8_t high)
{
	uint16_t result;
	result=high;
	result=result<<8;
	result|=low;
	return result;
}
uint8_t red_area,green_area,blue_area,last_red_area,last_green_area,last_blue_area;
void cal_blob_area(void)
{
		last_red_area=red_area;
		last_green_area=green_area;
		last_blue_area=blue_area;
	
	if(color_all_pos[0][0]<160&&color_all_pos[0][1]<120)
		red_area=1;
	if(color_all_pos[0][0]>160&&color_all_pos[0][1]<120)
		red_area=2;
	if(color_all_pos[0][1]>160)
		red_area=3;
	
	if(color_all_pos[1][0]<160&&color_all_pos[1][1]<120)
		green_area=1;
	if(color_all_pos[1][0]>160&&color_all_pos[1][1]<120)
		green_area=2;
	if(color_all_pos[1][1]>160)
		green_area=3;
	
	if(color_all_pos[2][0]<160&&color_all_pos[2][1]<120)
		blue_area=1;
	if(color_all_pos[2][0]>160&&color_all_pos[2][1]<120)
		blue_area=2;
	if(color_all_pos[2][1]>160)
		blue_area=3;
}
void view_task(void)
{
	static uint8_t RxState = 0;
	static uint8_t pRxPacket = 0;
	
		if (RxState == 0)
		{
			if (view_rxdata == 0xFF)
			{
				RxState = 1;
				pRxPacket = 0;
				color_temp=view_rxbuf[4];
			}
			else if(view_rxdata==0xAA)
			{
				RxState = 3;
				pRxPacket = 0;
			}
		}
		else if (RxState == 1)
		{
			view_rxbuf[pRxPacket] = view_rxdata;
			pRxPacket ++;
			if (pRxPacket >= 5)		
				RxState = 2;
		}
		else if(RxState==3)
		{
			view_rxbuf2[pRxPacket] = view_rxdata;
			pRxPacket ++;
			if (pRxPacket >= 12)		
				RxState = 4;
		}
		else if (RxState == 2)
		{
			if (view_rxdata == 0xFE)
			{
				if(color_temp!=view_rxbuf[4])
				last_color=color_temp;			
				last_blob_x=blob_float_data[0];
				last_blob_y=blob_float_data[1];
				color_temp=view_rxbuf[4];
				blob_data[0]=cal_data(view_rxbuf[0],view_rxbuf[1]);
				blob_data[1]=cal_data(view_rxbuf[2],view_rxbuf[3]);
				blob_float_data[0]=(float)blob_data[0];
				blob_float_data[1]=(float)blob_data[1];
				color=view_rxbuf[4];
				blob_float_data[0]/=100;
				blob_float_data[1]/=100;
			}			
			RxState = 0;
		}
		else if(RxState==4)
		{
			if(view_rxdata==0xBB)
			{
				color_all_pos_last[0][0]=color_all_pos[0][0];
				color_all_pos_last[0][1]=color_all_pos[0][1];
				color_all_pos_last[1][0]=color_all_pos[1][0];
				color_all_pos_last[1][1]=color_all_pos[1][1];
				color_all_pos_last[2][0]=color_all_pos[2][0];
				color_all_pos_last[2][1]=color_all_pos[2][1];
				
				color_all_pos[0][0]=cal_data(view_rxbuf2[0],view_rxbuf2[1]);
				color_all_pos[0][1]=cal_data(view_rxbuf2[2],view_rxbuf2[3]);
				color_all_pos[1][0]=cal_data(view_rxbuf2[4],view_rxbuf2[5]);
				color_all_pos[1][1]=cal_data(view_rxbuf2[6],view_rxbuf2[7]);
				color_all_pos[2][0]=cal_data(view_rxbuf2[8],view_rxbuf2[9]);
				color_all_pos[2][1]=cal_data(view_rxbuf2[10],view_rxbuf2[11]);
				
				
				cal_blob_area();
			}
			RxState=0;
		}
	HAL_UART_Receive_IT(&huart3,&view_rxdata,1);
}

void data_reset(void)
{
	color_temp=0,last_color=0,color=0;
	last_blob_x=0,last_blob_y=0;
	red_area=0,blue_area=0,green_area=0;
	blob_float_data[0]=0.0,blob_float_data[1]=0.0;
	blob_float_data[0]=0,blob_float_data[1]=0;
	color_all_pos[0][0]=0,color_all_pos[0][1]=0;
	color_all_pos[1][0]=0,color_all_pos[1][1]=0;
	color_all_pos[2][0]=0,color_all_pos[2][1]=0;
	
	color_all_pos_last[0][0]=0,color_all_pos_last[0][1]=0;
	color_all_pos_last[1][0]=0,color_all_pos_last[1][1]=0;
	color_all_pos_last[2][0]=0,color_all_pos_last[2][1]=0;
}

uint8_t screen_rxdata;
uint8_t ahand_flag,behind_flag,left_flag,right_flag,turn_left_flag,turn_right_flag,stop_flag,run_circle_flag,method2_flag,position_flag,
	             grab_red_flag,grab_green_flag,grab_blue_flag,
							 grab_position1_flag,grab_position2_flag,grab_position3_flag,arm_up_flag,arm_down_flag,arm_up_reset_flag,upload_flag,
							grab_to_own1_flag,start_flag,set_material_flag,turn_self_zero_flag;
uint8_t screen_code[2];
uint16_t qrcode_data[2];
uint8_t qrcode_count;
void cal_qrcode(void);
void screen_task(void)
{
		static uint8_t RxState = 0;
		static uint8_t pRxPacket = 0;			
		if (RxState == 0)
		{
			if (screen_rxdata == 0xAA)
			{
				RxState = 1;
				pRxPacket = 0;
			}
		}
		
		else if (RxState == 1)
		{
			screen_code[pRxPacket] = screen_rxdata;
			pRxPacket ++;
			if (pRxPacket >= 2)
			{
				RxState = 2;
			}
		}
		else if (RxState == 2)
		{
			if (screen_rxdata ==0xBB)
			{
				RxState = 0;
			}
			if(screen_code[0]==0xA1)
			{
				ahand_flag=1;
			}
			else if(screen_code[0]==0xA4)
			{
					behind_flag=1;
			}
			else if(screen_code[0]==0xA2)
			{
				left_flag=1;
			}
			else if(screen_code[0]==0xA3)
			{
				right_flag=1;
			}
			else if(screen_code[0]==0xC2)
			{
				turn_left_flag=1;
			}
			else if(screen_code[0]==0xc3)
			{
				turn_right_flag=1;
			}
			else if(screen_code[0]==0xA5)
			{
				stop_flag=1;
			}
			else if(screen_code[0]==0x04)
			{
				run_circle_flag=1;
			}
			else if(screen_code[0]==0x05)
			{
				upload_flag=1;
			}
			else if(screen_code[0]==0x03)
			{
				position_flag=1;
			}
			else if(screen_code[0]==0x07)
			{
				grab_red_flag=1;
			}
			else if(screen_code[0]==0x08)
			{
				grab_green_flag=1;
			}
			else if(screen_code[0]==0x09)
			{
				grab_blue_flag=1;
			}
			else if(screen_code[0]==0xD1)
			{
				grab_position1_flag=1;
			}
			else if (screen_code[0]==0xD2)
			{
				grab_position2_flag=1;
			}
			else if(screen_code[0]==0xD3)
			{
				grab_position3_flag=1;
			}
			else if(screen_code[0]==0xE1)
			{
				arm_up_flag=1;
			}
			else if(screen_code[0]==0xE2)
			{
				arm_down_flag=1;
			}
			else if(screen_code[0]==0xE3)
			{
				arm_up_reset_flag=1;
			}
			else if(screen_code[0]==0xD4)
			{
				grab_to_own1_flag=1;
			}
			else if(screen_code[0]==0x01)
			{
				start_flag=1;
			}
			else if(screen_code[0]==0x06)
			{
				set_material_flag=1;
			}
			else if(screen_code[0]==0xc1)
			{
				turn_self_zero_flag=1;
			}

			else if(screen_code[0]==0xF1&&screen_code[1]==0x01)
			{
				qrcode_data[0]=123;
			}
			else if(screen_code[0]==0xF1&&screen_code[1]==0x02)
			{
				qrcode_data[0]=132;
			}
			else if(screen_code[0]==0xF1&&screen_code[1]==0x03)
			{
				qrcode_data[0]==213;
			}
			else if(screen_code[0]==0xf1&&screen_code[1]==0x04)
			{
				qrcode_data[0]==231;
			}
			else if(screen_code[0]==0xf1&&screen_code[1]==0x05)
			{
				qrcode_data[0]=312;
			}
			else if(screen_code[0]==0xf1&&screen_code[1]==0x06)
			{
				qrcode_data[0]=321;
			}
			
			else if(screen_code[0]==0xf2&&screen_code[1]==0x01)
			{
				qrcode_data[1]=123;
			}
			else if(screen_code[0]==0xf2&&screen_code[1]==0x02)
			{
				qrcode_data[1]=132;
			}
			else if(screen_code[0]==0xf2&&screen_code[1]==0x03)
			{
				qrcode_data[1]=213;
			}
			else if(screen_code[0]==0xf2&&screen_code[1]==0x04)
			{
				qrcode_data[1]=231;
			}
			else if(screen_code[0]==0xf2&&screen_code[1]==0x05)
			{
				qrcode_data[1]=312;
			}
			else if(screen_code[0]==0xf2&&screen_code[1]==0x06)
			{
				qrcode_data[1]=321;
			}
		}
	HAL_UART_Receive_IT(&huart2,&screen_rxdata,1);
}

void cal_qrcode(void)
{
	uint16_t qrcode1,qrcode2;
	qrcode1=(qrcode_buf[0]-48)*100+(qrcode_buf[1]-48)*10+(qrcode_buf[2]-48);
	qrcode2=(qrcode_buf[4]-48)*100+(qrcode_buf[5]-48)*10+(qrcode_buf[6]-48);
	qrcode_data[0]=qrcode1;
	qrcode_data[1]=qrcode2;
}
void show_qrocde(int16_t num1,int16_t num2)
{
	Serial_Printf(&huart2,"t0.txt=\"+\"\xff\xff\xff");
	delay_ms(5);
	Serial_Printf(&huart2,"n0.val=%d\xff\xff\xff",num1);
	delay_ms(5);
	Serial_Printf(&huart2,"n1.val=%d\xff\xff\xff",num2);
	delay_ms(5);
	Serial_Printf(&huart2,"t0.txt=\"+\"\xff\xff\xff");
	delay_ms(5);
}