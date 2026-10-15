#include "stm32f4xx.h"                  // Device header
#include "main.h"
#include "usart.h"
#include "serial.h"

uint8_t unlock_register[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
uint8_t reset_z_axis[] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
uint8_t set_output_200Hz[] = {0xFF, 0xAA, 0x03, 0x0B, 0x00};
uint8_t set_baudrate_115200[] = {0xFF, 0xAA, 0x04, 0x06, 0x00};
uint8_t save_settings[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
uint8_t restart_device[] = {0xFF, 0xAA, 0x00, 0xFF, 0x00};

//uint8_t HWT101_rx_temp[255];

uint8_t HWT101_rxbuf[11];
uint8_t HWT101_rxdata;
//陀螺仪校准函数
void HW101_adjust(void)
{
	uint8_t array1[]={0xff,0xaa,0x69,0x88,0xb5};
	uint8_t array2[]={0xff,0xaa,0x48,0x01,0x00};
	uint8_t array3[]={0xff,0xaa,0x69,0x88,0xb5};
	uint8_t array4[]={0xff,0xaa,0x00,0x00,0x00};
	HAL_UART_Transmit_DMA(&huart4,array1,sizeof(array1));
	delay_ms(1000);
	HAL_UART_Transmit_DMA(&huart4,array2,sizeof(array2));
	delay_ms(2000);
	delay_ms(2000);
	delay_ms(2000);
	delay_ms(2000);
	delay_ms(2000);
	delay_ms(2000);
	delay_ms(2000);
	delay_ms(2000);
	delay_ms(2000);
	delay_ms(2000);
	delay_ms(2000);
	delay_ms(2000);
	delay_ms(2000);
	delay_ms(2000);
	delay_ms(2000);
	HAL_UART_Transmit_DMA(&huart4,array3,sizeof(array3));
	delay_ms(1000);
	HAL_UART_Transmit_DMA(&huart4,array4,sizeof(array4));
}

uint8_t CalculateChecksum(uint8_t *data, uint16_t length, uint8_t type) 
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}

void set_angle_zero(void)
{
	HAL_UART_Transmit(&huart4,reset_z_axis,sizeof(reset_z_axis),0xff);
}
extern DMA_HandleTypeDef hdma_uart4_rx;
void HWT101_Init(void)
{	
	Serial_Init_IT(&huart4,HWT101_rxbuf);
	HAL_UART_Transmit(&huart4,unlock_register, sizeof(unlock_register),0xff);
	delay_ms(1);
  HAL_UART_Transmit(&huart4,reset_z_axis, sizeof(reset_z_axis),0xff);
	delay_ms(1);
  HAL_UART_Transmit(&huart4,set_output_200Hz, sizeof(set_output_200Hz),0xff);
	delay_ms(1);
  HAL_UART_Transmit(&huart4,set_baudrate_115200, sizeof(set_baudrate_115200),0xff);
	delay_ms(1);
  HAL_UART_Transmit(&huart4,save_settings, sizeof(save_settings),0xff);
	delay_ms(1);
  HAL_UART_Transmit(&huart4,restart_device, sizeof(restart_device),0xff);
	delay_ms(1);
	set_angle_zero();
	delay_ms(1);
}

float global_angle;
uint8_t new_data_received;
float angular_velocity_y;
float angular_velocity_z;
uint8_t received_data_packet[11];
float global_angle_all;
void ParseAndPrintData(uint8_t *data, uint16_t length) 
{
				static int flag=0;
				static float last_angle;
    if (length == 11) {
        uint8_t checksum = CalculateChecksum(data, length - 1, data[1]);
        if (checksum != data[length - 1]) {
            return;
        }
 
        if (data[0] == 0x55 && data[1] == 0x53)
				{
            uint8_t yaw_l = data[6];
            uint8_t yaw_h = data[7];
            int16_t yaw = (int16_t)((yaw_h << 8) | yaw_l);
 

            float angle = ((float)yaw / 32768.0) * 180.0;
            global_angle = angle;
            new_data_received = 1;
					
					if(global_angle-last_angle<-180.0)
						flag++;
					else if(global_angle-last_angle>180.0)
						flag--;
					last_angle=global_angle;
					
					global_angle_all=global_angle+flag*360.0;
					
        } 
				else if (data[0] == 0x55 && data[1] == 0x52) {
            uint8_t wy_l = data[4];
            uint8_t wy_h = data[5];
            int16_t wy = (int16_t)((wy_h << 8) | wy_l);
            angular_velocity_y = ((float)wy / 32768.0) * 2000.0;
 
            uint8_t wz_l = data[6];
            uint8_t wz_h = data[7];
            int16_t wz = (int16_t)((wz_h << 8) | wz_l);
            angular_velocity_z = ((float)wz / 32768.0) * 2000.0;
 
            new_data_received = 1;
        }				
    }
}
uint8_t RX_Data[13];
float angledata[3];
float data_a[3];
float data_g[3],start_angle,global_angle;
uint8_t angle_init_flag;

void HWT101_Receive_Task(void) 
{
    static uint8_t rx_index = 0;
    static uint8_t state = 0;
 
    if (state == 0)				
		{ 
        if (HWT101_rxdata == 0x55)
				{
            HWT101_rxbuf[0] = HWT101_rxdata;
            rx_index = 1;
            state = 1;
        }
    } 
		else if (state == 1) 
		{ 
            HWT101_rxbuf[rx_index++] = HWT101_rxdata;
            if (rx_index == 2 && HWT101_rxbuf[1] != 0x53 && HWT101_rxbuf[1] != 0x52) 
							{
                if (HWT101_rxbuf[1] == 0x55) 
								{
                    rx_index = 1;
                }
								else 
								{
                    state = 0;
                    rx_index = 0;
                }
							} 
							else if (rx_index == 11) 
							{ 
                ParseAndPrintData(HWT101_rxbuf, 11);
                rx_index = 0;
                state = 0; 
							}
     }



//		if(state==0)
//		{
//			if(HWT101_rxdata==0x55)
//			{
//				state=1;
//			}
//		}
//		else if(state==1)
//		{
//			if(HWT101_rxdata==0x55)
//			{
//				state=2;
//			}
//			else 
//				state=0;
//		}
//		else if(state==2)
//		{
//			if(HWT101_rxdata==0x01)
//			{
//				state=3;
//			}
//			else if(HWT101_rxdata==0x03)
//			{
//				state=4;
//			}
//			else 
//				state=0;
//		}
//		else if(state==3)
//		{
//			RX_Data[rx_index]=HWT101_rxdata;
//			rx_index++;
//			if(rx_index>=7)
//			{
//				angledata[0]= (float)((int16_t)(RX_Data[2] << 8) | RX_Data[1]) / 32768 * 180;
//				angledata[1]= (float)((int16_t)(RX_Data[4] << 8) | RX_Data[3]) / 32768 * 180;
//				angledata[2]= (float)((int16_t)(RX_Data[6] << 8) | RX_Data[5]) / 32768 * 180;
//				if(angle_init_flag==0)
//				{
//					start_angle=angledata[2];
//					angle_init_flag=1;
//				}
//				rx_index=0;
//				state=0;
//				global_angle=angledata[2]-start_angle;
//				static int flag=0;
//				static float last_angle;
//				if(global_angle-last_angle<-180.0)
//					flag++;
//				else if(global_angle-last_angle>180.0)
//					flag--;
//				last_angle=global_angle;
//				
//				global_angle_all=global_angle+flag*360.0;
//			}
//		}
//		
//		else if(state==4)
//		{
//			RX_Data[rx_index]=HWT101_rxdata;
//			rx_index++;
//			if(rx_index>=13)
//			{
//				data_a[0]=(float)((int16_t)(RX_Data[2] << 8)|RX_Data[1])/32768 * 4*9.8;
//				data_a[1]=(float)((int16_t)(RX_Data[4] << 8)|RX_Data[3])/32768 * 4*9.8;
//				data_a[2]=(float)((int16_t)(RX_Data[6] << 8)|RX_Data[5])/32768 * 4*9.8;
//				data_g[0]= (float)((int16_t)(RX_Data[8] << 8) | RX_Data[7])/ 32768 * 2000;
//				data_g[1]= (float)((int16_t)(RX_Data[10] << 8) | RX_Data[9])/ 32768 *2000;
//				data_g[2]= (float)((int16_t)(RX_Data[12] << 8) | RX_Data[11])/ 32768 * 2000;
//				rx_index=0;
//				state=0;
//		}
//	}


		HAL_UART_Receive_IT(&huart4,&HWT101_rxdata,1);
}
