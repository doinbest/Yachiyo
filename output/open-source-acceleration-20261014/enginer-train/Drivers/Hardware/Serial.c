#include "serial.h"
#include "main.h"
#include "stdio.h"
#include <stdarg.h>
#include "usart.h"
#include "hwt101.h"
#include "user_uart.h"
#include "serial_task.h"
void Serial_Init_IT(UART_HandleTypeDef *huart,uint8_t * Data_Buf)
{
	HAL_UART_Receive_IT(huart,Data_Buf,1);

}
void Serial_Init_DMA(UART_HandleTypeDef *huart,uint8_t * Data_Buf)
{
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	HAL_UART_Receive_DMA(huart,Data_Buf,255);
}
int fputc(int ch, FILE *f)  
{
	HAL_UART_Transmit(&huart3,(uint8_t *)&ch,1,0xFFFF);
	return ch;
}

void Serial_Printf(UART_HandleTypeDef *huart,const char *format, ...)
{
	char buf[512]; 
  va_list args;
  va_start(args, format);
  uint16_t len = vsnprintf((char *)buf, sizeof(buf), (char *)format, args);
  va_end(args);
  HAL_UART_Transmit_DMA(huart,(uint8_t *)buf,len); 
}
uint8_t qrcode_rx_data;
extern uint8_t screen_code[2];
extern uint16_t qrcode_data[2];
extern uint8_t qrcode_count;
extern uint16_t qrcode_data[2];
extern uint8_t qrcode_buf[32];
extern uint8_t serial2_choose;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART2)
	{
		screen_task();
	}
	if(huart->Instance==USART3)
	{
		view_task();
	}
	if(huart->Instance==UART4)
	{
		HWT101_Receive_Task();
	}
	if(huart->Instance==UART5)
	{
		SERVO_recvive_Task(huart);
	}
	if(huart->Instance==USART6)
	{
//		if(serial2_choose==1)
//		{
			if(qrcode_data[0]==0&&qrcode_data[1]==0)
			{
				qrcode_buf[qrcode_count]=qrcode_rx_data;
				qrcode_count++;
			}
			if(qrcode_rx_data==0x0d)
			{
				qrcode_count=0;
				cal_qrcode();
			}
//		}
			HAL_UART_Receive_IT(&huart6, (uint8_t *)&qrcode_rx_data, 1);
	}

}

extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance==UART4)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart4,HWT101_rxbuf,255);
		__HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);  
		ParseAndPrintData(HWT101_rxbuf,Size);
		memset(HWT101_rxbuf,0,255);		
	}	
	if(huart->Instance==USART3)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3,view_rxbuf2,12);
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT); 
		if(Size==4 && view_rxbuf2[0]==0xff)		
		{
					blob_data[0]=cal_data(view_rxbuf2[0],view_rxbuf2[1]);
					blob_data[1]=cal_data(view_rxbuf2[2],view_rxbuf2[3]);
					blob_float_data[0]=(float)blob_data[0];
					blob_float_data[1]=(float)blob_data[1];
					blob_float_data[0]/=100;
					blob_float_data[1]/=100;
		}
//		else if(Size==12 && view_rxbuf2[0]==0xaa)
//		{
//				color_all_pos[0][0]=cal_data(view_rxbuf2[0],view_rxbuf2[1]);
//				color_all_pos[0][1]=cal_data(view_rxbuf2[2],view_rxbuf2[3]);
//				color_all_pos[1][0]=cal_data(view_rxbuf2[4],view_rxbuf2[5]);
//				color_all_pos[1][1]=cal_data(view_rxbuf2[6],view_rxbuf2[7]);
//				color_all_pos[2][0]=cal_data(view_rxbuf2[8],view_rxbuf2[9]);
//				color_all_pos[2][1]=cal_data(view_rxbuf2[10],view_rxbuf2[11]);
//		}
		memset(view_rxbuf2,0,255);		
	}
}


