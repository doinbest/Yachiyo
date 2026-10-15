#include "my_uart.h"

float pos_x=0;//地图绝对坐标X轴
float pos_y=0;//地图绝对坐标Y轴
float zangle=0;
float xangle=0;
float yangle=0;
float w_z=0;
char data[] = "ACT0";   //正交编码轮初始化
uint8_t data2=0x00;
uint8_t data3=0x01;

/**
 * @brief 数据解析函数  
 * @param  rec 串口接收到的字节数据
 */
void Data_Analyse(uint8_t rec)
{
	static uint8_t ch;
	static union
	{
		
		
		uint8_t date[24];
		float ActVal[6];
	}posture;
	static uint8_t count=0;
	static uint8_t i=0;

	ch=rec;
	switch(count)
	{
		case 0:
			if(ch==0x0d)
				count++;
			else
				count=0;
			break;
		case 1:
			if(ch==0x0a)
			{
				i=0;
				count++;
			}
			else if(ch==0x0d);
			else
				count=0;
			break;
		case 2:
			posture.date[i]=ch;
			i++;
			if(i>=24)
			{
				i=0;
				count++;
			}
			break;
		case 3:
			if(ch==0x0a)
				count++;
			else
				count=0;
			break;
		case 4:
			if(ch==0x0d)
			{
				zangle=posture.ActVal[0];
				xangle=posture.ActVal[1];
				yangle=posture.ActVal[2];
				pos_x=posture.ActVal[3];
				pos_y=posture.ActVal[4];
				w_z=posture.ActVal[5];
			}
			count=0;
			break;
		default:
			count=0;
		break;
	}
}

void Data_stop(double*pos_X,double*pos_Y)//坐标清零函数
{
  *pos_X=0;
  *pos_Y=0;
  HAL_UART_Transmit_IT(&huart3, (uint8_t*)data, sizeof(data) - 1);//正交编码器置零
  HAL_Delay(800); // 等待数据发送完成
}


/**
 * @brief 陀螺仪Z轴角度初始化函数  
 */
void Gyro_Init(void)
{
  HAL_UART_Transmit_IT(&huart5, &data2, 1);
  HAL_Delay(200); // 等待数据发送完成
  HAL_UART_Transmit_IT(&huart5, &data2, 1);
}

/**
 * @brief 二维码读取函数 
 */
void rwm_dq(void)
{
  HAL_UART_Transmit_IT(&huart5, &data3, 1);
  HAL_Delay(200); // 等待数据发送完成
  HAL_UART_Transmit_IT(&huart5, &data3, 1);
}
