#include "smp.h"
#include "main.h"
#include "usart.h"
#include <string.h>



uint8_t data_2m[9] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD}; 
uint8_t data_2m_2[3];
double smp_num[3]={0};//【0，1】：偏差值(X轴Y轴值)，【2】:颜色ID（0：红色，1：绿色，2：蓝色）
uint8_t smp_wk[3]={254,254,254};//0:红色，1：绿色，2：蓝色（这个是数组的下标），里面存放的是物块的位置ID（0：左上，1：右上，2：正下）---------这个每抓一次物块，就会增加1（余3后的值）
bool smp_bool=false; //标志正常接收到数据--1：接受到，0：未接受到
/**
 * @brief    树莓派数据解析函数（逐字节接收）
 * @param    byte    接收到的单个字节
 * @Return:  返回解析状态（1：成功，0：正在接收，-1：错误）
 */
int smp_por(uint8_t byte)
{
    static uint8_t ch;
    static uint8_t frame_fh[2];   // 定位帧符号数据（正负）
    static uint8_t frame_data[6]; // 定位帧数据
    static uint8_t count = 0;

    ch = byte;

    switch(count)
    {
        // --------- 统一入口，判断帧头 ---------
        case 0:
            if(ch == 'F') {
                count = 1; // 进入定位帧状态机
            } else if(ch == 'E') {
                count = 100; // 进入物块顺序帧状态机
            } else if(ch == 'G') {
                count=200;//进入颜色识别帧状态机
            }
            else {
                count = 0;
            }
            break;

        // --------- 定位帧状态机 ---------
        case 1:
            frame_fh[0] = ch; //确定X的符号
            count++;
            break;
        case 2: case 3: case 4: 
            frame_data[count-2] = ch;
            count++;
            break;
        case 5:
            frame_fh[1] = ch;//确定Y的符号
            count++;
            break; 
        case 6: case 7: case 8: 
            frame_data[count-3] = ch;
            count++;
            break;
        case 9:
             {   // 解析X，Y值（树莓派修正）
                char x_str[4], y_str[4];
                for(int i=0;i<3;i++) x_str[i]=frame_data[i];
                x_str[3]='\0';
                for(int i=0;i<3;i++) y_str[i]=frame_data[3+i];
                y_str[3]='\0';
                int x_int = atoi(x_str);
                int y_int = atoi(y_str);
                if( frame_fh[0] == '-') x_int = -x_int;
                if( frame_fh[1] == '-') y_int = -y_int;
                smp_num[0] = x_int;
                smp_num[1] = y_int;
                count = 0;

                smp_bool=true;//标志正常接收到数据

                break;
             }
        // --------- 二维码帧状态机 ---------
        case 100:
        {
            smp_wk[0] = ch-'0'; // 物块位置ID（0：左上，1：右上，2：正下）
            count++;
            break;
        }
        case 101:
        {
            smp_wk[1] = ch-'0';
            count++;
            break;
        }
        case 102:
        {
        	smp_wk[2] = ch-'0';
            count=0;// 重置计数器

            smp_bool=true;//标志正常接收到数据

            break;
        }
        // --------- 颜色识别帧状态机 ---------
        case 200:
        {
            smp_num[2] = ch-'0';//颜色ID（0：红色，1：绿色，2：蓝色）
            count=0;// 重置计数器

            smp_bool=true;//标志正常接收到数据

            break;
        }
        default:
            count = 0;
            break;
    }
    return 0; // 正在接收
}

/**
 * @brief    发送函数（树莓派）
 * @param    0x00:开启识别物块功能（摄像头开始识别物块并返回信息）
 * @param    0x01:开启定位功能（摄像头开始定位并返回信息）
 * @param    0x02:关闭树莓派功能（摄像头停止识别物块和定位）
 * @param    0x03:识别颜色功能---红色
 * @param    0x04:识别颜色功能---绿色
 * @param    0x05:识别颜色功能---蓝色
 */

void smp_fs(int n)
{  
	data_2m_2[0]='E';
	data_2m_2[1]=n+ '0';
	data_2m_2[2]='F'; 
	HAL_UART_Transmit_IT(&huart2,data_2m_2, sizeof(data_2m_2));
    HAL_Delay(200); // 等待数据发送完成
    HAL_UART_Transmit_IT(&huart2,data_2m_2, sizeof(data_2m_2));
}

/**
 * @brief    二维码读取函数
 */
void smp_2m_dq(void)
{  
	HAL_UART_Transmit_IT(&huart5,data_2m,sizeof(data_2m));
}
