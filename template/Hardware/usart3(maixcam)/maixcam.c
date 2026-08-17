#include "maixcam.h"

static UART_HandleTypeDef *maixcam_huart = NULL;
static uint8_t maixcam_rx_byte;

uint8_t maixcam_Serial_TxPacket[4];
uint8_t maixcam_data_buff[MAIXCAM_RX_SIZE];

volatile uint8_t target_number;
volatile uint8_t turn_flag = 0;
volatile uint8_t receive_flag = 0;
volatile uint8_t send_flag = 0;
volatile int abc = 0;

uint8_t trace_value[8];

// volatile uint8_t maix_rx_byte = 0;
volatile uint8_t maix_frame_data = 0;
volatile uint8_t maix_state = 0;

volatile uint8_t road_side = 0;

/**
  * 函    数：MaixCam 初始化
  * 说    明：启动 UART 中断接收
  */
void maixcam_Init(UART_HandleTypeDef *huart)
{
    maixcam_huart = huart;

    target_number = 0;
    turn_flag = 0;
    receive_flag = 0;
    send_flag = 0;

    HAL_UART_Receive_IT(maixcam_huart, &maixcam_rx_byte, 1);
}


/**
  * 函    数：串口发送一个字节
  */
void maixcam_Serial_SendByte(uint8_t Byte)
{
    if (maixcam_huart == NULL)
    {
        return;
    }

    HAL_UART_Transmit(maixcam_huart, &Byte, 1, 100);
}


/**
  * 函    数：串口发送数组
  */
void maixcam_Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;

    for (i = 0; i < Length; i++)
    {
        maixcam_Serial_SendByte(Array[i]);
    }
}


/**
  * 函    数：串口发送字符串
  */
void maixcam_Serial_SendString(char *String)
{
    uint16_t i;

    for (i = 0; String[i] != '\0'; i++)
    {
        maixcam_Serial_SendByte(String[i]);
    }
}


/**
  * 函    数：次方函数
  */
static uint32_t maixcam_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;

    while (Y--)
    {
        Result *= X;
    }

    return Result;
}


/**
  * 函    数：串口发送数字
  */
void maixcam_Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;

    for (i = 0; i < Length; i++)
    {
        maixcam_Serial_SendByte(Number / maixcam_Pow(10, Length - i - 1) % 10 + '0');
    }
}


/**
  * 函    数：串口发送数据包
  * 说    明：指定第3个数据位后发送4字节数据
  * 格    式：4字节数据
  */
void maixcam_SendPacket(uint8_t data)
{

    maixcam_Serial_TxPacket[0] = MAIXCAM_HEAD; // 包头
    maixcam_Serial_TxPacket[1] = MAIXCAM_HEAD; // 包头
    maixcam_Serial_TxPacket[2] = data;   // 修改第3个数据位
    maixcam_Serial_TxPacket[3] = MAIXCAM_TAIL; // 包尾

    maixcam_Serial_SendArray(maixcam_Serial_TxPacket, 4);
}


/**
  * 函    数：MaixCam 接收数据校验
  * 接收格式：B4 B4 target_number receive_flag turn_flag 6B
  */
static uint8_t maixcam_data_parse(uint8_t *pack)
{
    if (pack[0] != MAIXCAM_HEAD) return 0;
    if (pack[1] != MAIXCAM_HEAD) return 0;
    if (pack[MAIXCAM_RX_SIZE - 1] != MAIXCAM_TAIL) return 0;

    abc++;  // test

    return 1;
}


/**
  * 函    数：MaixCam 串口接收回调
  * 说    明：在 HAL_UART_RxCpltCallback 中调用
  */
void maixcam_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint8_t i = 0;
    static uint8_t dataFlag = 0;

    uint8_t data;

    if (maixcam_huart == NULL)
    {
        return;
    }

    if (huart->Instance != maixcam_huart->Instance)
    {
        return;
    }

    data = maixcam_rx_byte;

    if (dataFlag == 0)
    {
        if (data == MAIXCAM_HEAD)
        {
            dataFlag = 1;
            i = 0;

            maixcam_data_buff[i] = data; // 保存第一个 0xFF
            i++;
        }
    }
    else
    {
        maixcam_data_buff[i] = data; // 保存第二个 0xFF、target_number、0xEE

        if (i < MAIXCAM_RX_SIZE - 1)
        {
            i++;
        }
        else
        {
            dataFlag = 0;
            i = 0;

            if (maixcam_data_parse(maixcam_data_buff))
            {
                receive_flag = 1;
            }
        }
    }

    HAL_UART_Receive_IT(maixcam_huart, &maixcam_rx_byte, 1);
}

/**
 * 函    数：十进制转二进制数组
 * 说    明：将 0~255 的十进制数转换为 8 位二进制数组
 * 参    数：num   十进制数据
 * 参    数：bits  存放转换结果的数组，长度为 8
 * 返 回 值：无
 *
 * 例    子：
 * num = 0b10000001
 * bits[0] = 1
 * bits[1] = 0
 * ...
 * bits[7] = 1
 */
void maixcam_DecToBin(uint8_t num, uint8_t bits[8])
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        bits[i] = (num >> (7 - i)) & 0x01;
    } 
}

uint8_t MaixCAM_Proc(void)
{
    uint8_t data;

    if (maix_state == 0)
    {
        /*
         * 发车前识别目标数字
         * car_go = 0，target_number = 0 时由 main 设置 maix_state = 0
         */
        if (send_flag == 0 && target_number == 0)
        {
            maixcam_SendPacket(0x01);
            send_flag = 1;
            return MAIX_RET_BUSY;
        }

        /*
         * 没收到完整帧，不处理
         */
        if (receive_flag == 0)
        {
            return MAIX_RET_BUSY;
        }

        receive_flag = 0;
        data = maixcam_data_buff[2];

        if (data >= 0x01 && data <= 0x08)
        {
            target_number = data;

            maixcam_data_buff[2] = 0;
            send_flag = 0;

            return MAIX_RET_TARGET_OK;
        }

        maixcam_data_buff[2] = 0;
        send_flag = 0;
        return MAIX_RET_ERROR;
    }
    else if (maix_state == 1)
    {
        /*
         * 路口判断左右
         * car_go = 1 且 target_number = 3~8 后，
         * task_search_middle_then_far() 里设置 maix_state = 1
         */
        if (send_flag == 0 && road_side == 0)
        {
            maixcam_SendPacket(0x02);
            send_flag = 1;
            return MAIX_RET_BUSY;
        }

        /*
         * 没收到完整帧，不处理
         */
        if (receive_flag == 0)
        {
            return MAIX_RET_BUSY;
        }

        receive_flag = 0;
        data = maixcam_data_buff[2];

        if (data == Turn_Left)
        {
            road_side = Turn_Left;
            maixcam_data_buff[2] = 0;
            return MAIX_RET_LEFT;
        }
        else if (data == Turn_Right)
        {
            road_side = Turn_Right;
            maixcam_data_buff[2] = 0;
            return MAIX_RET_RIGHT;
        }
        else if (data == NOT_FOUND)
        {
            /*
             * 如果 MaixCAM 是单次请求单次回复模式：
             * 收到 A3 后允许下一轮继续发送 0x02。
             */
            road_side = 0;
            maixcam_data_buff[2] = 0;
            send_flag = 0;

            return MAIX_RET_NOT_FOUND;
        }
        else if (data == NO_TARGET)
        {
            road_side = NO_TARGET;
            maixcam_data_buff[2] = 0;
            send_flag = 0;

            return MAIX_RET_NO_TARGET;
        }

        maixcam_data_buff[2] = 0;
        send_flag = 0;
        return MAIX_RET_ERROR;
    }
    else if (maix_state == 3)
    {
        /*
         * 远端左偏扫描模式：发送 0x04
         * 不判断目标在画面左边还是右边。
         * 只判断画面里有没有目标数字。
         *
         * 左偏 30° 后，如果识别到目标，就认为目标在远端左侧。
         */
        if (send_flag == 0 && road_side == 0)
        {
            maixcam_SendPacket(0x04);
            send_flag = 1;
            return MAIX_RET_BUSY;
        }

        if (receive_flag == 0)
        {
            return MAIX_RET_BUSY;
        }

        receive_flag = 0;
        data = maixcam_data_buff[2];

        /*
         * 这里兼容两种 MaixCAM 返回方式：
         *
         * 1. 返回目标数字：01~08
         * 2. 返回找到标志：A1 或 A2
         *
         * 只要有识别到，就认为远端目标在左侧。
         */
        if ((data >= 0x01 && data <= 0x08) ||
            data == Turn_Left ||
            data == Turn_Right)
        {
            road_side = Turn_Left;

            maixcam_data_buff[2] = 0;
            send_flag = 0;

            return MAIX_RET_LEFT;
        }
        else if (data == NOT_FOUND)
        {
            /*
             * 没找到，继续允许下一轮发送 0x04。
             * 注意这里不要把 road_side 设成 Turn_Right。
             * 是否判定右侧，交给 task 里的超时逻辑处理。
             */
            road_side = 0;
            maixcam_data_buff[2] = 0;
            send_flag = 0;

            return MAIX_RET_NOT_FOUND;
        }
        else if (data == NO_TARGET)
        {
            road_side = NO_TARGET;
            maixcam_data_buff[2] = 0;
            send_flag = 0;

            return MAIX_RET_NO_TARGET;
        }

        maixcam_data_buff[2] = 0;
        send_flag = 0;
        return MAIX_RET_ERROR;
    }
    else
    {
        /*
         * maix_state == 2
         * 空闲状态，不发送命令，不处理任务
         */
        return MAIX_RET_BUSY;
    }
}