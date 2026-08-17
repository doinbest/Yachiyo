#ifndef __MAIXCAM_H
#define __MAIXCAM_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "headfile.h"

#define MAIXCAM_RX_SIZE 4
#define MAIXCAM_HEAD 0xFF
#define MAIXCAM_TAIL 0xEE

#define Turn_Left 0xA1
#define Turn_Right 0xA2
#define NOT_FOUND 0xA3
#define NO_TARGET 0xA4

#define MAIX_RET_BUSY 0      // 还在等待
#define MAIX_RET_TARGET_OK 1 // 识别目标数字成功
#define MAIX_RET_LEFT 2      // 目标在左边
#define MAIX_RET_RIGHT 3     // 目标在右边
#define MAIX_RET_NOT_FOUND 4 // 没找到目标
#define MAIX_RET_NO_TARGET 5 // 没有设置目标
#define MAIX_RET_ERROR 6     // 数据错误

    extern uint8_t maixcam_Serial_TxPacket[4];

    extern volatile uint8_t target_number;
    extern volatile uint8_t turn_flag;
    extern volatile uint8_t receive_flag;
    extern volatile uint8_t send_flag;
    extern volatile int abc;
extern uint8_t trace_value[8];
extern uint8_t maixcam_data_buff[4];

extern uint8_t maix_rx_byte;
extern volatile uint8_t maix_frame_data;
extern volatile uint8_t maix_state;
extern volatile uint8_t road_side;

void maixcam_Init(UART_HandleTypeDef *huart);

void maixcam_Serial_SendByte(uint8_t Byte);
void maixcam_Serial_SendArray(uint8_t *Array, uint16_t Length);
void maixcam_Serial_SendString(char *String);
void maixcam_Serial_SendNumber(uint32_t Number, uint8_t Length);
void maixcam_SendPacket(uint8_t data);

void maixcam_RxCpltCallback(UART_HandleTypeDef *huart);
void maixcam_DecToBin(uint8_t num, uint8_t bits[8]);
uint8_t MaixCAM_Proc(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIXCAM_H */