#ifndef RADAR_TEST_MAIN_H
#define RADAR_TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t counter; } DMA_HandleTypeDef;
typedef struct { void *Instance;DMA_HandleTypeDef *hdmarx;uint32_t ErrorCode; } UART_HandleTypeDef;
typedef enum { HAL_UART_RXEVENT_TC,HAL_UART_RXEVENT_HT,HAL_UART_RXEVENT_IDLE } HAL_UART_RxEventTypeTypeDef;
#define USART2 ((void *)2)
#define HAL_UART_ERROR_DMA 16U
#define __HAL_DMA_GET_COUNTER(p) ((p)->counter)
static inline uint32_t __get_PRIMASK(void){return 0;}
static inline void __disable_irq(void){}
static inline void __set_PRIMASK(uint32_t x){(void)x;}
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*,uint8_t*,uint16_t);
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef*);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef*);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef*);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef*,uint8_t*,uint16_t);
#endif
