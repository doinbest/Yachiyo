#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { unsigned int instance; } I2C_HandleTypeDef;
typedef struct { unsigned int instance; } SPI_HandleTypeDef;
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
typedef struct { uint32_t counter; } DMA_HandleTypeDef;
typedef struct { unsigned int instance; void *Instance; void *hdmarx; uint32_t ErrorCode; } UART_HandleTypeDef;
#define __HAL_DMA_GET_COUNTER(p) (((DMA_HandleTypeDef *)(p))->counter)
#define HAL_UART_ERROR_ORE 8U
#define HAL_UART_ERROR_FE 4U
#define HAL_UART_ERROR_NE 2U
#define HAL_UART_ERROR_DMA 16U
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
#define UART5 ((void *)5)
#define DMA_IT_HT 1U
#define __HAL_DMA_DISABLE_IT(a,b) ((void)(a),(void)(b))
#define __IO volatile
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *,uint8_t *,uint16_t);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *,uint8_t *,uint16_t);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *);
#define USART1 ((void *)1)
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *);
typedef struct { unsigned int instance; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
extern GPIO_TypeDef test_gpioe;
extern GPIO_TypeDef test_gpioa;
#define GPIOA (&test_gpioa)
#define GPIOE (&test_gpioe)
#define GPIO_PIN_2 (1U << 2)
#define GPIO_PIN_3 (1U << 3)
#define GPIO_PIN_4 (1U << 4)
#define GPIO_PIN_5 (1U << 5)
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint16_t);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
#define I2C_MEMADD_SIZE_8BIT 1U
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *, uint16_t, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t);
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t value) { (void)value; }
uint32_t HAL_GetTick(void);
#endif
