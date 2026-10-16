/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern _Bool uart_tx_bjdj;
extern _Bool uart_rx_bjdj;
extern uint8_t uart_rx_bjdj2;
extern uint8_t RX_pjdj[4];
extern uint8_t hd_num[2]; //鐏板害浼犳劅鍣ㄦ暟鎹?
extern float angle_value; //瑙掑害鍊?
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void UI_OLED(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USART2_TX_ly_Pin GPIO_PIN_2
#define USART2_TX_ly_GPIO_Port GPIOA
#define USART2_RX_ly_Pin GPIO_PIN_3
#define USART2_RX_ly_GPIO_Port GPIOA
#define Max7219_pinDIN_Pin GPIO_PIN_7
#define Max7219_pinDIN_GPIO_Port GPIOE
#define Max7219_pinCS_Pin GPIO_PIN_8
#define Max7219_pinCS_GPIO_Port GPIOE
#define Max7219_pinCLK_Pin GPIO_PIN_9
#define Max7219_pinCLK_GPIO_Port GPIOE
#define USART3_TX_zjbml_Pin GPIO_PIN_8
#define USART3_TX_zjbml_GPIO_Port GPIOD
#define USART3_RX_zjbml_Pin GPIO_PIN_9
#define USART3_RX_zjbml_GPIO_Port GPIOD
#define USART1_TX_bjdj_Pin GPIO_PIN_9
#define USART1_TX_bjdj_GPIO_Port GPIOA
#define USART1_RX_bjdj_Pin GPIO_PIN_10
#define USART1_RX_bjdj_GPIO_Port GPIOA
#define UART4_TX_zxdj_Pin GPIO_PIN_10
#define UART4_TX_zxdj_GPIO_Port GPIOC
#define UART4_RX_zxdj_Pin GPIO_PIN_11
#define UART4_RX_zxdj_GPIO_Port GPIOC
#define UART5_TX_smp_Pin GPIO_PIN_12
#define UART5_TX_smp_GPIO_Port GPIOC
#define UART5_RX_smp_Pin GPIO_PIN_2
#define UART5_RX_smp_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
