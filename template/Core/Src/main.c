/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *
  * @par OLED和JY61P接线
  * - OLED SCL  -> PB6（I2C1_SCL，100 kHz）
  * - OLED SDA  -> PB7（I2C1_SDA，设备7位地址默认0x3C）
  * - JY61P RX  -> PC10（UART4_TX，115200、8N1）
  * - JY61P TX  -> PC11（UART4_RX，115200、8N1）
  * - OLED、JY61P 和 STM32 必须共地，信号电平必须为3.3 V
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "chassis_key_test.h"
#include "jy61p.h"
#include "oled.h"
#include "screen_verify.h"
#include "tjc_screen.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* JY61P最新三轴角度，可在Keil Watch窗口中实时观察。 */
JY61P_Angle_t jy61p_angle = {0};

/* OLED通信状态：HAL_OK表示地址0x3C应答正常且初始化成功。 */
HAL_StatusTypeDef oled_status = HAL_ERROR;

/* 陶晶驰串口屏通信状态，可在Keil Watch窗口中实时观察。 */
TJC_Status_t tjc_status = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_UART5_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  /*
   * 完全按照张大头官方 UART 位置模式例程启动 UART5 DMA 接收。
   * PC12 为 TX，PD2 为 RX；rxFrameFlag/rxCmd/rxCount 与官方例程一致。
   */
  __HAL_UART_CLEAR_IDLEFLAG(&huart5);
  __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
  if (HAL_UART_Receive_DMA(&huart5, (uint8_t *)rxCmd, CMD_LEN) != HAL_OK)
  {
    Error_Handler();
  }

  /* JY61P使用UART4：PC10发送、PC11接收，115200、8N1。 */
  if (!JY61P_Init(&huart4))
  {
    Error_Handler();
  }

  /* OLED使用I2C1：PB6为SCL、PB7为SDA，默认7位地址0x3C；暂时只初始化并清屏。 */
  oled_status = OLED_Init(&hi2c1);

  /* 陶晶驰串口屏使用USART3：PB10发送、PB11接收、115200、8N1。 */
  if (TJC_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  Screen_Check_Init();

  /* 等待四台闭环驱动器完成上电初始化。 */
  HAL_Delay(500);

  /* 初始化PE2~PE5按键控制的麦轮底盘测试程序。 */
  Chassis_Key_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* UART4中断完成字节接收；主循环只读取最新姿态角，暂不显示到OLED。 */
    (void)JY61P_Angle_Get(&jy61p_angle);

    /* 串口屏DMA发送、接收帧解析和最小页面验证。 */
    Screen_Check_Process();
    (void)TJC_Status_Get(&tjc_status);

    Chassis_Key_Process(&jy61p_angle);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief    分发HAL串口接收完成事件
  * @param    huart ：触发回调的串口句柄
  * @retval   无
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  JY61P_Rx_Callback(huart);
  TJC_Rx_Callback(huart);
}

/**
  * @brief    分发HAL串口DMA发送完成事件
  * @param    huart ：触发回调的串口句柄
  * @retval   无
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  TJC_Tx_Callback(huart);
}

/**
  * @brief    分发HAL串口通信错误事件
  * @param    huart ：触发回调的串口句柄
  * @retval   无
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  JY61P_Error_Callback(huart);
  TJC_Error_Callback(huart);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
