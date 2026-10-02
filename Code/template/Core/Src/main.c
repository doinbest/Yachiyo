/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *
  * @par 二维码模块、Orange Pi和OLED接线
  * - OLED SCL  -> PB6（I2C1_SCL，100 kHz）
  * - OLED SDA  -> PB7（I2C1_SDA，设备7位地址默认0x3C）
  * - STM32 PC10（UART4_TX）连接二维码模块RX
  * - STM32 PC11（UART4_RX）连接二维码模块TX
  * - STM32 PA11/PA12：原生USB CDC连接香橙派
  * - HWT101 SCL/SDA：PB10/PB11，I2C2；串口屏：PD8/PD9
  * - OLED、串口模块和STM32必须共地，串口信号电平必须为3.3 V
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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "Camera.h"
#include "QR.h"
#include "ArmVision.h"
#include "MaterialVision.h"
#include "GrabTask.h"
#include "GrabRoute.h"
#include "Steer.h"
#include "arm_console.h"
#include "contest_screen.h"
#include "hwt101_i2c.h"
#include "hwt101_calibration.h"
#include "key.h"
#include "mechanical_arm.h"
#include "mecanum_chassis.h"
#include "oled_ui.h"
#include "tjc_screen.h"
#include "usbd_cdc_if.h"
#include "w25q128.h"
#include "motor_bus.h"
#include "console_tx.h"
#include "console_rx.h"
#include "chassis_motion.h"
#include "chassis_localization.h"
#include "chassis_route.h"
#include "chassis_telemetry.h"

#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HWT101_UPLOAD_INTERVAL_MS 200U
#define CONSOLE_UPLOAD_TIMEOUT_MS 100U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static uint8_t QR_RxByte;

static char QR_TaskCode[QR_TASK_CODE_BUFFER_SIZE];
static uint8_t Camera_ColorTaskReady;
static uint8_t Camera_ColorDigitIndex;
static HWT101_Angle_t HWT101_CurrentAngle;
static uint8_t HWT101_AngleValid;
static uint32_t HWT101_UploadLastTick;
static uint32_t HWT101_ProgressLastTick;
static uint32_t HWT101_UploadRunId, HWT101_UploadFinalRunId;
static HWT101_CalState_t HWT101_UploadFinalState;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Board bring-up diagnostic only; not part of normal startup. */
#ifndef W25Q128_BOOT_DIAGNOSTIC
#define W25Q128_BOOT_DIAGNOSTIC 0
#endif
#if W25Q128_BOOT_DIAGNOSTIC
/** 启动时先读三次 JEDEC ID，再读三次设备 ID，失败也继续启动。 */
static void W25Q128_StartupCheck(void)
{
  uint8_t Id[3] = {0};
  uint8_t Index;
  uint8_t Command;
  uint8_t Try;
  uint8_t Invalid;
  char Text[96];
  const char *StatusText;
  const char *Prefix;
  int32_t Length;
  HAL_StatusTypeDef Status;

  for (Index = 0U; Index < 6U; Index++)
  {
    /* 首次读取保持原来的启动条件，仅在相邻交易之间等待。 */
    if (Index > 0U)
    {
      HAL_Delay(10U);
    }
    Try = (uint8_t)(Index % 3U + 1U);
    Prefix = (Index == 0U) ? "\r\n" : "";
    if (Index < 3U)
    {
      Command = 0x9FU;
      Status = W25Q128_ReadJedecId(&hspi1, Id);
    }
    else
    {
      Command = 0x90U;
      Status = W25Q128_ReadDeviceId(&hspi1, Id);
    }

    if (Status != HAL_OK)
    {
      switch (Status)
      {
        case HAL_ERROR: StatusText = "ERROR"; break;
        case HAL_BUSY: StatusText = "BUSY"; break;
        case HAL_TIMEOUT: StatusText = "TIMEOUT"; break;
        default: StatusText = "UNKNOWN"; break;
      }
      /* 失败时不打印保留在 Id 中的上次结果。 */
      Length = snprintf(Text, sizeof(Text), "%s[FLASH] cmd=%02X try=%u HAL=%s\r\n",
                        Prefix, (unsigned)Command, (unsigned)Try, StatusText);
    }
    else
    {
      Invalid = ((Id[0] == 0U) && (Id[1] == 0U) &&
                 ((Command == 0x90U) || (Id[2] == 0U))) ||
                ((Id[0] == 0xFFU) && (Id[1] == 0xFFU) &&
                 ((Command == 0x90U) || (Id[2] == 0xFFU)));
      if (Command == 0x9FU)
      {
        Length = snprintf(Text, sizeof(Text),
                          "%s[FLASH] cmd=9F try=%u HAL=OK ID=%02X %02X %02X%s\r\n",
                          Prefix, (unsigned)Try, (unsigned)Id[0], (unsigned)Id[1],
                          (unsigned)Id[2], Invalid ? " INVALID" : "");
      }
      else
      {
        Length = snprintf(Text, sizeof(Text),
                          "%s[FLASH] cmd=90 try=%u HAL=OK ID=%02X %02X%s\r\n",
                          Prefix, (unsigned)Try, (unsigned)Id[0], (unsigned)Id[1],
                          Invalid ? " INVALID" : "");
      }
    }

    if ((Length > 0) && (Length < (int32_t)sizeof(Text)))
    {
      (void)ConsoleTx_Write((uint8_t *)Text, (uint16_t)Length);
    }
  }

  (void)ConsoleTx_Write((uint8_t *)"arm> ", 5U);
}
#endif

/**
  * 函    数：读取任务码第二组中当前选择的数字字符
  * 参    数：无
  * 返 回 值：第一批颜色组中当前选中的数字字符
  * 说    明：第二组为放置编号；颜色选择索引范围为0至2
  */
static char Camera_ColorTaskDigitGet(void)
{
  return (char)('0' + QR_ColorGet(0U, Camera_ColorDigitIndex));
}

/**
  * 函    数：将任务码ASCII数字转换为摄像头颜色编号
  * 参    数：TaskCodeDigit 任务码中的数字字符
  * 返 回 值：对应的颜色枚举；0表示字符不在1~6
  * 说    明：先将ASCII字符转换为颜色编号1~6
  */
static Camera_ColorTypeDef Camera_ColorCodeGet(char TaskCodeDigit)
{
  uint8_t ColorNumber;

  if ((TaskCodeDigit < '1') || (TaskCodeDigit > '6'))
  {
    return (Camera_ColorTypeDef)0;
  }

  ColorNumber = (uint8_t)(TaskCodeDigit - '0');
  return (Camera_ColorTypeDef)ColorNumber;
}

/* IMU_TIMING_BEGIN: 主循环分段计时；仅保存数值，失败前不发送日志。 */
typedef enum
{
  IMU_TIME_CAMERA, IMU_TIME_MOTOR, IMU_TIME_ARM_VISION, IMU_TIME_MATERIAL,
  IMU_TIME_CHASSIS, IMU_TIME_READ, IMU_TIME_CAL, IMU_TIME_LOG,
  IMU_TIME_CONSOLE, IMU_TIME_SCREEN, IMU_TIME_QR_KEYS, IMU_TIME_OLED,
  IMU_TIME_COUNT
} ImuTimingStage_t;

static struct
{
  uint32_t current[IMU_TIME_COUNT], previous[IMU_TIME_COUNT], maximum[IMU_TIME_COUNT];
  uint32_t start_tick, mark_tick, previous_loop_ms, max_loop_ms;
  uint32_t failure_partial_ms;
  uint8_t active, frozen;
} ImuTiming;
static uint32_t ImuTiming_LastRunId;

static void ImuTiming_Begin(void)
{
  HWT101_CalStatus_t Cal;
  if (!HWT101_Cal_GetStatus(&Cal)) return;
  if (Cal.run_id != ImuTiming_LastRunId)
    memset(&ImuTiming, 0, sizeof(ImuTiming));
  ImuTiming_LastRunId = Cal.run_id;
  ImuTiming.active = !ImuTiming.frozen && (Cal.busy || Cal.control_ready);
  if (!ImuTiming.active) return;
  memset(ImuTiming.current, 0, sizeof(ImuTiming.current));
  ImuTiming.start_tick = ImuTiming.mark_tick = HAL_GetTick();
}

static void ImuTiming_Mark(ImuTimingStage_t Stage)
{
  uint32_t Now, Elapsed;
  if (!ImuTiming.active) return;
  Now = HAL_GetTick();
  Elapsed = (uint32_t)(Now - ImuTiming.mark_tick);
  ImuTiming.current[Stage] = Elapsed;
  if (Elapsed > ImuTiming.maximum[Stage]) ImuTiming.maximum[Stage] = Elapsed;
  ImuTiming.mark_tick = Now;
}

static void ImuTiming_End(void)
{
  if (!ImuTiming.active) return;
  ImuTiming.previous_loop_ms = (uint32_t)(HAL_GetTick() - ImuTiming.start_tick);
  if (ImuTiming.previous_loop_ms > ImuTiming.max_loop_ms)
    ImuTiming.max_loop_ms = ImuTiming.previous_loop_ms;
  memcpy(ImuTiming.previous, ImuTiming.current, sizeof(ImuTiming.previous));
}

static void ImuTiming_Freeze(void)
{
  if (!ImuTiming.frozen)
    ImuTiming.failure_partial_ms = ImuTiming.active ? (uint32_t)(HAL_GetTick() - ImuTiming.start_tick) : 0U;
  ImuTiming.active = 0U;
  ImuTiming.frozen = 1U;
}

static void ImuTiming_Report(const HWT101_CalStatus_t *Cal)
{
  static const char *const Names[IMU_TIME_COUNT] = {
    "camera", "motor_events", "arm_vision", "material", "chassis", "imu_read",
    "imu_cal", "imu_log", "console", "screen", "qr_keys", "oled"
  };
  char Text[192];
  int Length;
  unsigned Stage;
  /* 先冻结，故障日志与异步恢复不覆盖首次失败现场。 */
  ImuTiming_Freeze();
  Length = snprintf(Text, sizeof(Text),
      "\r\n[IMU DIAG] gap_ms=%lu max_gap_ms=%lu limit_ms=%lu "
      "loop_prev_ms=%lu loop_max_ms=%lu loop_partial_ms=%lu\r\n",
      (unsigned long)Cal->gap_ms, (unsigned long)Cal->max_gap_ms,
      (unsigned long)HWT101_DATA_FRESH_MS, (unsigned long)ImuTiming.previous_loop_ms,
      (unsigned long)ImuTiming.max_loop_ms, (unsigned long)ImuTiming.failure_partial_ms);
  if ((Length > 0) && (Length < (int)sizeof(Text)))
    (void)ConsoleTx_Write((uint8_t *)Text, (uint16_t)Length);
  for (Stage = 0U; Stage < IMU_TIME_COUNT; Stage++)
  {
    Length = snprintf(Text, sizeof(Text),
        "[IMU TIME] stage=%s prev_ms=%lu current_ms=%lu max_ms=%lu\r\n", Names[Stage],
        (unsigned long)ImuTiming.previous[Stage], (unsigned long)ImuTiming.current[Stage],
        (unsigned long)ImuTiming.maximum[Stage]);
    if ((Length > 0) && (Length < (int)sizeof(Text)))
      (void)ConsoleTx_Write((uint8_t *)Text, (uint16_t)Length);
  }
}
/* IMU_TIMING_END */

/**
  * 函    数：通过USART1上传HWT101最新角度
  * 参    数：无
  * 返 回 值：无
  * 说    明：默认只报告标定进度和结果；stream开启时每200ms更新最新角度槽，发送层每路至少间隔500ms。
  */
static void HWT101_Upload_Process(void)
{
  char UploadText[320];
  int UploadLength;
  uint32_t CurrentTick = HAL_GetTick();
  uint8_t Fresh;
  HWT101_Status_t Comm = {0};
  HWT101_CalStatus_t Cal;
  bool Terminal;

  if (!HWT101_Cal_GetStatus(&Cal)) return;
  (void)HWT101_Status_Get(&Comm);
  if (Cal.run_id != HWT101_UploadRunId)
  {
    HWT101_UploadRunId = Cal.run_id;
    HWT101_ProgressLastTick = CurrentTick;
  }
  if (Cal.state == HWT101_CAL_RESTORING && strcmp(Cal.reason, "none") && strcmp(Cal.reason, "cancelled"))
    ImuTiming_Freeze();
  Terminal = !Cal.busy && ((Cal.state == HWT101_CAL_DONE) ||
             (Cal.state == HWT101_CAL_FAILED) || (Cal.state == HWT101_CAL_CANCELLED));
  if (Terminal &&
      ((Cal.run_id != HWT101_UploadFinalRunId) || (Cal.state != HWT101_UploadFinalState)))
  {
    HWT101_UploadFinalRunId = Cal.run_id;
    HWT101_UploadFinalState = Cal.state;
    if (!strcmp(Cal.result, "ZERO_SAMPLE"))
    {
      UploadLength = snprintf(UploadText, sizeof(UploadText),
          "\r\nOK imu zero sample before_deg=%.3f after_deg=%.3f verified=0 control_ready=0\r\narm> ",
          (double)Cal.zero_before_deg, (double)Cal.zero_after_deg);
      if ((UploadLength > 0) && (UploadLength < (int)sizeof(UploadText)))
        (void)ConsoleTx_Write((uint8_t *)UploadText, (uint16_t)UploadLength);
      return;
    }
    if (Cal.state == HWT101_CAL_FAILED) ImuTiming_Report(&Cal);
    UploadLength = snprintf(UploadText, sizeof(UploadText),
        "\r\n[IMU CAL] result=%s state=%s reason=%s HAL=%u run_id=%lu verified=%u control_ready=%u\r\n",
        Cal.result, Cal.state_name, Cal.reason, (unsigned)Cal.hal,
        (unsigned long)Cal.run_id, (unsigned)Cal.verified, (unsigned)Cal.control_ready);
    if ((UploadLength > 0) && (UploadLength < (int)sizeof(UploadText)))
      (void)ConsoleTx_Write((uint8_t *)UploadText, (uint16_t)UploadLength);
    UploadLength = snprintf(UploadText, sizeof(UploadText),
        "[IMU SAVE] save_state=%s save_requested=%u save_readback_ok=%u persistent=UNTESTED "
        "normal_mode=%u bias_before=0x%04X bias_after=0x%04X bias_after_valid=%u\r\n",
        Cal.save_state, (unsigned)Cal.save_requested, (unsigned)Cal.save_readback_ok,
        (unsigned)Cal.normal_mode_confirmed, (unsigned)Cal.bias_before,
        (unsigned)Cal.bias_after, (unsigned)Cal.bias_after_valid);
    if ((UploadLength > 0) && (UploadLength < (int)sizeof(UploadText)))
      (void)ConsoleTx_Write((uint8_t *)UploadText, (uint16_t)UploadLength);
    UploadLength = snprintf(UploadText, sizeof(UploadText),
        "[IMU VERIFY] samples=%lu drift_dps=%.5f rms_deg=%.4f gap_ms=%lu max_gap_ms=%lu "
        "i2c_errors=%lu elapsed_ms=%lu relative_deg=%.3f verify_elapsed_ms=%lu\r\narm> ",
        (unsigned long)Cal.samples, (double)Cal.drift_dps, (double)Cal.rms_deg,
        (unsigned long)Cal.gap_ms, (unsigned long)Cal.max_gap_ms,
        (unsigned long)Cal.i2c_errors, (unsigned long)Cal.elapsed_ms,
        (double)Cal.relative_deg, (unsigned long)Cal.verify_elapsed_ms);
    if ((UploadLength > 0) && (UploadLength < (int)sizeof(UploadText)))
      (void)ConsoleTx_Write((uint8_t *)UploadText, (uint16_t)UploadLength);
  }
  if (Cal.busy && ((uint32_t)(CurrentTick - HWT101_ProgressLastTick) >= 5000U))
  {
    HWT101_ProgressLastTick = CurrentTick;
    UploadLength = snprintf(UploadText, sizeof(UploadText),
        "\r\n[IMU CAL] state=%s elapsed_s=%lu total_s=%lu verify_elapsed_s=%lu keep_still=1\r\n",
        Cal.state_name, (unsigned long)(Cal.elapsed_ms / 1000U), (unsigned long)(Cal.total_ms / 1000U),
        (unsigned long)(Cal.verify_elapsed_ms / 1000U));
    if ((UploadLength > 0) && (UploadLength < (int)sizeof(UploadText)))
      (void)ConsoleTx_Write((uint8_t *)UploadText, (uint16_t)UploadLength);
  }
  /* Busy calibration suppresses telemetry; sampling continues independently. */
  if (Cal.busy || ArmConsole_ImuStreamEnabled() == 0U) return;
  if (HWT101_Angle_Get(&HWT101_CurrentAngle)) HWT101_AngleValid = 1U;
  if ((uint32_t)(CurrentTick - HWT101_UploadLastTick) < HWT101_UPLOAD_INTERVAL_MS) return;
  Fresh = (HWT101_AngleValid != 0U) &&
          HWT101_Angle_Is_Fresh(&HWT101_CurrentAngle, HWT101_DATA_FRESH_MS);
  if (HWT101_AngleValid == 0U)
    UploadLength = snprintf(UploadText, sizeof(UploadText),
        "HWT101 yaw_deg=NA t_ms=%lu fresh=0 cal_state=%s i2c_errors=%lu\r\n",
        (unsigned long)CurrentTick, Cal.state_name, (unsigned long)Comm.i2c_error_count);
  else
    UploadLength = snprintf(UploadText, sizeof(UploadText),
        "HWT101 yaw_deg=%.2f count=%lu t_ms=%lu age_ms=%lu fresh=%u cal_state=%s i2c_errors=%lu\r\n",
        (double)HWT101_CurrentAngle.yaw, (unsigned long)HWT101_CurrentAngle.update_count,
        (unsigned long)CurrentTick, (unsigned long)(CurrentTick - HWT101_CurrentAngle.last_update_ms),
        (unsigned)Fresh, Cal.state_name, (unsigned long)Comm.i2c_error_count);
  if ((UploadLength > 0) && (UploadLength < (int)sizeof(UploadText)))
  {
    (void)ConsoleTx_Debug(CONSOLE_DEBUG_IMU, UploadText, (uint16_t)UploadLength);
    HWT101_UploadLastTick = CurrentTick;
  }
}

/** 按当前选中颜色启动机械臂对准；拒绝原因交给显示模块保留。 */
static void ArmVision_ColorAlignStart(void)
{
  Camera_ColorTypeDef Color;
  Mecanum_HeadingTestStatus_t Heading;

  if (HWT101_Cal_IsBusy())
  {
    OledUi_NoticeSet("IMU busy");
    return;
  }
  Mecanum_HeadingTest_StatusGet(&Heading);
  if (Heading.active)
  {
    OledUi_NoticeSet("Heading test busy");
    return;
  }

  if (Camera_ColorTaskReady == 0U)
  {
    OledUi_NoticeSet("Code required");
    return;
  }
  if (ArmVision_IsBusy() || MaterialVision_IsBusy() || MechanicalArm_IsBusy())
  {
    OledUi_NoticeSet("Busy");
    return;
  }
  if (!ArmVision_IsReferenceValid())
  {
    OledUi_NoticeSet("Ref required");
    return;
  }
  if (!ArmVision_IsCalibrated())
  {
    OledUi_NoticeSet("Cal required");
    return;
  }
  Color = Camera_ColorCodeGet(Camera_ColorTaskDigitGet());
  switch (ArmVision_MaterialStart(Color, ARM_VISION_JOB_ALIGN_ONLY))
  {
    case ARM_VISION_RESULT_OK: OledUi_NoticeSet(NULL); break;
    case ARM_VISION_RESULT_BUSY: OledUi_NoticeSet("Busy"); break;
    case ARM_VISION_RESULT_NOT_READY: OledUi_NoticeSet("Ref/Cal required"); break;
    case ARM_VISION_RESULT_PARAM_ERROR: OledUi_NoticeSet("Color invalid"); break;
    default: OledUi_NoticeSet("Camera tx"); break;
  }
}

/** Optional blocking I2C work yields to every active motor/control workflow. */
static bool OptionalDisplayReady(void)
{
  MotorBus_Diagnostic_t bus;
  MotorBus_Recovery_t recovery;
  MotorBus_DiagnosticGet(&bus);
  MotorBus_RecoveryGet(&recovery);
  return !HWT101_Cal_IsBusy() && !ChassisMotion_IsBusy() && !ChassisRoute_IsBusy() &&
         !GrabTask_IsBusy() && !GrabRoute_IsBusy() && !MechanicalArm_IsBusy() &&
         !ArmVision_IsBusy() && !MaterialVision_IsBusy() && !Mecanum_IsBusy() &&
         !ChassisMotion_StopPending() && !ArmConsole_OperationBusy() &&
         !bus.active && !recovery.active;
}

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
  (void)Camera_Init();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_UART5_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* CS 仍为高电平时提前使能 SPI1，验证首次读取的使能时序。 */
  __HAL_SPI_ENABLE(&hspi1);

  /*
   * 优先启动USART1控制台。即使OLED或其他可选外设没有连接，
   * 上位机仍然能够看到启动信息并使用调试命令。
   */
  if (ArmConsole_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USART1循环DMA接收；HT/TC/IDLE均由HAL分发。 */
  if (ConsoleRx_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

#if W25Q128_BOOT_DIAGNOSTIC
  W25Q128_StartupCheck();
#endif

  /* OLED为可选显示设备，未连接时不阻止其他模块和控制台运行。 */
  OledUi_Init(&hi2c1);

  QR_Init();
  Key_Init();
  if (HAL_UART_Receive_IT(&huart4, &QR_RxByte, 1U) != HAL_OK)
  {
    Error_Handler();
  }


  /* PE9使用TIM1_CH1输出50Hz PWM；CCR为0时夹爪上电不会立即动作。 */
  if (Steer_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  ArmVision_Init();
  MaterialVision_Init();
  GrabTask_Init();
  GrabRoute_Init();

  /* HWT101使用I2C2；设备仍在启动时，由轮询稍后重新探测。 */
  (void)HWT101_Init(&hi2c2);
  HWT101_Cal_Init();
  HWT101_UploadRunId = HWT101_UploadFinalRunId = 0U;
  HWT101_UploadFinalState = HWT101_CAL_IDLE;
  HWT101_CurrentAngle.yaw = 0.0f;
  HWT101_CurrentAngle.update_count = 0U;
  HWT101_AngleValid = 0U;
  HWT101_UploadLastTick = HAL_GetTick() - HWT101_UPLOAD_INTERVAL_MS;

  /* 机械臂电机总线使用UART5，模块初始化内部会启动ReceiveToIdle DMA接收。 */
  if (MotorBus_Init(&huart5) != HAL_OK) Error_Handler();
  ChassisRoute_Init();
  ChassisMotion_Init();
  ChassisTelemetry_Init();
  if (MechanicalArm_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }

  /* 比赛串口屏使用USART3 TX DMA，PD8连接屏幕RX，PD9连接屏幕TX。 */
  if (TJC_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  ContestScreen_Init();
  /* IMU boot verification is armed only after successful three-axis homing. */
  GrabTask_BootHomeStart(); /* Z collision -> X collision -> Base near; once per boot. */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    MechanicalArm_EventTypeDef MotorEvent;
    KeyEvent_t Key;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    ImuTiming_Begin();
    ConsoleRx_Process();
    ArmConsole_StopProcess(); /* Ctrl+C cancels producers before they can submit motion. */
    ConsoleTx_Process();
    if (ArmConsole_ResetPending())
    {
      ArmConsole_Process(); /* Consume a queued stop before committing reset. */
      if (ArmConsole_ResetProcess()) continue;
    }
    MotorBus_Process();
    Camera_Process();
    ImuTiming_Mark(IMU_TIME_CAMERA);
    MechanicalArm_Process();
    if (MechanicalArm_ResultGet(&MotorEvent) != 0U)
    {
      if ((GrabTask_MotorEventHandle(&MotorEvent) == 0U) &&
          (ArmVision_MotorEventHandle(&MotorEvent) == 0U) &&
          (MaterialVision_MotorEventHandle(&MotorEvent) == 0U))
      {
        ArmConsole_MotorEventHandle(&MotorEvent);
      }
    }
    ImuTiming_Mark(IMU_TIME_MOTOR);
    ArmVision_Process();
    ImuTiming_Mark(IMU_TIME_ARM_VISION);
    MaterialVision_Process();
    ImuTiming_Mark(IMU_TIME_MATERIAL);
    HWT101_Process(); /* 独立采集；关闭日志不会停止传感器轮询。 */
    ImuTiming_Mark(IMU_TIME_READ);
    HWT101_Cal_BootVerifyProcess(!Mecanum_IsBusy() && !MechanicalArm_IsBusy() &&
        !ArmVision_IsBusy() && !MaterialVision_IsBusy() &&
        !ChassisRoute_IsBusy() && !ChassisMotion_IsBusy() &&
        !GrabTask_IsBusy() && !GrabRoute_IsBusy());
    HWT101_Cal_Process();
    ImuTiming_Mark(IMU_TIME_CAL);
    ChassisMotion_HeadingProcess();
    ChassisLocalization_Process();
    ChassisMotion_Process();
    ChassisRoute_Process();
    GrabRoute_Process();
    GrabTask_Process();
    Mecanum_Velocity_Process(); /* 航向控制使用本圈已检查的模块角度。 */
    ImuTiming_Mark(IMU_TIME_CHASSIS);
    ChassisTelemetry_Process();
    HWT101_Upload_Process();
    ImuTiming_Mark(IMU_TIME_LOG);
    ArmConsole_Process();
    if (ArmConsole_ResetPending()) continue;
    ImuTiming_Mark(IMU_TIME_CONSOLE);
    TJC_Process();
    if (!ChassisMotion_IsBusy() && !ChassisRoute_IsBusy()) ContestScreen_Process();
    ImuTiming_Mark(IMU_TIME_SCREEN);

    if (QR_TaskCodeGet(QR_TaskCode) == 1U)
    {
      (void)ContestScreen_TaskCodeSet(QR_TaskCode);
      (void)ContestScreen_ProgressSet(0U, 0U);
      Camera_ColorTaskReady = 1U;
      Camera_ColorDigitIndex = 0U;
      OledUi_TaskCodeSet(QR_TaskCode, Camera_ColorDigitIndex);
    }

    Key_Scan();
    Key = Key_Get_Press_Event(); /* 每圈只消费一次按键事件。 */
    if (!ArmConsole_OperationBusy() && !GrabTask_IsBusy() && !GrabRoute_IsBusy() &&
        (Key == KEY_EVENT_PE4 || Key == KEY_EVENT_PE5 ||
         (!ChassisMotion_IsBusy() && !ChassisRoute_IsBusy())) &&
        OledUi_KeyHandle(Key) == 0U)
    {
      switch (Key)
      {
        case KEY_EVENT_PE2:
          ArmVision_ColorAlignStart();
          break;

        case KEY_EVENT_PE3:
          if (Camera_ColorTaskReady && !ArmVision_IsBusy() &&
              !MaterialVision_IsBusy() && !MechanicalArm_IsBusy())
          {
            Camera_ColorDigitIndex = (uint8_t)((Camera_ColorDigitIndex + 1U) % 3U);
            OledUi_TaskCodeSet(QR_TaskCode, Camera_ColorDigitIndex);
          }
          break;

        default:
          break;
      }
    }
    ImuTiming_Mark(IMU_TIME_QR_KEYS);
    if (OptionalDisplayReady()) OledUi_Process(); /* Display I/O only in control-idle gaps. */
    ImuTiming_Mark(IMU_TIME_OLED);
    MotorBus_Process();
    ConsoleTx_Process();
    ImuTiming_End();
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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief    分发HAL串口单字节接收完成事件
  * @param    huart ：触发回调的串口句柄
  * @retval   无
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  TJC_Rx_Callback(huart);
  if (huart->Instance == UART4)
  {
    QR_ReceiveData(QR_RxByte);
    (void)HAL_UART_Receive_IT(&huart4, &QR_RxByte, 1U);
  }

}

/**
  * @brief    分发HAL串口DMA空闲接收事件
  * @param    huart ：触发回调的串口句柄
  * @param    Size  ：HAL提供的DMA写入位置（USART1读取实时NDTR防止重复事件）
  * @retval   无
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == UART5)
  {
    MotorBus_RxEventCallback(huart, Size);
  }
  else if (huart->Instance == USART1)
  {
    ConsoleRx_RxEventCallback(huart);
  }
}

/**
  * @brief    分发HAL串口DMA发送完成事件
  * @param    huart ：触发回调的串口句柄
  * @retval   无
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    MotorBus_TxCpltCallback(huart);
  }
  else if (huart->Instance == USART1)
  {
    ConsoleTx_TxCpltCallback(huart);
  }
  else if (huart->Instance == USART3)
  {
    TJC_Tx_Callback(huart);
  }
}

/**
  * @brief    分发HAL串口通信错误事件
  * @param    huart ：触发回调的串口句柄
  * @retval   无
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  TJC_Error_Callback(huart);
  if (huart->Instance == USART1)
  {
    ConsoleRx_ErrorCallback(huart);
    /* RX overrun/framing errors do not cancel a healthy TX DMA. */
    if (huart->ErrorCode & HAL_UART_ERROR_DMA) ConsoleTx_ErrorCallback(huart);
  }
  else if (huart->Instance == UART4)
  {
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    QR_ErrorCallback();
    (void)HAL_UART_Receive_IT(&huart4, &QR_RxByte, 1U);
  }


  else if (huart->Instance == UART5)
  {
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    MotorBus_ErrorCallback(huart);
  }
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
