#include "arm_console.h"

#include "mechanical_arm.h"
#include "motor_bus.h"
#include "mechanical_arm_config.h"
#include "Arm.h"
#include "GrabTask.h"
#include "GrabRoute.h"
#include "turntable.h"
#include "mecanum_chassis.h"
#include "Camera.h"
#include "QR.h"
#include "hwt101_i2c.h"
#include "hwt101_calibration.h"
#include "tjc_screen.h"
#include "oled_ui.h"
#include "console_tx.h"
#include "console_rx.h"
#include "chassis_motion.h"
#include "chassis_config.h"
#include "chassis_route.h"
#include "chassis_telemetry.h"
#include "radar_console.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ARM_CONSOLE_RING_SIZE 512U
/* USART1 DMA回调写入的软件环形缓冲区长度。 */

#define ARM_CONSOLE_LINE_SIZE 64U
/* 单条终端命令允许占用的最大缓冲区长度。 */

#define ARM_CONSOLE_TOKEN_COUNT 6U
/* 一条命令解析时允许的最大字段数量。 */

#define ARM_CONSOLE_TX_SIZE 256U
/* 单次ASCII回复格式化缓冲区长度。 */

#define ARM_CONSOLE_CAMERA_TRACE_INTERVAL_MS 100U
/* 视觉调试数据的最小输出间隔，避免串口打印拖慢主循环。 */

#define ARM_CONSOLE_BACKWARD_TEST_SPEED_MM_S 30.0f
/* 独立底盘后退方向测试速度。 */

static UART_HandleTypeDef *ArmConsole_Uart;
static uint8_t ArmConsole_ImuStream;
static uint8_t ResetPending, ResetAckDrained;
static uint32_t ResetStarted, ResetDrainTick, ResetDropBaseline;
static uint8_t ArmConsole_RingBuffer[ARM_CONSOLE_RING_SIZE];
static volatile uint16_t ArmConsole_RingWrite;
static volatile uint16_t ArmConsole_RingRead;
static volatile uint8_t ArmConsole_RingOverflow;
static volatile uint8_t ArmConsole_RxDiscard;
static volatile uint32_t ArmConsole_RxOverflowCount;
static volatile uint16_t ArmConsole_RxPeak;
static volatile uint8_t ArmConsole_StopPending;
static char ArmConsole_LineBuffer[ARM_CONSOLE_LINE_SIZE];
static uint8_t ArmConsole_LineLength;
static uint8_t ArmConsole_LineOverflow;
static uint8_t ArmConsole_LastWasCr;
static uint8_t ArmConsole_CameraTrace;
static uint32_t ArmConsole_CameraTraceSequence;
static uint32_t ArmConsole_CameraTraceLastTick;
static uint8_t PixelLogEnabled;
static uint32_t PixelLogId, PixelLogCount, PixelLogQueued, PixelLogDropped;
static uint8_t ArmConsole_QrStream;
static QR_SnapshotTypeDef ArmConsole_QrPending;
static uint32_t ArmConsole_QrSeenSequence;
static uint32_t ArmConsole_QrLastEventTick;
static uint32_t ArmConsole_QrSkipped;
static uint8_t BusRecoveryPending;
static uint32_t BusRecoveryStarted;
static struct {
  bool active, pending;
  uint8_t address;
  uint16_t rounds;
  uint32_t attempted, sent, received, timeouts, uart_errors, rejected;
  uint32_t token, started, elapsed, next_tick, wait_since, reply_max, last_uart_error;
  const char *reason;
} BusTest;
static uint32_t BusTestToken;
static void ArmConsole_BusTestStop(const char *reason);
static uint8_t ManualGuardActive, ManualGuardExecuting, ManualGuardAddress, ManualGuardLast, ManualGuardKind;
static uint32_t ManualGuardStarted;
static char ManualGuardLine[ARM_CONSOLE_LINE_SIZE];
static bool ArmConsole_RadarMotionGuard(unsigned n,char *t[])
{
  static const char *const moves[]={"run","heading","hold","move","velocity","forward","backward","left","right"};
  unsigned i;
  if(!RadarConsole_ScanBusy() || n<2U) return false;
  if(!strcmp(t[0],"wheel") && strcmp(t[1],"status") && strcmp(t[1],"stop")) return true;
  if(!strcmp(t[0],"grab") && (!strcmp(t[1],"start") || !strcmp(t[1],"route") ||
     !strcmp(t[1],"mission") || !strcmp(t[1],"place") || !strcmp(t[1],"pose") || !strcmp(t[1],"store") ||
     !strcmp(t[1],"take") || !strcmp(t[1],"return") || !strcmp(t[1],"align") ||
     !strcmp(t[1],"stack") || !strcmp(t[1],"ring"))) return true;
  if(n>=3U && !strcmp(t[0],"grab") && !strcmp(t[1],"track") && !strcmp(t[2],"start")) return true;
  if(n==3U && !strcmp(t[0],"grab") && !strcmp(t[1],"calib") &&
     strcmp(t[2],"status") && strcmp(t[2],"ref")) return true;
  if(strcmp(t[0],"chassis")) return false;
  for(i=0;i<sizeof(moves)/sizeof(moves[0]);i++) if(!strcmp(t[1],moves[i])) return true;
  return n>=3U && !strcmp(t[1],"route") && (!strcmp(t[2],"start") || !strcmp(t[2],"auto") || !strcmp(t[2],"next"));
}

/* Held static operations retain their owned feedback token; no manual motion producer. */
static bool ArmConsole_RingCommandHandle(unsigned count,char *tokens[])
{
  if(count>=3U && !strcmp(tokens[0],"grab") && !strcmp(tokens[1],"stack"))
    return GrabTask_Command(count,tokens);
  if(count!=4U || strcmp(tokens[0],"grab") || strcmp(tokens[1],"ring") ||
     strcmp(tokens[2],"place") || !GrabTask_RingManualAllowed())return false;
  return GrabTask_Command(count,tokens);
}
static uint8_t ArmConsole_CommandExecute(char *Line);

static void ArmConsole_ManualGuardCancel(void)
{
  if (ManualGuardActive) MotorBus_Cancel(MOTOR_BUS_GUARD);
  ManualGuardActive=0U;
}

/**
  * 函    数：向USART1输出一段ASCII文本
  * 参    数：Text 以零结尾的字符串
  * 返 回 值：无
  * 说    明：主循环复制到回复队列，由USART1 DMA发送，不等待线路发送结束
  */
static void ArmConsole_Write(const char *Text)
{
  if ((ArmConsole_Uart == NULL) || (Text == NULL))
  {
    return;
  }
  (void)ConsoleTx_Write((const uint8_t *)Text, (uint16_t)strlen(Text));
}

/**
  * 函    数：格式化并输出ASCII回复
  * 参    数：Format printf格式字符串及可变参数
  * 返 回 值：无
  * 说    明：回复长度超过缓冲区时自动截断，避免动态内存
  */
static void ArmConsole_Printf(const char *Format, ...)
{
  char Buffer[ARM_CONSOLE_TX_SIZE];
  va_list Arguments;

  va_start(Arguments, Format);
  (void)vsnprintf(Buffer, sizeof(Buffer), Format, Arguments);
  va_end(Arguments);
  ArmConsole_Write(Buffer);
}

/**
  * 函    数：输出终端提示符
  * 参    数：无
  * 返 回 值：无
  * 说    明：异步电机命令完成后再输出下一提示符
  */
static void ArmConsole_PromptShow(void)
{
  ArmConsole_Write("arm> ");
}

/**
  * 函    数：开始独立摄像头测试数据输出
  * 参    数：无
  * 返 回 值：无
  * 说    明：独立识别输出，直到camera stop或任务接管才停止
  */
static void ArmConsole_CameraTraceDirectStart(void)
{
  Camera_SnapshotTypeDef Snapshot;

  ArmConsole_CameraTrace = 1U;
  ArmConsole_CameraTraceLastTick = HAL_GetTick() -
                                   ARM_CONSOLE_CAMERA_TRACE_INTERVAL_MS;
  Camera_SnapshotGet(&Snapshot);
  ArmConsole_CameraTraceSequence = Snapshot.Data.Sequence;
}

/**
  * 函    数：停止视觉调试数据输出
  * 参    数：无
  * 返 回 值：无
  * 说    明：视觉任务停止、结束或摄像头测试停止时调用
  */
static void ArmConsole_CameraTraceStop(void)
{
  ConsoleTx_DebugCancel(CONSOLE_DEBUG_VISION);
  ArmConsole_CameraTrace = 0U;
  ArmConsole_CameraTraceSequence = 0U;
  ArmConsole_CameraTraceLastTick = 0U;
}

/**
  * 函    数：输出独立相机识别期间收到的新坐标
  * 参    数：无
  * 返 回 值：无
  * 说    明：在标定和自动对准期间限频输出，不清除OLED待刷新数据
  */
static void ArmConsole_CameraTraceShow(void)
{
  char text[192];
  int length;
  Camera_SnapshotTypeDef Snapshot;
  Camera_DataTypeDef *Data;
  uint32_t Now;

  if (ArmConsole_CameraTrace == 0U)
  {
    return;
  }
  Camera_SnapshotGet(&Snapshot);
  Data = &Snapshot.Data;
  if ((Camera_VisualStateGet(&Snapshot) != CAMERA_VIS_OK) ||
      (Snapshot.HasValidData == 0U) || (Snapshot.TargetValid == 0U) ||
      (Data->Sequence == ArmConsole_CameraTraceSequence))
  {
    return;
  }
  Now = HAL_GetTick();
  if ((Now - ArmConsole_CameraTraceLastTick) <
      ARM_CONSOLE_CAMERA_TRACE_INTERVAL_MS)
  {
    return;
  }
  ArmConsole_CameraTraceLastTick = Now;
  ArmConsole_CameraTraceSequence = Data->Sequence;
  length = snprintf(text, sizeof(text), "VISION DATA fn=0x%02X target=0x%02X\r\nCX=%d CY=%d DX=%d DY=%d seq=%lu\r\n",
                     (unsigned int)Data->Function,
                     (unsigned int)Data->Target,
                     (int)Data->CX,
                     (int)Data->CY,
                     (int)Data->DX,
                     (int)Data->DY,
                     (unsigned long)Data->Sequence);
  if (length > 0 && length < (int)sizeof(text))
    (void)ConsoleTx_Debug(CONSOLE_DEBUG_VISION, text, (uint16_t)length);
}

/* Each parsed frame is copied into the existing console FIFO, never sampled
 * from the latest-only camera snapshot. Stop replies keep their reserved queue.
 * Limit pixel backlog to half the FIFO; overflow is explicit, never a control wait. */
static void ArmConsole_PixelFrame(const uint8_t *frame,uint32_t received,uint8_t matched)
{
  char line[112];ConsoleTx_Stats_t stats;uint32_t crc=0xFFFFFFFFUL;
  uint32_t index=PixelLogCount++;
  unsigned cx=((unsigned)frame[4]<<8)|frame[5],cy=((unsigned)frame[6]<<8)|frame[7];
  int length=snprintf(line,sizeof line,"PIX id=%lu i=%lu t=%lu c=%u v=%u m=%u x=%u y=%u",
    (unsigned long)PixelLogId,(unsigned long)index,(unsigned long)received,frame[2],frame[3],matched,cx,cy);
  if(length<=0 || length>=(int)sizeof(line)-16){PixelLogDropped++;return;}
  for(int n=0;n<length;n++){
    crc^=(uint8_t)line[n];for(unsigned bit=0;bit<8;bit++)crc=(crc>>1)^((crc&1U)?0xEDB88320UL:0U);
  }
  int tail=snprintf(line+length,sizeof(line)-(unsigned)length," crc=%08lX\r\n",(unsigned long)~crc);
  ConsoleTx_GetStats(&stats);
  if(tail<=0 || stats.reply_pending>4096U ||
     !ConsoleTx_Write((const uint8_t *)line,(uint16_t)(length+tail)))PixelLogDropped++;
  else PixelLogQueued++;
}

static uint8_t ArmConsole_PixelLogCommand(unsigned count,char *tokens[])
{
  if(count<2 || strcmp(tokens[0],"camera") || strcmp(tokens[1],"log"))return 0U;
  if(count!=3 || (strcmp(tokens[2],"start") && strcmp(tokens[2],"stop") && strcmp(tokens[2],"status")))
  {ArmConsole_Write("ERR camera log start|stop|status\r\n");return 1U;}
  if(!strcmp(tokens[2],"start")){
    if(!PixelLogEnabled){if(++PixelLogId==0)PixelLogId=1;PixelLogCount=PixelLogQueued=PixelLogDropped=0;}
    PixelLogEnabled=1;Camera_FrameObserverSet(ArmConsole_PixelFrame);
    ArmConsole_Printf("OK camera log_begin id=%lu next=%lu mode=per_frame fields=rx,target,valid,matched,cx,cy\r\n",
      (unsigned long)PixelLogId,(unsigned long)PixelLogCount);
  }else if(!strcmp(tokens[2],"stop")){
    PixelLogEnabled=0;Camera_FrameObserverSet(NULL);
    ArmConsole_Printf("OK camera log_end id=%lu count=%lu queued=%lu dropped=%lu\r\n",
      (unsigned long)PixelLogId,(unsigned long)PixelLogCount,(unsigned long)PixelLogQueued,(unsigned long)PixelLogDropped);
  }else ArmConsole_Printf("OK camera log_status enabled=%u id=%lu count=%lu queued=%lu dropped=%lu\r\n",
    PixelLogEnabled,(unsigned long)PixelLogId,(unsigned long)PixelLogCount,(unsigned long)PixelLogQueued,(unsigned long)PixelLogDropped);
  return 1U;
}

/**
  * 函    数：从环形缓冲区取出一个接收字节
  * 参    数：Data 字节输出地址
  * 返 回 值：1取到数据，0为空，2先处理Ctrl+C，3接收丢失需重同步（不消费后续字节）
  * 说    明：短临界区防止Ctrl+C回调清空缓冲区时读指针竞争
  */
static uint8_t ArmConsole_RingPop(uint8_t *Data)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (ArmConsole_StopPending)
  {
    __set_PRIMASK(primask);
    return 2U;
  }
  if (ArmConsole_RingOverflow) {
    ArmConsole_RingOverflow = 0U;
    ArmConsole_LineLength = ArmConsole_LastWasCr = 0U;
    ArmConsole_LineOverflow = 1U;
    __set_PRIMASK(primask);
    return 3U;
  }
  if ((Data == NULL) || (ArmConsole_RingRead == ArmConsole_RingWrite))
  {
    __set_PRIMASK(primask);
    return 0U;
  }

  *Data = ArmConsole_RingBuffer[ArmConsole_RingRead];
  ArmConsole_RingRead = (uint16_t)((ArmConsole_RingRead + 1U) % ARM_CONSOLE_RING_SIZE);
  __set_PRIMASK(primask);
  return 1U;
}

/**
  * 函    数：将命令行切分为空格分隔字段
  * 参    数：Line 可修改的命令行；Tokens 字段地址数组
  * 返 回 值：实际字段数量；字段过多时返回0
  * 说    明：连续空格被忽略，不支持引号和转义字符
  */
static uint8_t ArmConsole_Tokenize(char *Line, char *Tokens[ARM_CONSOLE_TOKEN_COUNT])
{
  uint8_t Count;
  char *Token;

  Count = 0U;
  Token = strtok(Line, " \t");
  while (Token != NULL)
  {
    if (Count >= ARM_CONSOLE_TOKEN_COUNT)
    {
      return 0U;
    }
    Tokens[Count] = Token;
    Count++;
    Token = strtok(NULL, " \t");
  }
  return Count;
}

/**
  * 函    数：完整解析一个十进制有符号整数
  * 参    数：Text 输入文本；Value 解析结果地址
  * 返 回 值：1表示完整转换成功，0表示格式或范围错误
  * 说    明：同时检查strtol尾指针以及32位有符号范围
  */
static uint8_t ArmConsole_IntegerParse(const char *Text, int32_t *Value)
{
  char *End;
  long ParsedValue;

  if ((Text == NULL) || (Value == NULL) || (Text[0] == '\0'))
  {
    return 0U;
  }
  errno = 0;
  ParsedValue = strtol(Text, &End, 10);
  if ((*End != '\0') || (errno == ERANGE))
  {
    return 0U;
  }
  *Value = (int32_t)ParsedValue;
  return 1U;
}

/**
  * 函    数：完整解析一个浮点数
  * 参    数：Text 输入文本；Value 解析结果地址
  * 返 回 值：1表示完整转换成功，0表示格式错误
  * 说    明：用于夹爪占空比和机械臂角度命令
  */
static uint8_t ArmConsole_FloatParse(const char *Text, float *Value)
{
  char *End;
  double ParsedValue;

  if ((Text == NULL) || (Value == NULL) || (Text[0] == '\0'))
  {
    return 0U;
  }
  errno = 0;
  ParsedValue = strtod(Text, &End);
  if ((*End != '\0') || (errno == ERANGE) || (ParsedValue != ParsedValue))
  {
    return 0U;
  }
  *Value = (float)ParsedValue;
  return 1U;
}

/**
  * 函    数：将电机轴角度转换为位置命令脉冲
  * 参    数：Axis 目标轴；Degree 相对转动角度；Pulses 脉冲结果地址
  * 返 回 值：1表示转换成功，0表示角度超限或小于一个可执行脉冲
  * 说    明：Base/X保留单圈角度校验；Z轴不限制角度，底层仍检查脉冲上限
  */
static uint8_t ArmConsole_DegreeToPulse(MechanicalArm_AxisTypeDef Axis,
                                         float Degree,
                                         int32_t *Pulses)
{
  float PulseValue;

  if ((Pulses == NULL) ||
      ((Axis != MECHANICAL_ARM_AXIS_Z) &&
       ((Degree < -MECHANICAL_ARM_DEGREES_PER_REV) ||
        (Degree > MECHANICAL_ARM_DEGREES_PER_REV))) ||
      (Degree == 0.0f))
  {
    return 0U;
  }

  PulseValue = Degree * (float)MECHANICAL_ARM_COMMAND_PULSES_PER_REV /
               MECHANICAL_ARM_DEGREES_PER_REV;
  if (PulseValue > 0.0f)
  {
    *Pulses = (int32_t)(PulseValue + 0.5f);
  }
  else
  {
    *Pulses = (int32_t)(PulseValue - 0.5f);
  }

  return (*Pulses != 0) ? 1U : 0U;
}

/**
  * 函    数：将位置命令脉冲转换为电机轴角度
  * 参    数：Pulses 带方向的位置脉冲
  * 返 回 值：对应的电机轴角度
  * 说    明：仅用于控制台回显，不改变底层电机协议
  */
static float ArmConsole_PulseToDegree(int32_t Pulses)
{
  return (float)Pulses * MECHANICAL_ARM_DEGREES_PER_REV /
         (float)MECHANICAL_ARM_COMMAND_PULSES_PER_REV;
}

/**
  * 函    数：将回零模式名称转换为协议模式值
  * 参    数：Text 模式名称；HomeMode 模式值输出地址
  * 返 回 值：1表示识别成功，0表示模式名称非法
  * 说    明：near、dir、collision、limit分别对应协议模式0至3
  */
static uint8_t ArmConsole_HomeModeParse(const char *Text, uint8_t *HomeMode)
{
  if ((Text == NULL) || (HomeMode == NULL))
  {
    return 0U;
  }
  if (strcmp(Text, "near") == 0)
  {
    *HomeMode = 0U;
  }
  else if ((strcmp(Text, "dir") == 0) || (strcmp(Text, "direction") == 0))
  {
    *HomeMode = 1U;
  }
  else if (strcmp(Text, "collision") == 0)
  {
    *HomeMode = 2U;
  }
  else if (strcmp(Text, "limit") == 0)
  {
    *HomeMode = 3U;
  }
  else
  {
    return 0U;
  }
  return 1U;
}

/**
  * 函    数：取得回零模式值对应的ASCII名称
  * 参    数：HomeMode 回零模式值
  * 返 回 值：模式名称常量字符串
  * 说    明：非法模式返回invalid
  */
static const char *ArmConsole_HomeModeNameGet(uint8_t HomeMode)
{
  static const char *HomeModeName[4] =
  {
    "near", "dir", "collision", "limit"
  };

  if (HomeMode > 3U)
  {
    return "invalid";
  }
  return HomeModeName[HomeMode];
}

/**
  * 函    数：输出帮助命令列表
  * 参    数：无
  * 返 回 值：无
  * 说    明：全部内容使用ASCII，避免串口工具编码差异
  */
static void ArmConsole_HelpShow(void)
{
  ArmConsole_Write("console status | chassis snapshot (read-only diagnostics)\r\n");
  ArmConsole_Write("bus recover (stop tasks, verify motors, never resume) | bus status\r\n");
  ArmConsole_Write("bus test <id 1..8> [rounds 1..100] | bus test status | bus test stop (read only)\r\n");
  ArmConsole_Write("turntable get/set <key> [value] | enable | disable | save_zero | home | origin | index <slot> | jog <degree> | stop | status\r\n");
  ArmConsole_Write("grab store proc <slot> <color> | grab take <slot> rough|temp1|temp2 | grab return\r\n");
  /* 按命令组入队，由USART1 DMA发送；不占用底盘控制周期等待串口。 */
  ArmConsole_Write("chassis move <map_dx_mm> <map_dy_mm> (norm 1..300 mm, qualified feedback)\r\n");
  ArmConsole_Printf("chassis route start|auto [10..5000mm/s, default 200] | next | status | cancel (15 targets, auto dwell %lums)\r\n",
                    (unsigned long)CHASSIS_ROUTE_AUTO_DWELL_MS);
  ArmConsole_Write(
      "chassis run <vx_mm_s> <vy_mm_s> <omega_rad_s> <hold_ms>\r\n"
      "  omega=0: hold starting IMU heading; verified fresh IMU required\r\n"
      "chassis heading <vx_mm_s> <vy_mm_s> <module_heading_deg> <hold_ms>\r\n"
      "chassis turn <map_heading_deg -180..180> (feedback angle and stopped wheels)\r\n"
      "chassis origin <map_x_mm> <map_y_mm> <map_heading_deg> (verified IMU, stationary)\r\n"
      "chassis profile receive (compatibility only; Emm42 default)\r\n"
      "chassis feedback <0..4> (0=all, 1..4=single, polling stays off)\r\n"
      "chassis feedback on|off\r\n"
      "chassis feedback (read cached raw values, valid=speed,position,state)\r\n"
      "chassis units 65536 (Emm42 protocol units per motor revolution) | 0 (clear confirmation)\r\n"
      "chassis stream on <nonzero_session_u32> | off\r\n"
      "chassis task | stop [client_token] | stop-status; Ctrl+C stops chassis\r\n"
      "chassis reset-confirmed (ONLY after physical driver reset)\r\n");
  ArmConsole_Write(
      "help\r\n"
      "info\r\n"
      "enable <base|z|x|all>\r\n"
      "disable <base|z|x|all>\r\n"
      "pos <base|z|x> <degree>  z angle has no fixed limit\r\n"
      "config <base|z|x|all>\r\n"
      "config <base|z|x|all> <rpm> <acc> <limit_pulses>\r\n"
      "config z ... limit_pulses=0 means unlimited\r\n"
      "state <base|z|x|all>\r\n"
      "position <base|z|x>\r\n"
      "origin <base|z|x|all>  save mechanical zero\r\n"
      "home <base|z|x|all> <near|dir|collision|limit>\r\n"
      "zero <base|z|x|all>    clear current position only\r\n"
      "stop <base|z|x|all>\r\n"
      "grip duty <2.5..12.5>\r\n"
      "grip <idle|open|catch>\r\n");
  ArmConsole_Write(
      "camera ring <1|2|3> | camera material <1|2|3|4|5|6>  recognition only, no motion\r\n"
      "camera status  readonly recognition and USB statistics\r\n"
      "camera log start|stop|status  passive per-frame pixel recording\r\n"
      "camera stop\r\n"
      "imu status     readonly HWT101 angle and I2C statistics\r\n"
      "imu cal start  keep still: 20s native calibration + 30s verification\r\n"
      "imu cal cancel  cancel and clear boot verification\r\n"
      "system reset  restart STM32 when motion and IMU tasks are idle\r\n"
      "imu zero  zero Z angle only; reverify and reset map origin afterwards\r\n"
      "imu verify  zero yaw then verify for 5s\r\n"
      "imu stream on|off  continuous angle log (default off)\r\n"
      "screen status  readonly USART3 statistics\r\n"
      "qr status  UART4 bytes, accepted/rejected frames and event drops\r\n"
      "qr read  latest task code, non-consuming\r\n"
      "grab qr begin [upper|lower] [10..200mm/s] | status | stop (Z-only scan)\r\n"
      "qr stream on|off  future task codes, <=5Hz, default off\r\n"
      "grab calib ref | grab calib <1..6> | grab calib status; cancel: grab stop\r\n");
  ArmConsole_Write(
      "chassis backward  continuous test, stop with chassis stop\r\n"
      "chassis <forward|backward> <1..200mm>\r\n"
      "chassis velocity <forward_mm_s> <left_mm_s> <yaw_deg_s>\r\n"
      "chassis stop\r\n"
      "chassis hold <-100..100mm/s> <1..10s>  verified IMU heading test\r\n"
      "chassis status  heading test state\r\n"
      "wheel <fl|rl|rr|fr> <-100..100rpm>\r\n"
      "wheel stop\r\n");
  ArmConsole_Write("grab start fixed|align|pick [color 1..6] | grab route [speed_mm_s, default 200]\r\n"
                   "grab status | grab stop | grab rehome | grab set <key> <value> | grab get <key>\r\n");
  ArmConsole_Write("grab ring align [1|2|3] [z_mm] [stay] | grab ring place <z_mm>\r\n");
  ArmConsole_Write("grab mission [speed_mm_s, default 200] | grab mission continue | grab pose rear|work | grab place car\r\n");
  ArmConsole_Write("grab track observe [color] | reference | start [color] | stop | status | trace [index]\r\n"
                   "grab track begin [color] [remaining_mm] | grab track pick [color] [z_mm] [speed_mm_s]\r\n"
                   "grab track visual|feedback [color] [remaining_mm] (follow only; stop before switching)\r\n"
                   "grab track log start [50..1000 ms] | stop | status (passive diagnostics, default off)\r\n"
                   "grab track get <key> [predict|visual|feedback] | set <key> <value> [profile] (independent RAM)\r\n");
}

/**
  * 函    数：输出一根轴的RAM配置
  * 参    数：Axis 目标轴
  * 返 回 值：无
  * 说    明：配置仅影响后续pos命令，复位后恢复默认
  */
static void ArmConsole_ConfigShow(MechanicalArm_AxisTypeDef Axis)
{
  MechanicalArm_ConfigTypeDef Config;

  if (MechanicalArm_ConfigGet(Axis, &Config) == 0U)
  {
    return;
  }
  if ((Axis == MECHANICAL_ARM_AXIS_Z) && (Config.PulseLimit == 0U))
  {
    ArmConsole_Printf("OK config %s rpm=%u acc=%u limit_pulses=unlimited\r\n",
                      MechanicalArm_AxisNameGet(Axis),
                      (unsigned int)Config.SpeedRpm,
                      (unsigned int)Config.Acceleration);
  }
  else
  {
    ArmConsole_Printf("OK config %s rpm=%u acc=%u limit_pulses=%lu\r\n",
                      MechanicalArm_AxisNameGet(Axis),
                      (unsigned int)Config.SpeedRpm,
                      (unsigned int)Config.Acceleration,
                      (unsigned long)Config.PulseLimit);
  }
}

/**
  * 函    数：输出工程串口和三轴映射信息
  * 参    数：无
  * 返 回 值：无
  * 说    明：便于上位机确认USART1、UART5和电机ID配置
  */
static void ArmConsole_BuildShow(void)
{
  ArmConsole_Write("BUILD motorbus_diag=1 date=" __DATE__ " time=" __TIME__ "\r\n");
  ArmConsole_Printf("GRAB protocol=B2 align_color=%u pick_color=%u\r\n",
    (unsigned)GRAB_ALIGN_COLOR,(unsigned)GRAB_PICK_COLOR);
}

/** 输出首次总线锁定原因；仅查询RAM，不访问驱动器、不解除锁定。 */
static void ArmConsole_MotorBusShow(void)
{
  MotorBus_Diagnostic_t Diagnostic;
  MotorBus_DiagnosticGet(&Diagnostic);
  ArmConsole_Printf("[MOTOR BUS] locked=%u reason=%s fault_ms=%lu owner=%u "
                   "addr=%u func=0x%02X active=%u wait=%u uart_error=0x%08lX\r\n",
                   (unsigned)Diagnostic.locked, Diagnostic.reason,
                   (unsigned long)Diagnostic.tick_ms, (unsigned)Diagnostic.owner,
                   (unsigned)Diagnostic.address, (unsigned)Diagnostic.function,
                   (unsigned)Diagnostic.active, (unsigned)Diagnostic.waiting_reply,
                   (unsigned long)Diagnostic.uart_error);
  if (Diagnostic.startup_fe_recovered)
    ArmConsole_Printf("[MOTOR BUS STARTUP] recovered_fe=%lu last_ms=%lu rx_restarted=1\r\n",
                      (unsigned long)Diagnostic.startup_fe_recovered,
                      (unsigned long)Diagnostic.startup_fe_last_ms);
}

static void ArmConsole_InfoShow(void)
{
  ArmConsole_Write("ARM STM32F407 USART1=115200 UART5=115200\r\n");
  ArmConsole_BuildShow();
  ArmConsole_MotorBusShow();
  ArmConsole_Write("chassis FL=id1 RL=id2 RR=id3 FR=id4\r\n");
  ArmConsole_Write("arm base=id5 z=id6 x=id7 pos=relative unit=degree\r\n");
  ArmConsole_Printf("arm positive_dir base=%u z=%u x=%u (0=CW 1=CCW)\r\n",
                   (unsigned)MECHANICAL_ARM_BASE_POSITIVE_DIR,
                   (unsigned)MECHANICAL_ARM_Z_POSITIVE_DIR,
                   (unsigned)MECHANICAL_ARM_X_POSITIVE_DIR);
  ArmConsole_ConfigShow(MECHANICAL_ARM_AXIS_BASE);
  ArmConsole_ConfigShow(MECHANICAL_ARM_AXIS_Z);
  ArmConsole_ConfigShow(MECHANICAL_ARM_AXIS_X);
}


/**
  * 函    数：把终端数字转换为摄像头物料颜色枚举
  * 参    数：Text 字符串1～6；Color 输出地址
  * 返 回 值：1转换成功，0参数非法
  * 说    明：颜色1～6依次对应红、黄、蓝、绿、黑和浅蓝
  */
static uint8_t ArmConsole_ColorParse(const char *Text,
                                     Camera_ColorTypeDef *Color)
{
  if ((Text == NULL) || (Color == NULL) || (Text[1] != '\0') ||
      (Text[0] < '1') || (Text[0] > '6'))
  {
    return 0U;
  }
  *Color = (Camera_ColorTypeDef)(Text[0] - '0');
  return 1U;
}

/**
  * 函    数：将车轮缩写转换为麦克纳姆轮枚举
  * 参    数：Text fl、rl、rr或fr；Wheel 输出地址
  * 返 回 值：1转换成功，0名称非法
  * 说    明：轮序从左上角开始逆时针，对应电机ID1至ID4
  */
static uint8_t ArmConsole_WheelParse(const char *Text, MecanumWheel_t *Wheel)
{
  if ((Text == NULL) || (Wheel == NULL))
  {
    return 0U;
  }
  if (strcmp(Text, "fl") == 0)
    *Wheel = MECANUM_WHEEL_FRONT_LEFT;
  else if (strcmp(Text, "rl") == 0)
    *Wheel = MECANUM_WHEEL_REAR_LEFT;
  else if (strcmp(Text, "rr") == 0)
    *Wheel = MECANUM_WHEEL_REAR_RIGHT;
  else if (strcmp(Text, "fr") == 0)
    *Wheel = MECANUM_WHEEL_FRONT_RIGHT;
  else
    return 0U;
  return 1U;
}

/**
  * 函    数：把轴位掩码格式化为逗号分隔名称
  * 参    数：Mask 轴位掩码；Buffer 输出缓冲区；BufferSize 缓冲区长度
  * 返 回 值：无
  * 说    明：位0、1、2分别对应base、z、x
  */
static void ArmConsole_AxisMaskFormat(uint8_t Mask,
                                     char *Buffer,
                                     uint16_t BufferSize)
{
  uint8_t Axis;
  uint8_t First;

  if ((Buffer == NULL) || (BufferSize == 0U))
  {
    return;
  }
  Buffer[0] = '\0';
  First = 1U;
  for (Axis = 0U; Axis < 3U; Axis++)
  {
    if ((Mask & (uint8_t)(1U << Axis)) != 0U)
    {
      if (First == 0U)
      {
        (void)strncat(Buffer, ",", BufferSize - strlen(Buffer) - 1U);
      }
      (void)strncat(Buffer,
                    MechanicalArm_AxisNameGet((MechanicalArm_AxisTypeDef)Axis),
                    BufferSize - strlen(Buffer) - 1U);
      First = 0U;
    }
  }
  if (First != 0U)
  {
    (void)strncpy(Buffer, "none", BufferSize - 1U);
    Buffer[BufferSize - 1U] = '\0';
  }
}

/**
  * 函    数：输出机械臂异步请求结果
  * 参    数：Event 已完成请求的结果
  * 返 回 值：无
  * 说    明：accepted表示驱动器确认收到了位置命令，不表示运动到位
  */
static void ArmConsole_EventShow(const MechanicalArm_EventTypeDef *Event)
{
  char AxisList[20];
  uint8_t Axis;
  uint8_t Flags;

  if (Event == NULL)
  {
    return;
  }

  if (Event->Result == MECHANICAL_ARM_RESULT_OK)
  {
    switch (Event->Action)
    {
      case MECHANICAL_ARM_ACTION_ENABLE:
        ArmConsole_Printf("OK enable %s\r\n", MechanicalArm_AxisNameGet(Event->Axis));
        break;
      case MECHANICAL_ARM_ACTION_DISABLE:
        ArmConsole_Printf("OK disable %s\r\n", MechanicalArm_AxisNameGet(Event->Axis));
        break;
      case MECHANICAL_ARM_ACTION_POSITION:
        ArmConsole_Printf("OK pos %s degree=%.2f pulses=%ld accepted\r\n",
                          MechanicalArm_AxisNameGet(Event->Axis),
                          (double)ArmConsole_PulseToDegree(
                              Event->PositionPulses),
                          (long)Event->PositionPulses);
        break;
      case MECHANICAL_ARM_ACTION_POSITION_READ:
        ArmConsole_Printf("OK position %s raw_units=%ld\r\n",
                          MechanicalArm_AxisNameGet(Event->Axis),
                          (long)Event->CurrentPosition);
        break;
      case MECHANICAL_ARM_ACTION_STATE:
        if (Event->Axis == MECHANICAL_ARM_AXIS_ALL)
        {
          for (Axis = 0U; Axis < 3U; Axis++)
          {
            Flags = Event->StateFlags[Axis];
            ArmConsole_Printf("OK state %s raw=0x%02X en=%u reached=%u\r\n",
                              MechanicalArm_AxisNameGet((MechanicalArm_AxisTypeDef)Axis),
                              (unsigned int)Flags,
                              (unsigned int)(Flags & 0x01U),
                              (unsigned int)((Flags >> 1U) & 0x01U));
          }
        }
        else
        {
          Flags = Event->StateFlags[(uint8_t)Event->Axis];
          ArmConsole_Printf("OK state %s raw=0x%02X en=%u reached=%u\r\n",
                            MechanicalArm_AxisNameGet(Event->Axis),
                            (unsigned int)Flags,
                            (unsigned int)(Flags & 0x01U),
                            (unsigned int)((Flags >> 1U) & 0x01U));
        }
        break;
      case MECHANICAL_ARM_ACTION_ORIGIN_SET:
        ArmConsole_Printf("OK origin %s saved\r\n", MechanicalArm_AxisNameGet(Event->Axis));
        break;
      case MECHANICAL_ARM_ACTION_HOME:
        ArmConsole_Printf("OK home %s mode=%s accepted\r\n",
                          MechanicalArm_AxisNameGet(Event->Axis),
                          ArmConsole_HomeModeNameGet(Event->HomeMode));
        break;
      case MECHANICAL_ARM_ACTION_ZERO:
        ArmConsole_Printf("OK zero %s\r\n", MechanicalArm_AxisNameGet(Event->Axis));
        break;
      case MECHANICAL_ARM_ACTION_STOP:
        ArmConsole_Printf("OK stop %s sent\r\n", MechanicalArm_AxisNameGet(Event->Axis));
        break;
      default:
        ArmConsole_Write("ERR format\r\n");
        break;
    }
    return;
  }

  if ((Event->Action == MECHANICAL_ARM_ACTION_ENABLE) &&
      (Event->Axis == MECHANICAL_ARM_AXIS_ALL))
  {
    ArmConsole_AxisMaskFormat(Event->SuccessMask, AxisList, sizeof(AxisList));
    ArmConsole_Printf("ERR enable_all partial=%s\r\n", AxisList);
    return;
  }
  if ((Event->Action == MECHANICAL_ARM_ACTION_DISABLE) &&
      (Event->Axis == MECHANICAL_ARM_AXIS_ALL))
  {
    ArmConsole_AxisMaskFormat(Event->FailureMask, AxisList, sizeof(AxisList));
    ArmConsole_Printf("ERR disable_all failed=%s\r\n", AxisList);
    return;
  }
  if ((Event->Action == MECHANICAL_ARM_ACTION_ZERO) &&
      (Event->Axis == MECHANICAL_ARM_AXIS_ALL))
  {
    ArmConsole_AxisMaskFormat(Event->FailureMask, AxisList, sizeof(AxisList));
    ArmConsole_Printf("ERR zero_all failed=%s\r\n", AxisList);
    return;
  }
  if ((Event->Action == MECHANICAL_ARM_ACTION_ORIGIN_SET) &&
      (Event->Axis == MECHANICAL_ARM_AXIS_ALL))
  {
    ArmConsole_AxisMaskFormat(Event->FailureMask, AxisList, sizeof(AxisList));
    ArmConsole_Printf("ERR origin_all failed=%s\r\n", AxisList);
    return;
  }
  if ((Event->Action == MECHANICAL_ARM_ACTION_HOME) &&
      (Event->Axis == MECHANICAL_ARM_AXIS_ALL))
  {
    ArmConsole_AxisMaskFormat(Event->FailureMask, AxisList, sizeof(AxisList));
    ArmConsole_Printf("ERR home_all failed=%s\r\n", AxisList);
    return;
  }
  if (Event->Result == MECHANICAL_ARM_RESULT_ACK_TIMEOUT)
  {
    ArmConsole_Printf("ERR ack_timeout axis=%s state_unknown bus_locked=%u\r\n",
                      MechanicalArm_AxisNameGet(Event->FailureAxis),
                      (unsigned int)MotorBus_IsQuarantined());
  }
  else if (Event->Result == MECHANICAL_ARM_RESULT_MOTOR_ERROR)
  {
    ArmConsole_Printf("ERR motor axis=%s code=0x%02X\r\n",
                      MechanicalArm_AxisNameGet(Event->FailureAxis),
                      (unsigned int)Event->MotorCode);
  }
  else
  {
    /* No driver response code exists for a transport failure. */
    ArmConsole_Printf("ERR motor communication axis=%s bus_locked=%u\r\n",
                      MechanicalArm_AxisNameGet(Event->FailureAxis),
                      (unsigned int)MotorBus_IsQuarantined());
  }
  ArmConsole_MotorBusShow();
}

/**
  * 函    数：显示主循环分发的机械臂事件
  * 参    数：Event 已完成请求的结果
  * 返 回 值：无
  * 说    明：MechanicalArm_ResultGet只由main调用一次，控制台不再抢先取走事件
  */
void ArmConsole_MotorEventHandle(const MechanicalArm_EventTypeDef *Event)
{
  ArmConsole_EventShow(Event);
  ArmConsole_PromptShow();
}

/**
  * 函    数：处理机械臂命令接口的立即返回结果
  * 参    数：Result 请求接收结果
  * 返 回 值：1表示已输出错误且应显示提示符，0表示异步请求已启动
  * 说    明：异步请求成功启动时结果为NONE
  */
static uint8_t ArmConsole_RequestResultShow(MechanicalArm_ResultTypeDef Result)
{
  if (Result == MECHANICAL_ARM_RESULT_NONE)
  {
    return 0U;
  }
  if (Result == MECHANICAL_ARM_RESULT_BUSY)
  {
    ArmConsole_Write("ERR busy\r\n");
  }
  else if (Result == MECHANICAL_ARM_RESULT_TX_ERROR)
  {
    ArmConsole_Printf("ERR motor tx bus_locked=%u\r\n",
                      (unsigned int)MotorBus_IsQuarantined());
    ArmConsole_MotorBusShow();
  }
  else
  {
    ArmConsole_Write("ERR format\r\n");
  }
  return 1U;
}

/**
  * 函    数：执行一条已经结束的命令行
  * 参    数：Line 可修改的命令行缓冲区
  * 返 回 值：1表示命令同步结束，0表示已启动异步电机请求
  * 说    明：解析只在主循环运行，中断不调用strtol和字符串函数
  */
static uint8_t ArmConsole_GripCommandHandle(uint8_t TokenCount, char *Tokens[])
{
  float Duty;

  if ((TokenCount == 3U) && (strcmp(Tokens[0], "grip") == 0) &&
      (strcmp(Tokens[1], "duty") == 0))
  {
    if ((ArmConsole_FloatParse(Tokens[2], &Duty) == 0U) ||
        (Duty < 2.5f) || (Duty > 12.5f))
    {
      ArmConsole_Write("ERR range duty\r\n");
    }
    else
    {
      Arm_GripperDutySet(Duty);
      ArmConsole_Printf("OK grip duty=%.2f\r\n", (double)Duty);
    }
    return 1U;
  }
  if ((TokenCount == 2U) && (strcmp(Tokens[0], "grip") == 0))
  {
    if (strcmp(Tokens[1], "idle") == 0)
      Arm_GripperSet(ARM_GRIPPER_IDLE);
    else if (strcmp(Tokens[1], "open") == 0)
      Arm_GripperSet(ARM_GRIPPER_OPEN);
    else if (strcmp(Tokens[1], "catch") == 0)
      Arm_GripperSet(ARM_GRIPPER_CATCH);
    else
    {
      ArmConsole_Write("ERR format\r\n");
      return 1U;
    }
    ArmConsole_Printf("OK grip %s\r\n", Tokens[1]);
    return 1U;
  }

  ArmConsole_Write("ERR format\r\n");
  return 1U;
}

static uint8_t ArmConsole_VisionCommandHandle(uint8_t TokenCount, char *Tokens[])
{
  HAL_StatusTypeDef CameraStatus;
  Camera_ColorTypeDef Color;

  if ((TokenCount == 2U) && (strcmp(Tokens[0], "camera") == 0))
  {
    if (strcmp(Tokens[1], "stop") == 0)
    {
      Camera_RequestStop();
      ArmConsole_CameraTraceStop();
      OledUi_NoticeSet(NULL);
      ArmConsole_Write("OK camera stop\r\n");
    }
    else
    {
      ArmConsole_Write("ERR format\r\n");
    }
    return 1U;
  }
  if ((TokenCount == 3U) && (strcmp(Tokens[0], "camera") == 0) &&
      ((strcmp(Tokens[1], "material") == 0) || (strcmp(Tokens[1], "ring") == 0)))
  {
    if (ArmConsole_ColorParse(Tokens[2], &Color) == 0U ||
        (!strcmp(Tokens[1],"ring") && Color > CAMERA_COLOR_BLUE))
    {
      ArmConsole_Write("ERR camera target\r\n");
      return 1U;
    }
    CameraStatus = !strcmp(Tokens[1],"ring") ? Camera_RingStart((uint8_t)Color) : Camera_MaterialStart(Color);
    if (CameraStatus == HAL_OK)
    {
      ArmConsole_CameraTraceDirectStart();
      if(!strcmp(Tokens[1],"ring"))
        ArmConsole_Printf("OK camera ring ring=%u target=%u sent\r\n",(unsigned int)Color,(unsigned int)(9U+Color));
      else ArmConsole_Printf("OK camera material color=%u sent\r\n",(unsigned int)Color);
    }
    else if (CameraStatus == HAL_BUSY)
    {
      ArmConsole_Write("ERR camera busy\r\n");
    }
    else
    {
      Camera_SnapshotTypeDef Snapshot;
      Camera_SnapshotGet(&Snapshot);
      ArmConsole_Write((Snapshot.UsbConfigured == 0U) ?
                       "ERR camera usb_unconfigured\r\n" :
                       "ERR camera tx\r\n");
    }
    return 1U;
  }

  ArmConsole_Write("ERR format\r\n");
  return 1U;
}

static uint8_t ArmConsole_ChassisCommandHandle(uint8_t TokenCount, char *Tokens[])
{
  float DistanceMm;
  MecanumWheel_t Wheel;

  if ((TokenCount == 2U) && !strcmp(Tokens[0], "chassis") && !strcmp(Tokens[1], "status"))
  {
    ChassisMotion_HoldStatus_t Status;
    ChassisMotion_HoldStatusGet(&Status);
    ArmConsole_Printf("OK chassis hold active=%u reason=%s target_deg=%.3f current_deg=%.3f output_rad_s=%.4f\r\n",
                      (unsigned)Status.active, Status.reason, (double)Status.target_deg,
                      (double)Status.current_deg, (double)Status.output_rad_s);
    return 1U;
  }
  if ((TokenCount == 4U) && !strcmp(Tokens[0], "chassis") && !strcmp(Tokens[1], "hold"))
  {
    int32_t Speed, Seconds;
    ChassisMotion_HoldStatus_t Status;
    if (MechanicalArm_IsBusy())
      ArmConsole_Write("ERR busy\r\n");
    else if (!ArmConsole_IntegerParse(Tokens[2], &Speed) ||
             !ArmConsole_IntegerParse(Tokens[3], &Seconds) ||
             Speed < -100 || Speed > 100 || Seconds < 1 || Seconds > 10)
      ArmConsole_Write("ERR range: chassis hold <-100..100mm/s> <1..10s>\r\n");
    else if (ChassisMotion_HoldStart((float)Speed, (uint32_t)Seconds * 1000U))
      ArmConsole_Printf("OK chassis hold speed=%ldmm/s duration=%lds source=module\r\n", (long)Speed, (long)Seconds);
    else
    {
      ChassisMotion_HoldStatusGet(&Status);
      ArmConsole_Printf("ERR chassis hold reason=%s (stop existing motion first)\r\n", Status.reason);
    }
    return 1U;
  }

  if ((TokenCount == 2U) &&
      (strcmp(Tokens[0], "chassis") == 0) &&
      (strcmp(Tokens[1], "backward") == 0))
  {
    if (MechanicalArm_IsBusy() != 0U)
    {
      ArmConsole_Write("ERR busy\r\n");
      return 1U;
    }
    if ((ChassisMotion_VelocityUpdate(-ARM_CONSOLE_BACKWARD_TEST_SPEED_MM_S,0,0) ||
        ChassisMotion_VelocityStart(-ARM_CONSOLE_BACKWARD_TEST_SPEED_MM_S,0,0)))
    {
      ArmConsole_Write("OK chassis backward velocity=30mm/s request accepted\r\n");
    }
    else
    {
      ArmConsole_Write("ERR motor tx\r\n");
    }
    return 1U;
  }
  if ((TokenCount == 5U) &&
      (strcmp(Tokens[0], "chassis") == 0) &&
      (strcmp(Tokens[1], "velocity") == 0))
  {
    int32_t ForwardMmPerSecond;
    int32_t LeftMmPerSecond;
    int32_t YawDegreesPerSecond;
    if (MechanicalArm_IsBusy() != 0U)
    {
      ArmConsole_Write("ERR busy\r\n");
      return 1U;
    }
    if ((ArmConsole_IntegerParse(Tokens[2], &ForwardMmPerSecond) == 0U) ||
        (ArmConsole_IntegerParse(Tokens[3], &LeftMmPerSecond) == 0U) ||
        (ArmConsole_IntegerParse(Tokens[4], &YawDegreesPerSecond) == 0U))
    {
      ArmConsole_Write("ERR format\r\n");
      return 1U;
    }
    if ((ForwardMmPerSecond < -1000) || (ForwardMmPerSecond > 1000) ||
        (LeftMmPerSecond < -1000) || (LeftMmPerSecond > 1000) ||
        (YawDegreesPerSecond < -360) || (YawDegreesPerSecond > 360))
    {
      ArmConsole_Write("ERR range velocity\r\n");
      return 1U;
    }
    if ((ForwardMmPerSecond == 0) && (LeftMmPerSecond == 0) &&
        (YawDegreesPerSecond == 0))
    {
      (void)ChassisMotion_StopRequest(0);
      ArmConsole_Write("OK chassis stop request accepted; physical stop unconfirmed\r\n");
      return 1U;
    }
    if ((ChassisMotion_VelocityUpdate((float)ForwardMmPerSecond,(float)LeftMmPerSecond,
         (float)YawDegreesPerSecond*3.14159265358979323846f/180.0f) ||
        ChassisMotion_VelocityStart((float)ForwardMmPerSecond,(float)LeftMmPerSecond,
         (float)YawDegreesPerSecond*3.14159265358979323846f/180.0f)))
    {
      ArmConsole_Printf("OK chassis velocity forward=%ld left=%ld yaw=%lddeg/s request accepted\r\n",
                        (long)ForwardMmPerSecond,
                        (long)LeftMmPerSecond,
                        (long)YawDegreesPerSecond);
    }
    else
    {
      ArmConsole_Write("ERR motor tx\r\n");
    }
    return 1U;
  }
  if ((TokenCount == 3U) && (strcmp(Tokens[0], "chassis") == 0))
  {
    int32_t DistanceMagnitudeMm;
    if (MechanicalArm_IsBusy() != 0U)
    {
      ArmConsole_Write("ERR busy\r\n");
      return 1U;
    }
    if ((ArmConsole_IntegerParse(Tokens[2], &DistanceMagnitudeMm) == 0U) ||
        (DistanceMagnitudeMm < 1) || (DistanceMagnitudeMm > 200))
    {
      ArmConsole_Write("ERR range distance\r\n");
      return 1U;
    }
    if (strcmp(Tokens[1], "forward") == 0)
    {
      DistanceMm = (float)DistanceMagnitudeMm;
    }
    else if (strcmp(Tokens[1], "backward") == 0)
    {
      DistanceMm = -(float)DistanceMagnitudeMm;
    }
    else
    {
      ArmConsole_Write("ERR format\r\n");
      return 1U;
    }
    if (Mecanum_Polarity_Move(DistanceMm))
      ArmConsole_Printf("OK chassis %s %ldmm request accepted\r\n",
                        Tokens[1], (long)DistanceMagnitudeMm);
    else
      ArmConsole_Write("ERR motor tx\r\n");
    return 1U;
  }
  if ((TokenCount == 3U) && (strcmp(Tokens[0], "wheel") == 0))
  {
    int32_t WheelRpm;
    if (MechanicalArm_IsBusy() != 0U)
    {
      ArmConsole_Write("ERR busy\r\n");
      return 1U;
    }
    if ((ArmConsole_WheelParse(Tokens[1], &Wheel) == 0U) ||
        (ArmConsole_IntegerParse(Tokens[2], &WheelRpm) == 0U))
    {
      ArmConsole_Write("ERR format\r\n");
      return 1U;
    }
    if ((WheelRpm == 0) || (WheelRpm < -100) || (WheelRpm > 100))
    {
      ArmConsole_Write("ERR range rpm\r\n");
      return 1U;
    }
    if (Mecanum_Wheel_Test(Wheel, (int16_t)WheelRpm))
      ArmConsole_Printf("OK wheel %s %ldrpm request accepted\r\n",
                        Tokens[1], (long)WheelRpm);
    else
      ArmConsole_Write("ERR motor tx\r\n");
    return 1U;
  }

  ArmConsole_Write("ERR format\r\n");
  return 1U;
}

static uint8_t ArmConsole_MechanicalArmCommandHandle(uint8_t TokenCount, char *Tokens[])
{
  MechanicalArm_AxisTypeDef Axis;
  MechanicalArm_ResultTypeDef Result;
  uint8_t HomeMode, Index;
  float Degree;

  if (TokenCount < 2U)
  {
    ArmConsole_Write("ERR format\r\n");
    return 1U;
  }
  Axis = MechanicalArm_AxisGet(Tokens[1]);
  if (Axis == MECHANICAL_ARM_AXIS_INVALID)
  {
    ArmConsole_Write("ERR axis\r\n");
    return 1U;
  }

  if ((TokenCount == 2U) && (strcmp(Tokens[0], "enable") == 0))
  {
    return ArmConsole_RequestResultShow(MechanicalArm_Enable(Axis));
  }
  if ((TokenCount == 2U) && (strcmp(Tokens[0], "disable") == 0))
  {
    return ArmConsole_RequestResultShow(MechanicalArm_Disable(Axis));
  }
  if ((TokenCount == 2U) && (strcmp(Tokens[0], "state") == 0))
  {
    return ArmConsole_RequestResultShow(MechanicalArm_StateRead(Axis));
  }
  if ((TokenCount == 2U) && (strcmp(Tokens[0], "position") == 0))
  {
    if (Axis == MECHANICAL_ARM_AXIS_ALL)
    {
      ArmConsole_Write("ERR axis\r\n");
      return 1U;
    }
    return ArmConsole_RequestResultShow(MechanicalArm_PositionRead(Axis));
  }
  if ((TokenCount == 2U) && (strcmp(Tokens[0], "origin") == 0))
  {
    return ArmConsole_RequestResultShow(MechanicalArm_OriginSet(Axis));
  }
  if ((TokenCount == 3U) && (strcmp(Tokens[0], "home") == 0))
  {
    if (ArmConsole_HomeModeParse(Tokens[2], &HomeMode) == 0U)
    {
      ArmConsole_Write("ERR mode\r\n");
      return 1U;
    }
    return ArmConsole_RequestResultShow(MechanicalArm_Home(Axis, HomeMode));
  }
  if ((TokenCount == 2U) && (strcmp(Tokens[0], "zero") == 0))
  {
    return ArmConsole_RequestResultShow(MechanicalArm_Zero(Axis));
  }
  if ((TokenCount == 2U) && (strcmp(Tokens[0], "stop") == 0))
  {
    return ArmConsole_RequestResultShow(MechanicalArm_Stop(Axis));
  }
  if ((TokenCount == 3U) && (strcmp(Tokens[0], "pos") == 0))
  {
    int32_t PositionPulses;
    if (Axis == MECHANICAL_ARM_AXIS_ALL)
    {
      ArmConsole_Write("ERR axis\r\n");
      return 1U;
    }
    if ((ArmConsole_FloatParse(Tokens[2], &Degree) == 0U) ||
        (ArmConsole_DegreeToPulse(Axis, Degree, &PositionPulses) == 0U))
    {
      ArmConsole_Write("ERR range degree\r\n");
      return 1U;
    }
    Result = MechanicalArm_Position(Axis, PositionPulses);
    if (Result == MECHANICAL_ARM_RESULT_PARAM_ERROR)
    {
      ArmConsole_Write("ERR range degree\r\n");
      return 1U;
    }
    return ArmConsole_RequestResultShow(Result);
  }
  if ((strcmp(Tokens[0], "config") == 0) && (TokenCount == 2U))
  {
    if (Axis == MECHANICAL_ARM_AXIS_ALL)
    {
      for (Index = 0U; Index < 3U; Index++)
      {
        ArmConsole_ConfigShow((MechanicalArm_AxisTypeDef)Index);
      }
    }
    else
    {
      ArmConsole_ConfigShow(Axis);
    }
    return 1U;
  }
  if ((strcmp(Tokens[0], "config") == 0) && (TokenCount == 5U))
  {
    int32_t SpeedRpm;
    int32_t Acceleration;
    int32_t PulseLimit;
    if ((ArmConsole_IntegerParse(Tokens[2], &SpeedRpm) == 0U) ||
        (ArmConsole_IntegerParse(Tokens[3], &Acceleration) == 0U) ||
        (ArmConsole_IntegerParse(Tokens[4], &PulseLimit) == 0U))
    {
      ArmConsole_Write("ERR format\r\n");
      return 1U;
    }
    if ((SpeedRpm < 1) || (SpeedRpm > 3000))
    {
      ArmConsole_Write("ERR range speed\r\n");
      return 1U;
    }
    if ((Acceleration < 1) || (Acceleration > 255))
    {
      ArmConsole_Write("ERR range accel\r\n");
      return 1U;
    }
    if ((PulseLimit < 0) ||
        (PulseLimit > (int32_t)MECHANICAL_ARM_COMMAND_PULSES_PER_REV) ||
        ((PulseLimit == 0) && (Axis != MECHANICAL_ARM_AXIS_Z)))
    {
      ArmConsole_Write("ERR range limit\r\n");
      return 1U;
    }
    (void)MechanicalArm_ConfigSet(Axis,
                                  (uint16_t)SpeedRpm,
                                  (uint8_t)Acceleration,
                                  (uint32_t)PulseLimit);
    if (Axis == MECHANICAL_ARM_AXIS_ALL)
    {
      ArmConsole_Printf("OK config all rpm=%ld acc=%ld limit_pulses=%ld\r\n",
                        (long)SpeedRpm, (long)Acceleration, (long)PulseLimit);
    }
    else
    {
      ArmConsole_ConfigShow(Axis);
    }
    return 1U;
  }

  ArmConsole_Write("ERR format\r\n");
  return 1U;
}

static uint8_t ArmConsole_StatusCommandHandle(uint8_t TokenCount, char *Tokens[])
{
  if ((TokenCount == 3U) && !strcmp(Tokens[0], "imu") && !strcmp(Tokens[1], "stream"))
  {
    if (!strcmp(Tokens[2], "on"))
    {
      if (HWT101_Cal_IsBusy())
      {
        ArmConsole_Write("ERR imu busy stream remains off\r\n");
        return 1U;
      }
      ArmConsole_ImuStream = 1U;
    }
    else if (!strcmp(Tokens[2], "off")) { ArmConsole_ImuStream = 0U; ConsoleTx_DebugCancel(CONSOLE_DEBUG_IMU); }
    else return 0U;
    ArmConsole_Printf("OK imu stream=%s\r\n", ArmConsole_ImuStream ? "on" : "off");
    return 1U;
  }
  if ((!strcmp(Tokens[0], "imu")) &&
      (((TokenCount == 3U) && !strcmp(Tokens[1], "cal") && !strcmp(Tokens[2], "start")) ||
       ((TokenCount == 2U) && !strcmp(Tokens[1], "zero")) ||
       ((TokenCount == 2U) && !strcmp(Tokens[1], "verify")) ||
       ((TokenCount == 3U) && !strcmp(Tokens[1], "verify") && !strcmp(Tokens[2], "5"))))
  {
    HWT101_CalStatus_t Cal;
    uint8_t Native = !strcmp(Tokens[1], "cal");
    uint8_t ZeroOnly = !strcmp(Tokens[1], "zero");
    uint32_t VerifyMs = Native ? 30000U : ZeroOnly ? 0U : 5000U;
    bool Started;
    if (HWT101_Cal_IsBusy())
    {
      ArmConsole_Write("ERR imu cal busy\r\n");
      return 1U;
    }
    if (Mecanum_IsBusy() || MechanicalArm_IsBusy())
    {
      ArmConsole_Write("ERR imu motion_busy stop all motion first\r\n");
      return 1U;
    }
    Started = Native ? HWT101_Cal_Start() : ZeroOnly ? HWT101_Cal_ZeroStart() : HWT101_Cal_VerifyStart(VerifyMs);
    if (Started)
    {
      ArmConsole_ImuStream = 0U;
      ConsoleTx_DebugCancel(CONSOLE_DEBUG_IMU);
      ArmConsole_CameraTraceStop();
      ArmConsole_Printf("OK imu %s started cal_ms=%lu verify_ms=%lu keep_still=1 stream=off\r\n",
                        Native ? "cal" : ZeroOnly ? "zero" : "verify", Native ? 20000UL : 0UL, (unsigned long)VerifyMs);
    }
    else if (HWT101_Cal_GetStatus(&Cal))
      ArmConsole_Printf("ERR imu cal reason=%s HAL=%u\r\n", Cal.reason, (unsigned)Cal.hal);
    return 1U;
  }
  if ((TokenCount == 3U) && !strcmp(Tokens[0], "imu") && !strcmp(Tokens[1], "cal"))
  {
    if (!strcmp(Tokens[2], "cancel") || !strcmp(Tokens[2], "clear"))
    {
      ChassisMotion_HoldStatus_t Heading;
      ChassisMotion_HoldStatusGet(&Heading);
      if (Heading.active && !ChassisMotion_StopRequest(0)) ArmConsole_Write("ERR chassis stop_tx_error\r\n");
      HWT101_Cal_Cancel();
      ArmConsole_Write("OK imu cal cancel requested boot_verification_cleared=1 module_bias_kept=1 normal_mode=pending\r\n");
      return 1U;
    }
    if (!strcmp(Tokens[2], "forget"))
    {
      ArmConsole_Write("ERR imu cal forget deprecated no_erase=1\r\n");
      return 1U;
    }
  }
  if ((TokenCount == 2U) && (strcmp(Tokens[0], "camera") == 0) &&
      (strcmp(Tokens[1], "status") == 0))
  {
    Camera_SnapshotTypeDef Snapshot;
    Camera_VisualStateTypeDef State;
    uint32_t FrameAgeMs;
    Camera_SnapshotGet(&Snapshot);
    State = Camera_VisualStateGet(&Snapshot);
    FrameAgeMs = (Snapshot.HasFrame != 0U) ?
                 (HAL_GetTick() - Snapshot.LastFrameTick) : 0U;
    ArmConsole_Printf("OK camera state=%s protocol=B2 age_source=rx usb=%u active=%u fn=%u target=%u "
                      "frame=%u current_valid=%u target_valid=%u\r\n",
                      Camera_VisualStateNameGet(State),
                      (unsigned int)Snapshot.UsbConfigured,
                      (unsigned int)Snapshot.RequestActive,
                      (unsigned int)Snapshot.RequestFunction,
                      (unsigned int)Snapshot.RequestTarget,
                      (unsigned int)Snapshot.HasFrame,
                      (unsigned int)Snapshot.HasValidData,
                      (unsigned int)Snapshot.TargetValid);
    if (State == CAMERA_VIS_OK)
    {
      ArmConsole_Printf("OK camera last fn=%u target=%u cx=%u cy=%u dx=%d dy=%d "
                      "seq=%lu data_tick=%lu\r\n",
                      (unsigned int)Snapshot.Data.Function,
                      (unsigned int)Snapshot.Data.Target,
                      (unsigned int)Snapshot.Data.CX,
                      (unsigned int)Snapshot.Data.CY,
                      (int)Snapshot.Data.DX,
                      (int)Snapshot.Data.DY,
                      (unsigned long)Snapshot.Data.Sequence,
                      (unsigned long)Snapshot.Data.Tick);
    }
    else
    {
      ArmConsole_Printf("OK camera last cx=na cy=na dx=na dy=na seq=na data_tick=na\r\n");
    }
    ArmConsole_Printf("OK camera stats last_frame_ms=%lu rx_bytes=%lu "
                      "checksum_errors=%lu overflows=%lu frame_age_ms=%lu invalid_reports=%lu\r\n",
                      (unsigned long)Snapshot.LastFrameTick,
                      (unsigned long)Snapshot.RxByteCount,
                      (unsigned long)Snapshot.ChecksumErrorCount,
                      (unsigned long)Snapshot.OverflowCount,
                      (unsigned long)FrameAgeMs,
                      (unsigned long)Snapshot.InvalidCount);
    return 1U;
  }
  if ((TokenCount == 2U) && (strcmp(Tokens[0], "imu") == 0) &&
      (strcmp(Tokens[1], "status") == 0))
  {
    HWT101_Angle_t Angle = {0};
    HWT101_Status_t Status = {0};
    HWT101_CalStatus_t Cal;
    uint8_t HasAngle = HWT101_Angle_Get(&Angle) ? 1U : 0U;
    (void)HWT101_Status_Get(&Status);
    ArmConsole_Printf("OK imu ready=%u valid=%u fresh=%u yaw_deg=%.2f "
                      "updates=%lu last_ms=%lu reads=%lu i2c_errors=%lu\r\n",
                      (unsigned int)(HWT101_Is_Ready() ? 1U : 0U),
                      (unsigned int)HasAngle,
                      (unsigned int)((HasAngle != 0U) &&
                          HWT101_Angle_Is_Fresh(&Angle, HWT101_DATA_FRESH_MS)),
                      (double)Angle.yaw,
                      (unsigned long)Angle.update_count,
                      (unsigned long)Status.last_update_ms,
                      (unsigned long)Status.valid_read_count,
                      (unsigned long)Status.i2c_error_count);
    if (!HWT101_Cal_IsBusy()) (void)HWT101_Cal_RefreshRegisters();
    if (!HWT101_Cal_GetStatus(&Cal)) return 1U;
    ArmConsole_Printf("OK imu stream=%s result=%s save_state=%s persistent=UNTESTED\r\n",
                      ArmConsole_ImuStream ? "on" : "off", Cal.result, Cal.save_state);
    ArmConsole_Printf("OK imu cal_state=%s reason=%s HAL=%u run_id=%lu busy=%u verified=%u control_ready=%u normal_mode=%u\r\n",
                      Cal.state_name, Cal.reason, (unsigned)Cal.hal, (unsigned long)Cal.run_id,
                      (unsigned)Cal.busy, (unsigned)Cal.verified, (unsigned)Cal.control_ready, (unsigned)Cal.normal_mode_confirmed);
    ArmConsole_Printf("OK imu registers_valid=%u version=0x%04X mode=0x%04X bias_before=0x%04X bias_after=0x%04X bias_after_valid=%u\r\n",
                      (unsigned)Cal.registers_valid, (unsigned)Cal.version, (unsigned)Cal.mode,
                      (unsigned)Cal.bias_before, (unsigned)Cal.bias_after, (unsigned)Cal.bias_after_valid);
    ArmConsole_Printf("OK imu elapsed_ms=%lu total_ms=%lu verify_ms=%lu verify_elapsed_ms=%lu samples=%lu\r\n",
                      (unsigned long)Cal.elapsed_ms, (unsigned long)Cal.total_ms, (unsigned long)Cal.verify_ms,
                      (unsigned long)Cal.verify_elapsed_ms, (unsigned long)Cal.samples);
    ArmConsole_Printf("OK imu zero_requested=%u zero_sample_received=%u zero_before_deg=%.3f zero_after_deg=%.3f\r\n",
                      (unsigned)Cal.zero_requested, (unsigned)Cal.zero_sample_received,
                      (double)Cal.zero_before_deg, (double)Cal.zero_after_deg);
    ArmConsole_Printf("OK imu gap_ms=%lu max_gap_ms=%lu limit_ms=%lu i2c_errors=%lu\r\n",
                      (unsigned long)Cal.gap_ms, (unsigned long)Cal.max_gap_ms,
                      (unsigned long)HWT101_DATA_FRESH_MS, (unsigned long)Cal.i2c_errors);
    ArmConsole_Printf("OK imu relative_deg=%.3f drift_dps=%.5f rms_deg=%.4f save_requested=%u save_readback_ok=%u\r\n",
                      (double)Cal.relative_deg, (double)Cal.drift_dps, (double)Cal.rms_deg,
                      (unsigned)Cal.save_requested, (unsigned)Cal.save_readback_ok);
    return 1U;
  }
  if ((TokenCount == 2U) && (strcmp(Tokens[0], "screen") == 0) &&
      (strcmp(Tokens[1], "status") == 0))
  {
    TJC_Status_t Status = {0};
    (void)TJC_Status_Get(&Status);
    ArmConsole_Printf("OK screen tx_busy=%u tx=%lu rx_frames=%lu "
                      "rx_overflows=%lu uart_errors=%lu\r\n",
                      (unsigned int)(Status.tx_busy ? 1U : 0U),
                      (unsigned long)Status.tx_count,
                      (unsigned long)Status.rx_frame_count,
                      (unsigned long)Status.rx_overflow_count,
                      (unsigned long)Status.uart_error_count);
    return 1U;
  }

  return 0U;
}

/* Main-loop only. The QR consumer in main.c retains its independent ready flag. */
static uint8_t ArmConsole_QrCommandHandle(uint8_t TokenCount, char *Tokens[])
{
  QR_SnapshotTypeDef Snapshot;
  char Age[11];
  QR_SnapshotGet(&Snapshot);
  if (Snapshot.Valid)
    (void)snprintf(Age, sizeof(Age), "%lu", (unsigned long)(uint32_t)(HAL_GetTick() - Snapshot.ReceivedTick));
  else
    (void)strcpy(Age, "NA");

  if ((TokenCount == 2U) && !strcmp(Tokens[1], "status"))
  {
    ArmConsole_Printf("OK qr status valid=%u seq=%lu age_ms=%s received=%lu accepted=%lu rejected=%lu uart_errors=%lu stream=%s event_dropped=%lu source=%s\r\n",
      (unsigned)Snapshot.Valid, (unsigned long)Snapshot.Sequence, Age,
      (unsigned long)Snapshot.Received, (unsigned long)Snapshot.Accepted,
      (unsigned long)Snapshot.Rejected, (unsigned long)Snapshot.UartErrors,
      ArmConsole_QrStream ? "on" : "off", (unsigned long)(ArmConsole_QrSkipped + ConsoleTx_EventDropped()),
      Snapshot.Source==QR_SOURCE_SIMULATED?"simulated":Snapshot.Source==QR_SOURCE_UART?"uart":"none");
  }
  else if ((TokenCount == 2U) && !strcmp(Tokens[1], "read"))
  {
    ArmConsole_Printf("OK qr read valid=%u seq=%lu code=%s age_ms=%s source=%s\r\n",
      (unsigned)Snapshot.Valid, (unsigned long)Snapshot.Sequence,
      Snapshot.Valid ? Snapshot.Code : "NA", Age,
      Snapshot.Source==QR_SOURCE_SIMULATED?"simulated":Snapshot.Source==QR_SOURCE_UART?"uart":"none");
  }
  else if ((TokenCount == 3U) && !strcmp(Tokens[1], "stream") &&
           (!strcmp(Tokens[2], "on") || !strcmp(Tokens[2], "off")))
  {
    if (!strcmp(Tokens[2], "on"))
    {
      if (!ArmConsole_QrStream)
      {
        /* A follow request observes future codes; use qr read for the cache. */
        ArmConsole_QrSeenSequence = Snapshot.Sequence;
        ArmConsole_QrLastEventTick = HAL_GetTick();
        ArmConsole_QrPending.Valid = 0U;
      }
      ArmConsole_QrStream = 1U;
    }
    else
    {
      if (ArmConsole_QrPending.Valid) ArmConsole_QrSkipped++;
      ArmConsole_QrPending.Valid = 0U;
      ArmConsole_QrStream = 0U;
      ConsoleTx_EventCancel();
    }
    ArmConsole_Printf("OK qr stream=%s\r\n", ArmConsole_QrStream ? "on" : "off");
  }
  else
    ArmConsole_Write("ERR qr format: qr status | qr read | qr stream on|off\r\n");
  return 1U;
}

static void ArmConsole_QrEventProcess(void)
{
  QR_SnapshotTypeDef Snapshot;
  uint32_t Now;
  char Text[112];
  int Length;
  if (!ArmConsole_QrStream) return;
  QR_SnapshotGet(&Snapshot);
  Now = HAL_GetTick();
  if (Snapshot.Valid && (Snapshot.Sequence != ArmConsole_QrSeenSequence))
  {
    ArmConsole_QrSkipped += (uint32_t)(Snapshot.Sequence - ArmConsole_QrSeenSequence) - 1U;
    if (ArmConsole_QrPending.Valid) ArmConsole_QrSkipped++;
    ArmConsole_QrSeenSequence = Snapshot.Sequence;
    ArmConsole_QrPending = Snapshot;
  }
  if (!ArmConsole_QrPending.Valid || ((uint32_t)(Now - ArmConsole_QrLastEventTick) < 200U)) return;
  Length = snprintf(Text, sizeof(Text), "\r\nEVT qr code seq=%lu code=%s age_ms=%lu source=%s\r\n",
    (unsigned long)ArmConsole_QrPending.Sequence, ArmConsole_QrPending.Code,
    (unsigned long)(uint32_t)(Now - ArmConsole_QrPending.ReceivedTick),
    ArmConsole_QrPending.Source==QR_SOURCE_SIMULATED?"simulated":"uart");
  if (Length > 0 && (size_t)Length < sizeof(Text))
    (void)ConsoleTx_Event(Text, (uint16_t)Length);
  else
    ArmConsole_QrSkipped++;
  ArmConsole_QrPending.Valid = 0U;
  ArmConsole_QrLastEventTick = Now;
}

static ChassisStop_Status_t StopReported;

static void ArmConsole_StopStatusShow(void)
{
  char text[192];
  int length;
  ChassisStop_Status_t status;
  ChassisMotion_StopStatusGet(&status);
  length = snprintf(text, sizeof(text), "\r\nOK chassis stop id=%lu token=%lu requested_ms=%lu tx_complete=%u wheels_stopped=%u reason=%s\r\n",
    (unsigned long)status.id, (unsigned long)status.token, (unsigned long)status.requested_ms,
    status.tx_complete ? 1U : 0U, status.wheels_stopped ? 1U : 0U, status.reason);
  if (length > 0 && length < (int)sizeof(text))
    if (ConsoleTx_Urgent(text, (uint16_t)length)) StopReported = status;
}

static void ArmConsole_StopStatusProcess(void)
{
  GrabTask_Status_t grab;
  ChassisMotion_Status_t motion;
  ChassisStop_Status_t status;
  ChassisMotion_StopStatusGet(&status);
  if (status.id && (status.id != StopReported.id || status.token != StopReported.token ||
      status.tx_complete != StopReported.tx_complete || status.wheels_stopped != StopReported.wheels_stopped ||
      !StopReported.reason || strcmp(status.reason, StopReported.reason)))
    ArmConsole_StopStatusShow();
  GrabTask_StatusGet(&grab);
  ChassisMotion_StatusGet(&motion);
  ConsoleTx_BackgroundPause(ChassisMotion_StopPending() || grab.state == GRAB_STOPPING ||
                            motion.state == CHASSIS_MOTION_STOPPING);
}

static void ArmConsole_ResetCancel(void)
{
  if (ResetPending) ArmConsole_Write("ERR system reset cancelled_by_stop\r\n");
  ResetPending = 0U;
}

static void ArmConsole_StopRequest(uint32_t token)
{
  ArmConsole_BusTestStop("stopped");
  ArmConsole_ManualGuardCancel();
  BusRecoveryPending=0U;
  MotorBus_RecoveryCancel();
  ArmConsole_ResetCancel();
  /* Cancel every automatic producer before requesting the shared UART5 stop. */
  GrabRoute_Stop();
  Turntable_Stop();
  (void)ChassisRoute_Cancel();
  OledUi_NoticeSet(NULL);
  (void)ChassisMotion_StopRequest(token);
  ArmConsole_StopStatusShow();
}

static void ArmConsole_BusTestShow(const char *action)
{
  bool failed=BusTest.timeouts || BusTest.uart_errors || BusTest.rejected ||
    (BusTest.reason && !strcmp(BusTest.reason,"line_busy"));
  ArmConsole_Printf("%s bus test %s id=%u rounds=%u active=%u attempts=%lu sent=%lu received=%lu "
    "timeouts=%lu uart_errors=%lu rejected=%lu max_reply_ms=%lu last_uart_error=0x%08lX locked=%u reason=%s\r\n",
    failed?"ERR":"OK",action,(unsigned)BusTest.address,(unsigned)BusTest.rounds,(unsigned)BusTest.active,
    (unsigned long)BusTest.attempted,(unsigned long)BusTest.sent,(unsigned long)BusTest.received,
    (unsigned long)BusTest.timeouts,(unsigned long)BusTest.uart_errors,(unsigned long)BusTest.rejected,
    (unsigned long)BusTest.reply_max,(unsigned long)BusTest.last_uart_error,
    (unsigned)MotorBus_IsQuarantined(),BusTest.reason?BusTest.reason:"idle");
}
static void ArmConsole_BusTestStop(const char *reason)
{
  if(!BusTest.active) return;
  BusTest.active=false;BusTest.pending=false;BusTest.reason=reason;
  BusTest.elapsed=HAL_GetTick()-BusTest.started;
  MotorBus_ProbeStop();
  ArmConsole_BusTestShow("complete");
}
static void ArmConsole_BusTestProcess(void)
{
  MotorBus_Event_t e;
  static const uint8_t functions[]={0x3a,0x36,0x35};
  static const uint8_t lengths[]={4,8,6};
  uint32_t now=HAL_GetTick();
  if(!BusTest.active) { (void)MotorBus_EventGet(MOTOR_BUS_PROBE,&e);return; }
  if(MotorBus_EventGet(MOTOR_BUS_PROBE,&e) && BusTest.pending && e.token==BusTest.token) {
    uint32_t round=(BusTest.attempted-1U)/3U+1U;
    uint8_t function=functions[(BusTest.attempted-1U)%3U];
    BusTest.pending=false;BusTest.wait_since=now;BusTest.next_tick=now+20U;
    if(e.tx_completed) BusTest.sent++;
    if(e.result==MOTOR_BUS_CANCELLED) { ArmConsole_BusTestStop("stopped");return; }
    if(e.result==MOTOR_BUS_REPLY) {
      char hex[3U*MOTOR_BUS_FRAME_MAX+1U];
      uint32_t delay=e.tick_ms-e.tx_ms;
      for(unsigned i=0;i<e.length;i++)
        (void)snprintf(hex+3U*i,sizeof(hex)-3U*i,"%02X%s",e.data[i],i+1U<e.length?":":"");
      if(delay>BusTest.reply_max) BusTest.reply_max=delay;
      if(e.length==4U && (e.data[2]==0xe2 || e.data[2]==0xee)) BusTest.rejected++;
      else BusTest.received++;
      ArmConsole_Printf("%s bus test reply id=%u round=%lu func=0x%02X reply_ms=%lu hex=%s\r\n",
        e.length==4U && (e.data[2]==0xe2 || e.data[2]==0xee)?"ERR":"OK",
        (unsigned)BusTest.address,(unsigned long)round,(unsigned)function,(unsigned long)delay,hex);
    } else {
      if(e.result==MOTOR_BUS_TIMEOUT) BusTest.timeouts++;
      else { BusTest.uart_errors++;BusTest.last_uart_error=e.uart_error; }
      ArmConsole_Printf("ERR bus test read id=%u round=%lu func=0x%02X reason=%s uart_error=0x%08lX\r\n",
        (unsigned)BusTest.address,(unsigned long)round,(unsigned)function,
        e.result==MOTOR_BUS_TIMEOUT?(e.tx_completed?"reply_timeout":"tx_timeout"):
          (e.uart_error?"uart_error":"transport_error"),(unsigned long)e.uart_error);
    }
    if(BusTest.attempted==3U*BusTest.rounds) { ArmConsole_BusTestStop("done");return; }
  }
  if(BusTest.pending) return;
  if(!MotorBus_ProbeReady()) {
    if((uint32_t)(now-BusTest.wait_since)>1000U) ArmConsole_BusTestStop("line_busy");
    return;
  }
  if((int32_t)(now-BusTest.next_tick)<0) return;
  {
    uint8_t kind=(uint8_t)(BusTest.attempted%3U);
    uint8_t cmd[]={BusTest.address,functions[kind],0x6b};
    BusTest.token=++BusTestToken;
    if(MotorBus_Submit(MOTOR_BUS_PROBE,cmd,sizeof cmd,lengths[kind],BusTest.token,false)) {
      BusTest.pending=true;BusTest.attempted++;
    } else if((uint32_t)(now-BusTest.wait_since)>1000U) ArmConsole_BusTestStop("line_busy");
  }
}
static uint8_t ArmConsole_BusTestCommand(unsigned count,char *tokens[])
{
  int32_t id,rounds=10;
  Mecanum_Status_t status;
  if(count==3U && !strcmp(tokens[2],"status")) { ArmConsole_BusTestShow("status");return 1U; }
  if(count==3U && !strcmp(tokens[2],"stop")) {
    if(BusTest.active) ArmConsole_BusTestStop("stopped");else ArmConsole_BusTestShow("status");
    return 1U;
  }
  if((count!=3U && count!=4U) || !ArmConsole_IntegerParse(tokens[2],&id) || id<1 || id>8 ||
     (count==4U && (!ArmConsole_IntegerParse(tokens[3],&rounds) || rounds<1 || rounds>100))) {
    ArmConsole_Write("ERR format: bus test <id 1..8> [rounds 1..100] | bus test status | bus test stop\r\n");return 1U;
  }
  if(BusTest.active) { ArmConsole_Write("ERR bus test active; use bus test stop\r\n");return 1U; }
  Mecanum_StatusGet(&status);
  if(BusRecoveryPending || ManualGuardActive || GrabTask_IsBusy() || GrabRoute_IsBusy() ||
     Turntable_IsBusy() || MechanicalArm_IsBusy() || ChassisRoute_IsBusy() || ChassisMotion_IsBusy() ||
     status.stop_pending || (!status.locked && Mecanum_IsBusy())) {
    ArmConsole_Write("ERR bus test tasks_busy; stop active task before testing\r\n");return 1U;
  }
  if(!MotorBus_ProbeStart((uint8_t)id)) {
    ArmConsole_Write("ERR bus test transport_busy; wait for current transaction\r\n");return 1U;
  }
  memset(&BusTest,0,sizeof BusTest);
  BusTest.active=true;BusTest.address=(uint8_t)id;BusTest.rounds=(uint16_t)rounds;
  BusTest.started=BusTest.wait_since=BusTest.next_tick=HAL_GetTick();BusTest.reason="testing";
  ArmConsole_Printf("OK bus test start id=%u rounds=%u locked=%u read_only=1\r\n",
    (unsigned)id,(unsigned)rounds,(unsigned)MotorBus_IsQuarantined());
  return 1U;
}

static void ArmConsole_BusStatusShow(void)
{
  MotorBus_Recovery_t s;
  MotorBus_Timing_t timing;
  MotorBus_RecoveryGet(&s);
  ArmConsole_Printf("OK bus recovery active=%u success=%u reason=%s addr=%u checked=0x%02X locked=%u speed_addr=%u speed_rpm=%ld speed_ms=%lu\r\n",
    (unsigned)(BusRecoveryPending!=0 || s.active),(unsigned)s.success,
    BusRecoveryPending==1?"stopping_tasks":s.reason,(unsigned)s.address,
    (unsigned)s.checked_mask,(unsigned)MotorBus_IsQuarantined(),
    (unsigned)s.speed_address,(long)s.speed_rpm,(unsigned long)s.speed_ms);
  Mecanum_Status_t chassis; Mecanum_StatusGet(&chassis);
  ArmConsole_Printf("OK bus motion locked=%u error=%u feedback_allowed=%u\r\n",
    (unsigned)chassis.locked,(unsigned)chassis.error,(unsigned)MotorBus_FeedbackAllowed());
  MotorBus_TimingGet(&timing);
  ArmConsole_Printf("OK bus timing transactions=%lu ordinary=%lu emergency=%lu drained=%lu clock_ms=1\r\n",
    (unsigned long)timing.transactions,(unsigned long)timing.ordinary,
    (unsigned long)timing.emergency,(unsigned long)timing.drained);
  for(unsigned i=0;i<8;i++) {
    const MotorBus_IdTiming_t *t=&timing.id[i];
    ArmConsole_Printf("OK bus id=%u requests=%lu success=%lu rejected=%lu timeouts=%lu tx_ms=%lu reply_ms=%lu reply_last_ms=%lu reply_max_ms=%lu queue_last_ms=%lu queue_max_ms=%lu\r\n",
      i+1,(unsigned long)t->requests,(unsigned long)t->success,(unsigned long)t->rejected,
      (unsigned long)t->timeouts,(unsigned long)t->tx_ms,(unsigned long)t->reply_ms,
      (unsigned long)t->reply_last_ms,(unsigned long)t->reply_max_ms,
      (unsigned long)t->queue_last_ms,(unsigned long)t->queue_max_ms);
  }
}
static void ArmConsole_BusRecoveryProcess(void)
{
  MotorBus_Recovery_t s;
  if (!BusRecoveryPending) return;
  if (BusRecoveryPending==1) {
    if ((uint32_t)(HAL_GetTick()-BusRecoveryStarted)>10000U) {
      BusRecoveryPending=0;
      ArmConsole_Write("ERR bus recover tasks_not_idle; inspect grab status and info\r\n");return;
    }
    if (GrabTask_IsBusy() || GrabRoute_IsBusy() || Turntable_IsBusy() || MechanicalArm_IsBusy() ||
        ChassisRoute_IsBusy() || ChassisMotion_IsBusy()) return;
    /* A stopping task may have re-enabled its feedback before exiting. */
    Mecanum_Feedback_Enable(false);
    if (!MotorBus_RecoveryStart()) return;
    BusRecoveryPending=2;
  }
  MotorBus_RecoveryGet(&s);
  if (s.active) return;
  BusRecoveryPending=0;
  if (s.success) {
    bool chassis_ready=Mecanum_RecoveryAfterBusCheck();
    ArmConsole_Printf("OK bus recover complete arm_ready=1 chassis_ready=%u reason=%s tasks_resumed=0 home_reference=check chassis_cache=cleared wheels_stopped=1\r\n",
      (unsigned)chassis_ready,chassis_ready?"complete":"chassis_cache_or_state_requires_reset");
  } else ArmConsole_Printf("ERR bus recover reason=%s addr=%u locked=1 speed_addr=%u speed_rpm=%ld speed_ms=%lu\r\n",
    s.reason,(unsigned)s.address,(unsigned)s.speed_address,(long)s.speed_rpm,(unsigned long)s.speed_ms);
}

/* Manual commands are checked before dispatch; automatic tasks own their own
 * feedback gates. One pending command only, so double clicks cannot build a queue. */
static uint8_t ArmConsole_ManualGuardStart(uint8_t count,char *tokens[])
{
  unsigned i;size_t used=0;MechanicalArm_AxisTypeDef axis;
  if (ManualGuardExecuting || count<2 ||
      (strcmp(tokens[0],"pos") && strcmp(tokens[0],"home") && strcmp(tokens[0],"zero") &&
       strcmp(tokens[0],"origin") && strcmp(tokens[0],"disable"))) return 0;
  axis=MechanicalArm_AxisGet(tokens[1]);
  if (axis==MECHANICAL_ARM_AXIS_INVALID) return 0;
  if (MechanicalArm_IsBusy() || MotorBus_IsQuarantined()) {
    ArmConsole_Write("ERR motor_busy_or_locked use info or bus recover\r\n");return 1;
  }
  /* Address order differs from enum order: base=5, z=6, x=7. */
  ManualGuardAddress=axis==MECHANICAL_ARM_AXIS_ALL?5:(uint8_t)(5+(unsigned)axis);
  ManualGuardLast=axis==MECHANICAL_ARM_AXIS_ALL?7:ManualGuardAddress;
  for(i=0;i<count;i++) {
    int n=snprintf(ManualGuardLine+used,sizeof(ManualGuardLine)-used,"%s%s",i?" ":"",tokens[i]);
    if(n<0 || (size_t)n>=sizeof(ManualGuardLine)-used) { ArmConsole_Write("ERR format\r\n");return 1; }
    used+=(size_t)n;
  }
  ManualGuardKind=0;ManualGuardActive=1;ManualGuardStarted=HAL_GetTick();
  ArmConsole_Write("OK manual checking_idle\r\n");return 1;
}
static void ArmConsole_ManualGuardProcess(void)
{
  MotorBus_Event_t e;uint8_t cmd[3];
  static const uint8_t function[3]={0x35,0x3b,0x3a};
  if (!ManualGuardActive) { (void)MotorBus_EventGet(MOTOR_BUS_GUARD,&e);return; }
  if ((uint32_t)(HAL_GetTick()-ManualGuardStarted)>2000U) {
    ArmConsole_ManualGuardCancel();ArmConsole_Write("ERR manual check_timeout\r\n");return;
  }
  if (MotorBus_EventGet(MOTOR_BUS_GUARD,&e)) {
    if (e.result!=MOTOR_BUS_REPLY) {
      ManualGuardActive=0;ArmConsole_Write("ERR manual feedback_unavailable use info\r\n");return;
    }
    if ((ManualGuardKind==0 && (((unsigned)e.data[3]<<8)|e.data[4])!=0) ||
        (ManualGuardKind==1 && (e.data[2]&0x34U)) || (ManualGuardKind==2 && (e.data[2]&0x0cU))) {
      ManualGuardActive=0;
      ArmConsole_Printf("ERR manual axis_busy_or_fault addr=%u command_not_sent\r\n",(unsigned)ManualGuardAddress);return;
    }
    if (++ManualGuardKind==3) {
      ManualGuardKind=0;
      if (++ManualGuardAddress>ManualGuardLast) {
        ManualGuardActive=0;ManualGuardExecuting=1;
        (void)ArmConsole_CommandExecute(ManualGuardLine);
        ManualGuardExecuting=0;return;
      }
    }
  }
  if (MotorBus_OwnerBusy(MOTOR_BUS_GUARD)) return;
  cmd[0]=ManualGuardAddress;cmd[1]=function[ManualGuardKind];cmd[2]=0x6b;
  if (!MotorBus_Submit(MOTOR_BUS_GUARD,cmd,3,ManualGuardKind==0?6:4,0,false)) {
    ManualGuardActive=0;ArmConsole_Write("ERR manual bus_busy_or_locked command_not_sent\r\n");
  }
}

void ArmConsole_StopProcess(void)
{
  uint32_t primask = __get_PRIMASK();
  uint8_t pending;
  __disable_irq();
  pending = ArmConsole_StopPending;
  ArmConsole_StopPending = 0U;
  if (pending)
  {
    ArmConsole_LineLength = ArmConsole_LineOverflow = ArmConsole_LastWasCr = 0U;
    ArmConsole_LineBuffer[0] = '\0';
    ArmConsole_RingOverflow = 0U;
  }
  __set_PRIMASK(primask);
  if (pending)
  {
    ArmConsole_Write("\r\n");
    ArmConsole_StopRequest(0);
  }
  ArmConsole_StopStatusProcess();
}

static uint8_t ArmConsole_StopCommandHandle(uint8_t count, char *tokens[])
{
  uint32_t token = 0;
  const char *p;
  if (count < 2U || (strcmp(tokens[0], "chassis") && strcmp(tokens[0], "wheel"))) return 0;
  if (!strcmp(tokens[0], "chassis") && !strcmp(tokens[1], "stop-status"))
  {
    if (count == 2U) ArmConsole_StopStatusShow();
    else ArmConsole_Write("ERR format: chassis stop-status\r\n");
    return 1;
  }
  if (strcmp(tokens[1], "stop")) return 0;
  if (count == 3U && !strcmp(tokens[0], "chassis"))
  {
    for (p = tokens[2]; *p; p++)
    {
      if (*p < '0' || *p > '9' || token > (UINT32_MAX - (uint32_t)(*p - '0')) / 10U)
        break;
      token = token * 10U + (uint32_t)(*p - '0');
    }
    if (*p || !token) { ArmConsole_Write("ERR stop token: 1..4294967295\r\n"); return 1; }
  }
  else if (count != 2U)
  { ArmConsole_Write("ERR format: chassis stop [client_token]\r\n"); return 1; }
  ArmConsole_StopRequest(token);
  return 1;
}

uint8_t ArmConsole_ResetPending(void) { return ResetPending; }
uint8_t ArmConsole_OperationBusy(void) { return BusRecoveryPending || ManualGuardActive || BusTest.active; }

uint8_t ArmConsole_ResetProcess(void)
{
  ConsoleTx_Stats_t tx;
  if (!ResetPending) return 0U;
  ConsoleTx_GetStats(&tx);
  if (tx.urgent_dropped != ResetDropBaseline || (uint32_t)(HAL_GetTick() - ResetStarted) > 5000U)
  {
    ArmConsole_Write(tx.urgent_dropped != ResetDropBaseline ?
      "ERR system reset ack_failed cancelled\r\n" : "ERR system reset ack_timeout cancelled\r\n");
    ResetPending = 0U;
    return 0U;
  }
  if (ConsoleTx_UrgentIdle())
  {
    if (!ResetAckDrained) { ResetAckDrained = 1U; ResetDrainTick = HAL_GetTick(); }
    else if ((uint32_t)(HAL_GetTick() - ResetDrainTick) >= 200U)
    {
      uint32_t primask = __get_PRIMASK();
      __disable_irq();
      if (!ArmConsole_StopPending)
      {
        ResetPending = 0U;
        NVIC_SystemReset();
      }
      __set_PRIMASK(primask);
    }
  }
  else ResetAckDrained = 0U;
  return ResetPending;
}

static uint8_t ArmConsole_SystemCommandHandle(uint8_t count, char *tokens[])
{
  static const char reply[] = "\r\nOK system reset pending target=stm32 peripherals_not_reset=1\r\n";
  ConsoleTx_Stats_t tx;
  if (strcmp(tokens[0], "system")) return 0U;
  if (count != 2U || strcmp(tokens[1], "reset"))
    ArmConsole_Write("ERR format: system reset\r\n");
  else if (ChassisMotion_IsBusy() || ChassisRoute_IsBusy() || Mecanum_IsBusy() ||
           MechanicalArm_IsBusy() || HWT101_Cal_IsBusy() ||
           GrabTask_IsBusy() || GrabRoute_IsBusy() || Turntable_IsBusy())
    ArmConsole_Write("ERR system reset busy stop motion and finish/cancel calibration first\r\n");
  else
  {
    ConsoleTx_GetStats(&tx);
    ResetDropBaseline = tx.urgent_dropped;
    if (!ConsoleTx_Urgent(reply, sizeof(reply) - 1U))
      ArmConsole_Write("ERR system reset ack_queue_full cancelled\r\n");
    else { ResetPending = 1U; ResetAckDrained = 0U; ResetStarted = HAL_GetTick(); }
  }
  return 1U;
}

static uint8_t ArmConsole_BusCommandHandle(unsigned count,char *tokens[])
{
  if(count>=2U && !strcmp(tokens[0],"bus") && !strcmp(tokens[1],"test"))
    return ArmConsole_BusTestCommand(count,tokens);
  if (count==2 && !strcmp(tokens[0],"bus")) {
    if (!strcmp(tokens[1],"status")) ArmConsole_BusStatusShow();
    else if (!strcmp(tokens[1],"recover")) {
      if (BusRecoveryPending) ArmConsole_Write("ERR bus recovery_busy\r\n");
      else {
        ArmConsole_StopRequest(0);
        Mecanum_Feedback_Enable(false);
        (void)MechanicalArm_Stop(MECHANICAL_ARM_AXIS_ALL);
        BusRecoveryPending=1;BusRecoveryStarted=HAL_GetTick();
        ArmConsole_Write("OK bus recover requested tasks_resumed=0\r\n");
      }
    } else ArmConsole_Write("ERR format: bus recover | bus status\r\n");
    return 1;
  }
  return 0U;
}

static uint8_t ArmConsole_TurntableCommandHandle(unsigned count,char *tokens[])
{
  if (!strcmp(tokens[0],"turntable"))
  {
    if (count==2U && !strcmp(tokens[1],"stop")) ArmConsole_ResetCancel();
    else if (MechanicalArm_IsBusy() ||
             Mecanum_IsBusy() || ChassisRoute_IsBusy() || ChassisMotion_IsBusy() || HWT101_Cal_IsBusy())
    { ArmConsole_Write("ERR turntable other_task_busy\r\n");return 1U; }
    return Turntable_Command(count,tokens) ? 1U : 0U;
  }
  return 0U;
}

static uint8_t ArmConsole_TurntableBusyGuard(unsigned count,char *tokens[])
{
  if (Turntable_IsBusy() && strcmp(tokens[0],"turntable") &&
      strcmp(tokens[0],"stop") &&
      !(count==1U && (!strcmp(tokens[0],"help") || !strcmp(tokens[0],"info"))) &&
      !(count==2U && (!strcmp(tokens[1],"status") || !strcmp(tokens[1],"stop") ||
        !strcmp(tokens[0],"config") || !strcmp(tokens[0],"state") || !strcmp(tokens[0],"position"))))
  { ArmConsole_Write("ERR turntable_busy use turntable status or turntable stop\r\n");return 1U; }
  return 0U;
}

static uint8_t ArmConsole_TrackCommandHandle(unsigned count,char *tokens[])
{
  if(count<3U || strcmp(tokens[0],"grab") || strcmp(tokens[1],"track"))return 0U;
  if(GrabRoute_IsBusy()) { ArmConsole_Write("ERR grab track route_busy\r\n"); return 1U; }
  if(!strcmp(tokens[2],"observe") || !strcmp(tokens[2],"start"))ArmConsole_CameraTraceStop();
  (void)GrabTask_Command(count,tokens);
  return 1U;
}

static uint8_t ArmConsole_GrabQueryHandle(unsigned count,char *tokens[])
{
  /* Cached status queries do not compete for actuator ownership. */
  if(!strcmp(tokens[0],"chassis"))
  {
    if(count==2U && !strcmp(tokens[1],"task"))
      return ChassisTelemetry_Command(count,tokens)?1U:0U;
    if(count==3U && !strcmp(tokens[1],"route") && !strcmp(tokens[2],"status"))
      return ChassisRoute_Command(count,tokens)?1U:0U;
  }
  if(strcmp(tokens[0],"grab"))return 0U;
  if(count==3U && !strcmp(tokens[1],"calib") && !strcmp(tokens[2],"status"))
    return GrabTask_Command(count,tokens)?1U:0U;
  if(count==3U && (!strcmp(tokens[1],"route") || !strcmp(tokens[1],"mission") || !strcmp(tokens[1],"qr")) &&
     !strcmp(tokens[2],"status")) return GrabRoute_Command(count,tokens)?1U:0U;
  if(!(((count==2U || count==3U) && !strcmp(tokens[1],"status")) || (count==3U && !strcmp(tokens[1],"get")) ||
       (count>=3U && !strcmp(tokens[1],"track") &&
        (!strcmp(tokens[2],"status") || !strcmp(tokens[2],"get") || !strcmp(tokens[2],"trace") || !strcmp(tokens[2],"log")))))return 0U;
  (void)GrabRoute_Command(count,tokens); (void)GrabTask_Command(count,tokens);
  return 1U;
}

static bool ArmConsole_GrabBusyGuard(unsigned TokenCount,char *Tokens[])
{
  if ((GrabTask_IsBusy() || GrabRoute_IsBusy()) &&
      !(TokenCount==2U && !strcmp(Tokens[0],"grip") && GrabTask_RingManualAllowed()) &&
      !(GrabRoute_CanConfigure() && !strcmp(Tokens[0],"grab") &&
        ((TokenCount==4U && !strcmp(Tokens[1],"set")) ||
         (TokenCount==3U && !strcmp(Tokens[1],"mission") && !strcmp(Tokens[2],"continue")))) &&
      !(TokenCount==3U && !strcmp(Tokens[0],"grab") && !strcmp(Tokens[1],"qr") &&
        (!strcmp(Tokens[2],"status") || !strcmp(Tokens[2],"stop"))) &&
      !((TokenCount==1U && (!strcmp(Tokens[0],"help") || !strcmp(Tokens[0],"info"))) ||
        (TokenCount==2U && ((!strcmp(Tokens[1],"status") &&
           (!strcmp(Tokens[0],"camera") ||   !strcmp(Tokens[0],"console") || !strcmp(Tokens[0],"qr"))) ||
          (!strcmp(Tokens[0],"qr") && !strcmp(Tokens[1],"read")))) ||
        (TokenCount==3U && !strcmp(Tokens[0],"chassis") && !strcmp(Tokens[1],"route") && !strcmp(Tokens[2],"status"))))
    return true;
  return false;
}
static uint8_t ArmConsole_CommandExecute(char *Line)
{
  char *Tokens[ARM_CONSOLE_TOKEN_COUNT];
  uint8_t TokenCount;

  TokenCount = ArmConsole_Tokenize(Line, Tokens);
  if (TokenCount == 0U)
  {
    ArmConsole_Write("ERR format\r\n");
    return 1U;
  }
  if (!strcmp(Tokens[0],"material"))
  {
    ArmConsole_Write("ERR retired: material commands removed; use grab align color|ring, grab calib ref|status|<1..6>, grab stop\r\n");
    return 1U;
  }
  if (!strcmp(Tokens[0], "vision"))
  {
    ArmConsole_Write("ERR retired: use grab align color|ring or grab calib ref|status|<1..6>\r\n");
    return 1U;
  }
  /* Queries and log subscription never operate actuators and bypass motion guards. */
  if (ArmConsole_StopCommandHandle(TokenCount, Tokens)) return 1U;
  if (ArmConsole_PixelLogCommand(TokenCount, Tokens)) return 1U;
  if (!strcmp(Tokens[0],"turntable") &&
      ((TokenCount==2U && (!strcmp(Tokens[1],"status") || !strcmp(Tokens[1],"inventory"))) ||
       (TokenCount==3U && !strcmp(Tokens[1],"get"))))
    return Turntable_Command(TokenCount,Tokens) ? 1U : 0U;
  if (ArmConsole_BusCommandHandle(TokenCount,Tokens)) return 1U;
  if(BusTest.active && !((TokenCount==1U && (!strcmp(Tokens[0],"info") || !strcmp(Tokens[0],"help"))) ||
     (TokenCount>=2U && (!strcmp(Tokens[0],"stop") || !strcmp(Tokens[1],"stop"))) ||
     (TokenCount==3U && !strcmp(Tokens[0],"grab") && !strcmp(Tokens[1],"track") && !strcmp(Tokens[2],"stop")))) {
    ArmConsole_Write("ERR bus test active; use bus test status or bus test stop\r\n");return 1U;
  }
  if ((TokenCount==2 && (!strcmp(Tokens[0],"stop") || !strcmp(Tokens[1],"stop"))) ||
      (TokenCount==3 && !strcmp(Tokens[0],"grab") && !strcmp(Tokens[1],"track") && !strcmp(Tokens[2],"stop"))) {
    ArmConsole_ManualGuardCancel();
    ArmConsole_BusTestStop("stopped");
    BusRecoveryPending=0;MotorBus_RecoveryCancel();
    if (!strcmp(Tokens[0],"stop") && !strcmp(Tokens[1],"all")) Turntable_Stop();
  }
  if ((BusRecoveryPending || ManualGuardActive) &&
      !(TokenCount==1 && (!strcmp(Tokens[0],"info") || !strcmp(Tokens[0],"help"))) &&
      !(TokenCount==2 && (!strcmp(Tokens[1],"status") || !strcmp(Tokens[0],"state") || !strcmp(Tokens[0],"position"))) &&
      !(TokenCount==3 && !strcmp(Tokens[0],"grab") && !strcmp(Tokens[1],"calib") && !strcmp(Tokens[2],"status")) &&
      !(TokenCount>=3 && !strcmp(Tokens[0],"grab") && !strcmp(Tokens[1],"track") &&
        (!strcmp(Tokens[2],"status") || !strcmp(Tokens[2],"get") || !strcmp(Tokens[2],"trace") || !strcmp(Tokens[2],"stop")))) {
    ArmConsole_Write("ERR operation_busy command_not_queued\r\n");return 1;
  }
  /* Stopping any owned actuator cancels the whole grasp producer before dispatch.
   * Claw PWM and the load-bearing Z enable remain unchanged. */
  if ((TokenCount==2U && !strcmp(Tokens[0],"grab") && !strcmp(Tokens[1],"stop")) ||
      (TokenCount==3U && !strcmp(Tokens[0],"grab") && (!strcmp(Tokens[1],"track") || !strcmp(Tokens[1],"qr")) && !strcmp(Tokens[2],"stop")) ||
      ((GrabTask_IsBusy() || GrabRoute_IsBusy()) &&
       ((TokenCount==2U && ((!strcmp(Tokens[0],"stop") &&
          MechanicalArm_AxisGet(Tokens[1])!=MECHANICAL_ARM_AXIS_INVALID) ||
         (!strcmp(Tokens[1],"stop") && (!strcmp(Tokens[0],"camera") || !strcmp(Tokens[0],"turntable"))))) ||
        (TokenCount==3U && !strcmp(Tokens[0],"chassis") && !strcmp(Tokens[1],"route") && !strcmp(Tokens[2],"cancel")))))
  {
    ArmConsole_ResetCancel();
    if(!GrabTask_IsBusy())GrabTask_Stop(); /* An error is inactive but still needs cancellation. */
    GrabRoute_Stop();Turntable_Stop();
    ArmConsole_Write("OK grab stop requested physical_stop=unconfirmed claw=held\r\n");
    return 1U;
  }
  if (ResetPending)
  {
    /* Every valid foreground stop keeps its existing scope and cancels reset. */
    if (TokenCount == 2U &&
        ((!strcmp(Tokens[0], "stop") && MechanicalArm_AxisGet(Tokens[1]) != MECHANICAL_ARM_AXIS_INVALID) ||
         (!strcmp(Tokens[1], "stop") && (!strcmp(Tokens[0], "camera") || !strcmp(Tokens[0], "turntable")))))
      ArmConsole_ResetCancel();
    else { ArmConsole_Write("ERR system reset_pending\r\n"); return 1U; }
  }
  if (ArmConsole_SystemCommandHandle(TokenCount, Tokens)) return 1U;
  if (RadarConsole_Command(TokenCount,Tokens)) return 1U;
  if(ArmConsole_RadarMotionGuard(TokenCount,Tokens))
  { ArmConsole_Write("ERR radar stationary capture active; radar stop before driving\r\n");return 1U; }
  if(ArmConsole_GrabQueryHandle(TokenCount,Tokens))return 1U;
  if (ArmConsole_TurntableBusyGuard(TokenCount,Tokens)) return 1U;
  /* Track owns its feedback reads while observing/holding; do not mistake an
   * in-flight owned query for a competing manual motion producer. */
  if(ArmConsole_TrackCommandHandle(TokenCount,Tokens))return 1U;
  if(ArmConsole_RingCommandHandle(TokenCount,Tokens))return 1U;
  if(ArmConsole_GrabBusyGuard(TokenCount,Tokens))
  { ArmConsole_Write("ERR grab_busy use grab status or grab stop\r\n");return 1U; }
  if (ArmConsole_TurntableCommandHandle(TokenCount,Tokens)) return 1U;
  if (!strcmp(Tokens[0],"grab"))
  {
    if (MechanicalArm_IsBusy() ||
        Mecanum_IsBusy() || ChassisRoute_IsBusy() || ChassisMotion_IsBusy() || HWT101_Cal_IsBusy())
    { ArmConsole_Write("ERR grab other_task_busy\r\n");return 1U; }
    if(TokenCount>=2U && (!strcmp(Tokens[1],"start") || !strcmp(Tokens[1],"mission") || !strcmp(Tokens[1],"route") || !strcmp(Tokens[1],"store") || !strcmp(Tokens[1],"take") || !strcmp(Tokens[1],"return") || !strcmp(Tokens[1],"ring") || !strcmp(Tokens[1],"align") || !strcmp(Tokens[1],"calib")))
      ArmConsole_CameraTraceStop(); /* Give the latest-only vision slot to grab progress. */
    if (GrabRoute_Command(TokenCount,Tokens)) return 1U;
    return GrabTask_Command(TokenCount,Tokens) ? 1U : 0U;
  }
  if (TokenCount == 2U && !strcmp(Tokens[0], "console") && !strcmp(Tokens[1], "status")) {
    ConsoleTx_Stats_t tx;
    ConsoleRx_Stats_t rx;
    ConsoleTx_GetStats(&tx); ConsoleRx_GetStats(&rx);
    ArmConsole_Printf("OK console rx_bytes=%lu rx_overflow=%lu rx_peak=%u uart_errors=%lu ore=%lu fe=%lu ne=%lu dma_errors=%lu\r\n",
      (unsigned long)rx.bytes_received, (unsigned long)ArmConsole_RxOverflowCount, ArmConsole_RxPeak,
      (unsigned long)rx.uart_errors, (unsigned long)rx.overruns, (unsigned long)rx.framing_errors,
      (unsigned long)rx.noise_errors, (unsigned long)rx.dma_errors);
    ArmConsole_Printf("OK console tx_bytes=%lu reply_pending=%u reply_peak=%u reply_dropped=%lu urgent_dropped=%lu debug_dropped=%lu\r\n",
      (unsigned long)tx.bytes_sent, tx.reply_pending, tx.reply_peak, (unsigned long)tx.reply_dropped,
      (unsigned long)tx.urgent_dropped, (unsigned long)tx.debug_dropped);
    ArmConsole_Printf("OK console rx_restarts=%lu rx_restart_failures=%lu telemetry_dropped=%lu\r\n",
      (unsigned long)rx.restarts, (unsigned long)rx.restart_failures, (unsigned long)ConsoleTx_Dropped());
    return 1U;
  }
  if (!strcmp(Tokens[0], "qr")) return ArmConsole_QrCommandHandle(TokenCount, Tokens);
  if (ChassisRoute_Command(TokenCount, Tokens)) return 1U;
  if (ChassisTelemetry_Command(TokenCount, Tokens)) return 1U;
  if (ChassisMotion_IsBusy() || ChassisRoute_IsBusy())
  {
    ChassisMotion_Status_t motion;
    ChassisMotion_StatusGet(&motion);
    if (!strcmp(Tokens[0], "stop")) (void)ChassisRoute_Cancel();
    else if (!((TokenCount == 1U && (!strcmp(Tokens[0], "help") || !strcmp(Tokens[0], "info"))) ||
               (TokenCount == 2U && (!strcmp(Tokens[1], "status") || !strcmp(Tokens[1], "stop") || !strcmp(Tokens[0], "state") || !strcmp(Tokens[0], "position") || !strcmp(Tokens[0], "config"))) ||
               (TokenCount == 3U && !strcmp(Tokens[0], "imu") && !strcmp(Tokens[1], "stream")) ||
               (TokenCount == 3U && !strcmp(Tokens[0],"imu") && !strcmp(Tokens[1],"cal") &&
                (!strcmp(Tokens[2],"cancel") || !strcmp(Tokens[2],"clear"))) ||
               (motion.indefinite_velocity && TokenCount>=2U && !strcmp(Tokens[0],"chassis") &&
                (!strcmp(Tokens[1],"velocity") || !strcmp(Tokens[1],"backward")))))
    {
      ArmConsole_Write("ERR chassis_task_busy use chassis stop\r\n");
      return 1U;
    }
  }
  if ((TokenCount == 1U) && (strcmp(Tokens[0], "help") == 0))
  {
    ArmConsole_HelpShow();
    return 1U;
  }
  if ((TokenCount == 1U) && (strcmp(Tokens[0], "info") == 0))
  {
    ArmConsole_InfoShow();
    return 1U;
  }

  if (ArmConsole_StatusCommandHandle(TokenCount, Tokens) != 0U)
  {
    return 1U;
  }
  /* Calibration permits only IMU commands, queries and stops. */
  if (HWT101_Cal_IsBusy() && strcmp(Tokens[0], "imu") && strcmp(Tokens[0], "stop") &&
      !((TokenCount == 2U) && (!strcmp(Tokens[1], "status") || !strcmp(Tokens[1], "stop"))))
  {
    ArmConsole_Write("ERR imu_busy use imu cal cancel\r\n");
    return 1U;
  }

  if (strcmp(Tokens[0], "imu") == 0)
  {
    ArmConsole_Write("ERR imu format: imu status | imu cal start|cancel | imu zero | imu verify | imu stream on|off\r\n");
    return 1U;
  }

  if (strcmp(Tokens[0], "grip") == 0)
    return ArmConsole_GripCommandHandle(TokenCount, Tokens);
  if (strcmp(Tokens[0], "camera") == 0)
    return ArmConsole_VisionCommandHandle(TokenCount, Tokens);
  if ((strcmp(Tokens[0], "chassis") == 0) || (strcmp(Tokens[0], "wheel") == 0))
    return ArmConsole_ChassisCommandHandle(TokenCount, Tokens);
  if (ArmConsole_ManualGuardStart(TokenCount,Tokens)) return ManualGuardActive?0U:1U;
  return ArmConsole_MechanicalArmCommandHandle(TokenCount, Tokens);
}

/**
  * 函    数：初始化USART1机械臂终端
  * 参    数：huart USART1句柄
  * 返 回 值：HAL执行状态
  * 说    明：清空终端缓存并显示启动信息，循环DMA接收由main启动
  */
HAL_StatusTypeDef ArmConsole_Init(UART_HandleTypeDef *huart)
{
  memset(&StopReported, 0, sizeof StopReported);
  if ((huart == NULL) || (huart->Instance != USART1))
  {
    return HAL_ERROR;
  }

  ArmConsole_Uart = huart;
  ConsoleTx_Init(huart);
  ResetPending = ResetAckDrained = 0U;
  BusRecoveryPending=ManualGuardActive=ManualGuardExecuting=0U;
  memset(&BusTest,0,sizeof BusTest);BusTestToken=0U;
  ArmConsole_ImuStream = 0U;
  ArmConsole_QrStream = 0U;
  memset(&ArmConsole_QrPending, 0, sizeof(ArmConsole_QrPending));
  ArmConsole_QrSeenSequence = ArmConsole_QrLastEventTick = ArmConsole_QrSkipped = 0U;
  ArmConsole_RingWrite = 0U;
  ArmConsole_RingRead = 0U;
  ArmConsole_RingOverflow = 0U;
  ArmConsole_RxDiscard = 0U;
  ArmConsole_RxOverflowCount = 0U;
  ArmConsole_RxPeak = 0U;
  ArmConsole_StopPending = 0U;
  ArmConsole_LineLength = 0U;
  ArmConsole_LineOverflow = 0U;
  ArmConsole_LastWasCr = 0U;
  ArmConsole_CameraTrace = 0U;
  ArmConsole_CameraTraceSequence = 0U;
  memset(ArmConsole_RingBuffer, 0, sizeof(ArmConsole_RingBuffer));
  PixelLogEnabled=0;PixelLogId=PixelLogCount=PixelLogQueued=PixelLogDropped=0;
  Camera_FrameObserverSet(NULL);
  memset(ArmConsole_LineBuffer, 0, sizeof(ArmConsole_LineBuffer));
  ArmConsole_Write("\r\nSTM32 mechanical arm console ready\r\n");
  ArmConsole_BuildShow();
  ArmConsole_Write("Boot: Z/X collision home + Base/ID8 near home in parallel -> IMU verify=5s; enable_verify=on; keep still after homing; stream=off; imu cal start: 20s native calibration + 30s verification\r\n");
  ArmConsole_PromptShow();
  return HAL_OK;
}

uint8_t ArmConsole_ImuStreamEnabled(void)
{
  return ArmConsole_ImuStream;
}

/**
  * 函    数：接收一个USART1中断字节
  * 参    数：Data 上位机发来的字节
  * 返 回 值：无
  * 说    明：写入512字节环形缓冲；满时丢弃受损命令并等待换行重新同步
  */
void ArmConsole_ReceiveData(uint8_t Data)
{
  uint16_t NextWrite;
  uint16_t Used;

  if (Data == 0x03U)
  {
    /* Out-of-band latch cannot be lost even when the ASCII ring is full.
     * Discard input preceding this byte; following tokenized stop remains readable. */
    ArmConsole_RingRead = ArmConsole_RingWrite;
    ArmConsole_RxDiscard = 0U;
    ArmConsole_StopPending = 1U;
    return;
  }

  if (ArmConsole_RxDiscard) {
    if (Data != '\r' && Data != '\n') return;
    ArmConsole_RxDiscard = 0U;
  }

  NextWrite = (uint16_t)((ArmConsole_RingWrite + 1U) % ARM_CONSOLE_RING_SIZE);
  if (NextWrite == ArmConsole_RingRead)
  {
    ArmConsole_RxOverflowCount++;
    ArmConsole_ReceiveFault();
    return;
  }
  ArmConsole_RingBuffer[ArmConsole_RingWrite] = Data;
  ArmConsole_RingWrite = NextWrite;
  Used = (uint16_t)((NextWrite + ARM_CONSOLE_RING_SIZE - ArmConsole_RingRead) % ARM_CONSOLE_RING_SIZE);
  if (Used > ArmConsole_RxPeak) ArmConsole_RxPeak = Used;
}

void ArmConsole_ReceiveFault(void)
{
  ArmConsole_RingRead = ArmConsole_RingWrite;
  ArmConsole_RxDiscard = 1U;
  ArmConsole_RingOverflow = 1U;
}

/**
  * 函    数：处理USART1命令行和机械臂异步结果
  * 参    数：无
  * 返 回 值：无
  * 说    明：支持CR、LF、CRLF、退格和超长命令丢弃
  */
void ArmConsole_Process(void)
{
  uint8_t Data;
  uint8_t Result;

  ArmConsole_StopProcess();

  ArmConsole_CameraTraceShow();

  while ((Result = ArmConsole_RingPop(&Data)) != 0U)
  {
    if (Result == 3U) {
      ArmConsole_Write("\r\nERR rx data lost; discard through newline\r\n");
      continue;
    }
    if (ArmConsole_StopPending)
    {
      ArmConsole_StopProcess();
      continue; /* The byte popped before Ctrl+C belongs to discarded input. */
    }
    if (ArmConsole_RingOverflow) continue;
    if ((Data == '\r') || (Data == '\n'))
    {
      if ((Data == '\n') && (ArmConsole_LastWasCr != 0U))
      {
        ArmConsole_LastWasCr = 0U;
        continue;
      }
      ArmConsole_LastWasCr = (Data == '\r') ? 1U : 0U;
      ArmConsole_Write("\r\n");
      if (ArmConsole_LineOverflow != 0U)
      {
        ArmConsole_Write("ERR format\r\n");
        ArmConsole_PromptShow();
      }
      else if (ArmConsole_LineLength > 0U)
      {
        ArmConsole_LineBuffer[ArmConsole_LineLength] = '\0';
        if (ArmConsole_CommandExecute(ArmConsole_LineBuffer) != 0U)
        {
          ArmConsole_PromptShow();
        }
      }
      else
      {
        ArmConsole_PromptShow();
      }
      ArmConsole_LineLength = 0U;
      ArmConsole_LineOverflow = 0U;
      continue;
    }

    ArmConsole_LastWasCr = 0U;
    if ((Data == 0x08U) || (Data == 0x7FU))
    {
      if (ArmConsole_LineLength > 0U)
      {
        ArmConsole_LineLength--;
        ArmConsole_Write("\b \b");
      }
      continue;
    }
    if ((Data < 0x20U) || (Data > 0x7EU))
    {
      continue;
    }
    ArmConsole_Write((char[2]){(char)Data, '\0'});
    if (ArmConsole_LineLength >= (ARM_CONSOLE_LINE_SIZE - 1U))
    {
      ArmConsole_LineOverflow = 1U;
      continue;
    }
    ArmConsole_LineBuffer[ArmConsole_LineLength] = (char)Data;
    ArmConsole_LineLength++;
  }
  /* Handle stream off commands before producing the next QR event. */
  ArmConsole_ManualGuardProcess();
  ArmConsole_BusTestProcess();
  ArmConsole_BusRecoveryProcess();
  ArmConsole_QrEventProcess();
}
