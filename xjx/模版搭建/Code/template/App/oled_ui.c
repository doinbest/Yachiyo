#include "oled_ui.h"

#include "oled.h"
#include "Camera.h"
#include "ArmVision.h"
#include "MaterialVision.h"
#include "hwt101_i2c.h"
#include "tjc_screen.h"
#include "Arm.h"
#include "chassis_motion.h"
#include "chassis_route.h"

#include <stdio.h>
#include <string.h>

#define OLED_UI_COLUMNS 21U
#define OLED_UI_ROWS 8U
#define OLED_UI_REFRESH_MS 200U
#define OLED_UI_RETRY_MS 1000U
#define OLED_UI_SERVO_OPEN_US 500U
#define OLED_UI_SERVO_CATCH_US 700U

typedef enum
{
  OLED_UI_OVERVIEW = 0,
  OLED_UI_VISION,
  OLED_UI_LINK,
  OLED_UI_SERVO,
  OLED_UI_PAGE_COUNT
} OledUi_PageTypeDef;

static I2C_HandleTypeDef *OledUi_I2c;
static OledUi_PageTypeDef OledUi_Page;
static char OledUi_Lines[OLED_UI_ROWS][OLED_UI_COLUMNS + 1U];
static char OledUi_Sent[OLED_UI_ROWS][OLED_UI_COLUMNS + 1U];
static char OledUi_TaskCode[16];
static char OledUi_Error[17]; /* Err:加最多16字符，不截断诊断行。 */
static uint8_t OledUi_SelectedIndex;
static uint8_t OledUi_CodeReady;
static uint8_t OledUi_DirtyRows;
static uint8_t OledUi_Ready;
static uint8_t OledUi_RetryPending;
static uint8_t OledUi_Rebuild;
static uint8_t OledUi_LastArmBusy;
static uint8_t OledUi_LastMaterialBusy;
static uint8_t OledUi_TaskError; /* 0无任务错误，1为ArmVision，2为MaterialVision。 */
static uint8_t OledUi_LastTaskIsMaterial;
static ArmVision_ErrorTypeDef OledUi_LastArmError;
static MaterialVision_ErrorTypeDef OledUi_LastMaterialError;
static uint32_t OledUi_LastRefreshTick;
static uint32_t OledUi_LastFailureTick;
static uint16_t OledUi_ServoPulseUs;
static uint8_t OledUi_ServoActive;

static const char *OledUi_ColorName(uint8_t Color)
{
  static const char *Names[] = {"None", "Red", "Yellow", "Blue", "Green", "Black", "Lightblue"};
  return (Color <= 6U) ? Names[Color] : "Invalid";
}

/** 计数和数据年龄统一封顶；完整值由控制台status命令查看。 */
static void OledUi_CountText(char Text[6], uint32_t Count)
{
  if (Count > 9999U) (void)strcpy(Text, "9999+");
  else (void)snprintf(Text, 6U, "%lu", (unsigned long)Count);
}

static const char *OledUi_ArmErrorName(ArmVision_ErrorTypeDef Error)
{
  static const char *Names[] =
  {
    "None", "Camera wait", "Camera stale", "Camera tx", "Motor tx",
    "Motor busy", "Motor param", "Position ack", "State ack", "Motor fault",
    "Motor arrival", "Travel limit", "Zero correction", "Calibration",
    "Task timeout", "Internal", "Camera busy", "Usb off"
  };
  return ((unsigned)Error < sizeof(Names) / sizeof(Names[0])) ? Names[Error] : "Internal";
}

static const char *OledUi_MaterialErrorName(MaterialVision_ErrorTypeDef Error)
{
  static const char *Names[] =
  {
    "None", "Busy", "Not ready", "Camera wait", "Camera stale", "Camera tx",
    "Home timeout", "Motor ack", "Motor arrival", "Motor fault", "Search timeout",
    "Calibration", "Zero correction", "X limit", "Task timeout", "Internal",
    "Chassis tx", "Motor busy", "Motor param", "Motor tx", "Chassis limit",
    "Camera busy", "Usb off"
  };
  return ((unsigned)Error < sizeof(Names) / sizeof(Names[0])) ? Names[Error] : "Internal";
}

/** 每圈观察任务变化，避免200ms显示周期漏掉一次启动或停止；翻页不清错误。 */
static void OledUi_ErrorUpdate(void)
{
  uint8_t ArmBusy = ArmVision_IsBusy();
  uint8_t MaterialBusy = MaterialVision_IsBusy();
  ArmVision_ErrorTypeDef ArmError = ArmVision_ErrorGet();
  MaterialVision_ErrorTypeDef MaterialError = MaterialVision_ErrorGet();
  uint8_t NewTask = ((ArmBusy && !OledUi_LastArmBusy) ||
                     (MaterialBusy && !OledUi_LastMaterialBusy));
  uint8_t Stopped = ((!ArmBusy && OledUi_LastArmBusy && ArmError == ARM_VISION_ERROR_NONE) ||
                     (!MaterialBusy && OledUi_LastMaterialBusy && MaterialError == MATERIAL_VISION_ERROR_NONE));
  uint8_t ErrorCleared = ((OledUi_TaskError == 1U && OledUi_LastArmError != ARM_VISION_ERROR_NONE &&
                          ArmError == ARM_VISION_ERROR_NONE && strcmp(ArmVision_StateNameGet(), "IDLE") == 0) ||
                          (OledUi_TaskError == 2U && OledUi_LastMaterialError != MATERIAL_VISION_ERROR_NONE &&
                           MaterialError == MATERIAL_VISION_ERROR_NONE && MaterialVision_StateGet() == MATERIAL_VISION_STATE_IDLE));

  if (ArmBusy) OledUi_LastTaskIsMaterial = 0U;
  else if (MaterialBusy) OledUi_LastTaskIsMaterial = 1U;
  if (NewTask || (Stopped && OledUi_TaskError == 0U) || ErrorCleared)
  {
    OledUi_TaskError = 0U;
    (void)strcpy(OledUi_Error, "None");
  }
  if (!OledUi_TaskError && ArmError != OledUi_LastArmError && ArmError != ARM_VISION_ERROR_NONE)
  {
    (void)strcpy(OledUi_Error, OledUi_ArmErrorName(ArmError));
    OledUi_TaskError = 1U;
  }
  if (!OledUi_TaskError && MaterialError != OledUi_LastMaterialError && MaterialError != MATERIAL_VISION_ERROR_NONE)
  {
    (void)strcpy(OledUi_Error, OledUi_MaterialErrorName(MaterialError));
    OledUi_TaskError = 2U;
  }
  OledUi_LastArmBusy = ArmBusy;
  OledUi_LastMaterialBusy = MaterialBusy;
  OledUi_LastArmError = ArmError;
  OledUi_LastMaterialError = MaterialError;
}

static const char *OledUi_TaskName(void)
{
  if (MaterialVision_IsBusy())
  {
    if (MaterialVision_IsCalibrating()) return "Calibrating";
    switch (MaterialVision_StateGet())
    {
      case MATERIAL_VISION_STATE_SEARCH: return "Material Search";
      default: return "Material Align";
    }
  }
  if (ArmVision_IsBusy()) return ArmVision_IsCalibrating() ? "Calibrating" : "Arm Align";
  if (OledUi_TaskError) return "Error";
  if ((OledUi_LastTaskIsMaterial && MaterialVision_StateGet() == MATERIAL_VISION_STATE_ALIGNED) ||
      (!OledUi_LastTaskIsMaterial && strcmp(ArmVision_StateNameGet(), "ALIGNED") == 0)) return "Aligned";
  return "Idle";
}

static void OledUi_OverviewBuild(const Camera_SnapshotTypeDef *Camera, const char *Vision)
{
  HWT101_Angle_t Angle;
  uint8_t Color = OledUi_CodeReady ? (uint8_t)(OledUi_TaskCode[OledUi_SelectedIndex] - '0') : 0U;

  (void)strcpy(OledUi_Lines[0], "1/4 Overview Test");
  (void)strcpy(OledUi_Lines[1], OledUi_CodeReady ? OledUi_TaskCode : "Code:Waiting");
  if (OledUi_CodeReady)
    (void)snprintf(OledUi_Lines[2], sizeof(OledUi_Lines[2]), "Item:%u Color:%s",
                   (unsigned)(OledUi_SelectedIndex + 1U), OledUi_ColorName(Color));
  else (void)strcpy(OledUi_Lines[2], "Item:- Color:None");
  (void)snprintf(OledUi_Lines[3], sizeof(OledUi_Lines[3]), "Task:%s", OledUi_TaskName());
  (void)snprintf(OledUi_Lines[4], sizeof(OledUi_Lines[4]), "Usb:%s Vis:%s",
                 Camera->UsbConfigured ? "On" : "Off", Vision);
  if (!HWT101_Angle_Get(&Angle)) (void)strcpy(OledUi_Lines[5], "Yaw:--- Wait");
  else if (!HWT101_Angle_Is_Fresh(&Angle, HWT101_DATA_FRESH_MS))
    (void)strcpy(OledUi_Lines[5], "Yaw:--- Stale");
  else if (Angle.yaw >= -180.0f && Angle.yaw <= 180.0f)
    (void)snprintf(OledUi_Lines[5], sizeof(OledUi_Lines[5]), "Yaw:%+.1f Ok", (double)Angle.yaw);
  else (void)strcpy(OledUi_Lines[5], "Yaw:--- Range");
  (void)strcpy(OledUi_Lines[7], "2:Run 3:Sel 4<5>");
}

static void OledUi_VisionBuild(const Camera_SnapshotTypeDef *Camera, Camera_VisualStateTypeDef State,
                                const char *Vision, const char *Age)
{
  uint8_t Calibrated = OledUi_LastTaskIsMaterial ? MaterialVision_IsCalibrated() : ArmVision_IsCalibrated();

  (void)strcpy(OledUi_Lines[0], "2/4 Vision Test");
  if (Camera->RequestActive)
    (void)snprintf(OledUi_Lines[1], sizeof(OledUi_Lines[1]), "Target:%u %s",
                   (unsigned)Camera->RequestTarget, OledUi_ColorName(Camera->RequestTarget));
  else (void)strcpy(OledUi_Lines[1], "Target:None");
  if (State == CAMERA_VIS_OK)
  {
    (void)snprintf(OledUi_Lines[2], sizeof(OledUi_Lines[2]), "Cx:%04u Cy:%04u",
                   (unsigned)Camera->Data.CX, (unsigned)Camera->Data.CY);
    (void)snprintf(OledUi_Lines[3], sizeof(OledUi_Lines[3]), "Dx:%+04d Dy:%+04d",
                   (int)Camera->Data.DX, (int)Camera->Data.DY);
  }
  else
  {
    (void)strcpy(OledUi_Lines[2], "Cx:---- Cy:----");
    (void)strcpy(OledUi_Lines[3], "Dx:---- Dy:----");
  }
  (void)snprintf(OledUi_Lines[4], sizeof(OledUi_Lines[4]), "Age:%sms Vis:%s", Age, Vision);
  (void)snprintf(OledUi_Lines[5], sizeof(OledUi_Lines[5]), "Ref:%s Cal:%s",
                 ArmVision_IsReferenceValid() ? "Ok" : "No", Calibrated ? "Ok" : "No");
  (void)strcpy(OledUi_Lines[7], "4:Prev 5:Next");
}

static void OledUi_LinkBuild(const Camera_SnapshotTypeDef *Camera, const char *Vision, const char *Age)
{
  HWT101_Status_t Imu;
  TJC_Status_t Screen;
  char First[6], Second[6];

  (void)HWT101_Status_Get(&Imu);
  (void)TJC_Status_Get(&Screen);
  (void)strcpy(OledUi_Lines[0], "3/4 Link Test");
  OledUi_CountText(First, Camera->RxByteCount);
  (void)snprintf(OledUi_Lines[1], sizeof(OledUi_Lines[1]), "Usb:%s Rx:%s", Camera->UsbConfigured ? "On" : "Off", First);
  (void)snprintf(OledUi_Lines[2], sizeof(OledUi_Lines[2]), "Vis:%s Age:%sms", Vision, Age);
  OledUi_CountText(First, Imu.i2c_error_count);
  (void)snprintf(OledUi_Lines[3], sizeof(OledUi_Lines[3]), "I2c:%s Err:%s", HWT101_Is_Ready() ? "Ok" : "Off", First);
  OledUi_CountText(First, Camera->ChecksumErrorCount);
  OledUi_CountText(Second, Camera->OverflowCount);
  (void)snprintf(OledUi_Lines[4], sizeof(OledUi_Lines[4]), "Sum:%s Drop:%s", First, Second);
  OledUi_CountText(First, Screen.tx_count);
  OledUi_CountText(Second, Screen.rx_frame_count);
  /* T/R分别是发送完成/完整接收帧数。省掉x，保留两个9999+计数且不超21列。 */
  (void)snprintf(OledUi_Lines[5], sizeof(OledUi_Lines[5]), "Screen:T%s R%s", First, Second);
  (void)strcpy(OledUi_Lines[7], "4:Prev 5:Next");
}

static void OledUi_ServoBuild(void)
{
  (void)strcpy(OledUi_Lines[0], "4/4 Servo Test");
  (void)snprintf(OledUi_Lines[1], sizeof(OledUi_Lines[1]), "Signal:%s",
                 OledUi_ServoActive ? "On" : "Off");
  (void)snprintf(OledUi_Lines[2], sizeof(OledUi_Lines[2]), "Set:%uus",
                 (unsigned)OledUi_ServoPulseUs);
  (void)strcpy(OledUi_Lines[3], "2:Open 3:Catch");
  (void)strcpy(OledUi_Lines[4], "Open:500us");
  (void)strcpy(OledUi_Lines[5], "Catch:700us");
  (void)snprintf(OledUi_Lines[7], sizeof(OledUi_Lines[7]), "2:- 3:+ 4:Exit 5:%s",
                 OledUi_ServoActive ? "Off" : "On");
}

static void OledUi_LinesBuild(uint32_t Now)
{
  Camera_SnapshotTypeDef Camera;
  Camera_VisualStateTypeDef State;
  const char *Vision;
  char Age[6];

  Camera_SnapshotGet(&Camera);
  State = Camera_VisualStateGet(&Camera);
  Vision = Camera_VisualStateNameGet(State);
  if (Camera.RequestActive && Camera.HasFrame) OledUi_CountText(Age, Now - Camera.LastFrameTick);
  else (void)strcpy(Age, "---");
  switch (OledUi_Page)
  {
    case OLED_UI_OVERVIEW: OledUi_OverviewBuild(&Camera, Vision); break;
    case OLED_UI_VISION: OledUi_VisionBuild(&Camera, State, Vision, Age); break;
    case OLED_UI_LINK: OledUi_LinkBuild(&Camera, Vision, Age); break;
    default: OledUi_ServoBuild(); break;
  }
  (void)snprintf(OledUi_Lines[6], sizeof(OledUi_Lines[6]), "Err:%s", OledUi_Error);
}

void OledUi_Init(I2C_HandleTypeDef *I2c)
{
  OledUi_I2c = I2c;
  OledUi_Page = OLED_UI_OVERVIEW;
  OledUi_ServoPulseUs = OLED_UI_SERVO_OPEN_US;
  OledUi_ServoActive = 0U;
  OledUi_CodeReady = OledUi_SelectedIndex = 0U;
  OledUi_LastArmBusy = OledUi_LastMaterialBusy = OledUi_TaskError = 0U;
  OledUi_LastTaskIsMaterial = 0U;
  OledUi_LastArmError = ARM_VISION_ERROR_NONE;
  OledUi_LastMaterialError = MATERIAL_VISION_ERROR_NONE;
  (void)strcpy(OledUi_Error, "None");
  (void)memset(OledUi_Sent, 0, sizeof(OledUi_Sent));
  OledUi_DirtyRows = 0xFFU;
  OledUi_Rebuild = 1U;
  /* Register RAM state only; first idle Process performs optional hardware I/O. */
  OledUi_Ready = 0U;
  OledUi_RetryPending = 0U;
  OledUi_LastFailureTick = HAL_GetTick();
}

void OledUi_TaskCodeSet(const char *TaskCode, uint8_t SelectedIndex)
{
  OledUi_CodeReady = (TaskCode != NULL && strlen(TaskCode) == 15U && SelectedIndex < 3U);
  if (OledUi_CodeReady)
  {
    (void)memcpy(OledUi_TaskCode, TaskCode, sizeof(OledUi_TaskCode));
    OledUi_SelectedIndex = SelectedIndex;
  }
  OledUi_Rebuild = 1U;
}

uint8_t OledUi_KeyHandle(KeyEvent_t Key)
{
  if (OledUi_Page == OLED_UI_SERVO && Key == KEY_EVENT_PE5)
  {
    if (OledUi_ServoActive)
      Arm_GripperSignalOff();
    else if (!ChassisMotion_IsBusy() && !ChassisRoute_IsBusy())
      Arm_GripperDutySet((float)OledUi_ServoPulseUs / 200.0f);
    else return 1U;
    OledUi_ServoActive = (uint8_t)!OledUi_ServoActive;
    OledUi_Rebuild = 1U;
    return 1U;
  }
  if (Key == KEY_EVENT_PE4 || Key == KEY_EVENT_PE5)
  {
    if (OledUi_Page == OLED_UI_SERVO && OledUi_ServoActive)
    {
      Arm_GripperSignalOff();
      OledUi_ServoActive = 0U;
    }
    OledUi_Page = (OledUi_PageTypeDef)((OledUi_Page +
                  ((Key == KEY_EVENT_PE5) ? 1U : OLED_UI_PAGE_COUNT - 1U)) % OLED_UI_PAGE_COUNT);
    OledUi_DirtyRows = 0xFFU;
    OledUi_Rebuild = 1U;
    return 1U;
  }
  if (OledUi_Page == OLED_UI_SERVO &&
      (Key == KEY_EVENT_PE2 || Key == KEY_EVENT_PE3))
  {
    OledUi_ServoPulseUs = (Key == KEY_EVENT_PE2) ?
                           OLED_UI_SERVO_OPEN_US : OLED_UI_SERVO_CATCH_US;
    if (OledUi_ServoActive)
      Arm_GripperDutySet((float)OledUi_ServoPulseUs / 200.0f);
    OledUi_Rebuild = 1U;
    return 1U;
  }
  return (OledUi_Page != OLED_UI_OVERVIEW && (Key == KEY_EVENT_PE2 || Key == KEY_EVENT_PE3)) ? 1U : 0U;
}

void OledUi_NoticeSet(const char *Reason)
{
  if (Reason == NULL)
  {
    OledUi_TaskError = 0U;
    (void)strcpy(OledUi_Error, "None");
  }
  else if (!OledUi_TaskError)
    (void)snprintf(OledUi_Error, sizeof(OledUi_Error), "%.16s", Reason);
  OledUi_Rebuild = 1U;
}

void OledUi_Process(void)
{
  uint32_t Now = HAL_GetTick();
  uint8_t Row;

  OledUi_ErrorUpdate();
  if (OledUi_Rebuild || Now - OledUi_LastRefreshTick >= OLED_UI_REFRESH_MS)
  {
    OledUi_LinesBuild(Now);
    OledUi_LastRefreshTick = Now;
    OledUi_Rebuild = 0U;
  }
  if (OledUi_RetryPending && Now - OledUi_LastFailureTick < OLED_UI_RETRY_MS) return;
  if (!OledUi_Ready)
  {
    OledUi_Ready = (OLED_Init(OledUi_I2c) == HAL_OK) ? 1U : 0U;
    OledUi_RetryPending = !OledUi_Ready;
    OledUi_LastFailureTick = HAL_GetTick();
    return; /* 初始化后下一圈再发第一行。 */
  }
  for (Row = 0U; Row < OLED_UI_ROWS; Row++)
  {
    if ((OledUi_DirtyRows & (1U << Row)) || strcmp(OledUi_Lines[Row], OledUi_Sent[Row]) != 0)
    {
      if (OLED_Line_Show(Row, OledUi_Lines[Row]) == HAL_OK)
      {
        (void)strcpy(OledUi_Sent[Row], OledUi_Lines[Row]);
        OledUi_DirtyRows &= (uint8_t)~(1U << Row);
        OledUi_RetryPending = 0U;
      }
      else
      {
        OledUi_RetryPending = 1U;
        OledUi_LastFailureTick = HAL_GetTick();
      }
      return;
    }
  }
}
