#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "oled_ui.h"
#include "Camera.h"
#include "ArmVision.h"
#include "MaterialVision.h"
#include "hwt101_i2c.h"
#include "tjc_screen.h"
static uint32_t tick;
static char rows[8][22];
static unsigned sends, init_calls;
static HAL_StatusTypeDef oled_result = HAL_OK;
static Camera_SnapshotTypeDef camera;
static HWT101_Angle_t angle;
static HWT101_Status_t imu;
static TJC_Status_t screen;
static uint8_t arm_busy, material_busy;
static uint8_t arm_calibrated = 1;
static uint8_t material_calibrated;
static MaterialVision_StateTypeDef material_state;
static ArmVision_ErrorTypeDef arm_error;
static float servo_duty;
static unsigned servo_sets, servo_offs;
static uint8_t chassis_busy;
bool ChassisMotion_IsBusy(void) { return chassis_busy != 0U; }
bool ChassisRoute_IsBusy(void) { return false; }
void Arm_GripperDutySet(float duty) { servo_duty = duty; servo_sets++; }
void Arm_GripperSignalOff(void) { servo_duty = 0.0f; servo_offs++; }
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef OLED_Init(I2C_HandleTypeDef *i2c) { (void)i2c; init_calls++; return oled_result; }
HAL_StatusTypeDef OLED_Line_Show(uint8_t row, const char *text)
{ assert(row < 8 && strlen(text) <= 21); sends++; if (oled_result == HAL_OK) strcpy(rows[row], text); return oled_result; }
void Camera_SnapshotGet(Camera_SnapshotTypeDef *out) { *out = camera; }
/* Camera visual-state calculation is linked from its real source by the runner. */
uint8_t CDC_IsConfigured_FS(void) { return camera.UsbConfigured; }
uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t length) { (void)data; (void)length; return 0; }
uint8_t ArmVision_IsBusy(void) { return arm_busy; }
uint8_t ArmVision_IsCalibrating(void) { return 0; }
uint8_t ArmVision_IsReferenceValid(void) { return 1; }
uint8_t ArmVision_IsCalibrated(void) { return arm_calibrated; }
ArmVision_ErrorTypeDef ArmVision_ErrorGet(void) { return arm_error; }
const char *ArmVision_StateNameGet(void) { return arm_busy ? "COLLECT" : "IDLE"; }
uint8_t MaterialVision_IsBusy(void) { return material_busy; }
uint8_t MaterialVision_IsCalibrating(void) { return 0; }
uint8_t MaterialVision_IsCalibrated(void) { return material_calibrated; }
MaterialVision_StateTypeDef MaterialVision_StateGet(void) { return material_state; }
MaterialVision_ErrorTypeDef MaterialVision_ErrorGet(void) { return MATERIAL_VISION_ERROR_NONE; }
bool HWT101_Angle_Get(HWT101_Angle_t *out) { *out = angle; return angle.update_count != 0; }
bool HWT101_Angle_Is_Fresh(const HWT101_Angle_t *value, uint32_t age)
{ return value->update_count && tick - value->last_update_ms <= age; }
bool HWT101_Is_Ready(void) { return imu.valid_read_count != 0; }
bool HWT101_Status_Get(HWT101_Status_t *out) { *out = imu; return true; }
bool TJC_Status_Get(TJC_Status_t *out) { *out = screen; return true; }
static void refresh(void)
{
  unsigned i; tick += 200;
  for (i = 0; i < 8; i++) { unsigned before = sends; OledUi_Process(); assert(sends - before <= 1); }
}
int main(void)
{
  I2C_HandleTypeDef i2c = {0}; unsigned before;
  OledUi_Init(&i2c); refresh();
  assert(!strcmp(rows[0], "1/4 Overview Test"));
  assert(!strcmp(rows[1], "Code:Waiting"));
  assert(!OledUi_KeyHandle(KEY_EVENT_PE2));
  OledUi_TaskCodeSet("123+231+312+321", 1); refresh();
  assert(!strcmp(rows[1], "123+231+312+321"));
  assert(!strcmp(rows[2], "Item:2 Color:Blue"));
  OledUi_KeyHandle(KEY_EVENT_PE5); refresh();
  assert(!strcmp(rows[0], "2/4 Vision Test"));
  assert(OledUi_KeyHandle(KEY_EVENT_PE2) && OledUi_KeyHandle(KEY_EVENT_PE3));
  assert(!strcmp(rows[1], "Target:None"));
  camera.RequestActive = camera.UsbConfigured = camera.HasFrame = camera.HasValidData = camera.TargetValid = 1;
  camera.RequestTarget = 3; camera.LastFrameTick = tick;
  camera.Data.CX = camera.Data.CY = 65535; camera.Data.DX = -32768; camera.Data.DY = 32767;
  refresh(); assert(strstr(rows[2], "65535")); assert(strstr(rows[3], "-32768"));
  camera.TargetValid = 0; camera.LastFrameTick = tick;
  refresh(); assert(strstr(rows[4], "Lost")); assert(!strcmp(rows[2], "Cx:---- Cy:----"));
  tick += 1201; refresh(); assert(strstr(rows[4], "Stale"));
  camera.UsbConfigured = 0; refresh(); assert(strstr(rows[4], "Off"));
  OledUi_KeyHandle(KEY_EVENT_PE5);
  camera.RxByteCount = camera.ChecksumErrorCount = camera.OverflowCount = UINT32_MAX;
  screen.tx_count = screen.rx_frame_count = UINT32_MAX; imu.i2c_error_count = UINT32_MAX;
  refresh(); assert(!strcmp(rows[0], "3/4 Link Test")); assert(strstr(rows[1], "9999+"));
  arm_error = ARM_VISION_ERROR_MOTOR_ARRIVAL_TIMEOUT; refresh();
  assert(!strcmp(rows[6], "Err:Motor arrival"));
  OledUi_KeyHandle(KEY_EVENT_PE5); refresh();
  assert(!strcmp(rows[0], "4/4 Servo Test"));
  assert(!strcmp(rows[1], "Signal:Off") && servo_sets == 0);
  assert(OledUi_KeyHandle(KEY_EVENT_PE2) && servo_sets == 0);
  refresh(); assert(!strcmp(rows[2], "Set:500us"));
  chassis_busy = 1U; OledUi_KeyHandle(KEY_EVENT_PE5); assert(servo_sets == 0);
  chassis_busy = 0U; OledUi_KeyHandle(KEY_EVENT_PE5);
  assert(servo_sets == 1 && servo_duty == 2.5f);
  OledUi_KeyHandle(KEY_EVENT_PE3); assert(servo_duty == 4.0f);
  OledUi_KeyHandle(KEY_EVENT_PE5); assert(servo_offs == 1 && servo_duty == 0.0f);
  before = servo_sets;
  OledUi_KeyHandle(KEY_EVENT_PE2);
  refresh(); assert(!strcmp(rows[2], "Set:500us") && servo_sets == before);
  OledUi_KeyHandle(KEY_EVENT_PE3);
  refresh(); assert(!strcmp(rows[2], "Set:800us") && servo_sets == before);
  OledUi_KeyHandle(KEY_EVENT_PE5); assert(servo_duty == 4.0f);
  OledUi_KeyHandle(KEY_EVENT_PE4); assert(servo_offs == 2 && servo_duty == 0.0f);
  OledUi_KeyHandle(KEY_EVENT_PE4);
  OledUi_KeyHandle(KEY_EVENT_PE4);
  refresh(); assert(!strcmp(rows[0], "1/4 Overview Test"));
  assert(!strcmp(rows[6], "Err:Motor arrival"));
  OledUi_NoticeSet("Busy"); refresh(); assert(!strcmp(rows[6], "Err:Motor arrival"));
  arm_error = ARM_VISION_ERROR_NONE; arm_busy = 1; refresh(); assert(!strcmp(rows[6], "Err:None"));
  before = sends; refresh(); assert(sends == before); /* Unchanged rows are not sent. */
  oled_result = HAL_ERROR; OledUi_KeyHandle(KEY_EVENT_PE4); OledUi_Process(); before = sends;
  tick += 999; OledUi_Process(); assert(sends == before);
  tick++; OledUi_Process(); assert(sends == before + 1);
  oled_result = HAL_OK; tick += 1000; refresh(); assert(!strcmp(rows[0], "4/4 Servo Test"));
  oled_result = HAL_ERROR; OledUi_Init(&i2c); before = init_calls;
  OledUi_Process(); assert(init_calls == before);
  tick += 1000; OledUi_Process(); assert(init_calls == before + 1);
  oled_result = HAL_OK; OledUi_Init(&i2c);
  arm_busy = 0; arm_calibrated = 0; material_busy = 1; material_state = MATERIAL_VISION_STATE_COLLECT;
  OledUi_KeyHandle(KEY_EVENT_PE5); refresh(); assert(strstr(rows[5], "Cal:No"));
  material_busy = 0; material_state = MATERIAL_VISION_STATE_ALIGNED; material_calibrated = 1;
  refresh(); assert(strstr(rows[5], "Cal:Ok"));
  /* A failed retry must not overwrite a retained task error. */
  arm_error = ARM_VISION_ERROR_MOTOR_ARRIVAL_TIMEOUT; refresh();
  arm_error = ARM_VISION_ERROR_CAMERA_TX; refresh();
  assert(!strcmp(rows[6], "Err:Motor arrival"));
  puts("oled_ui_test: OK"); return 0;
}
