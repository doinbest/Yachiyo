#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include "../template/App/ArmVision.c"
static bool route_busy, motion_busy;
bool ChassisRoute_IsBusy(void) { return route_busy; }
bool ChassisMotion_IsBusy(void) { return motion_busy; }
static uint32_t tick;
static uint8_t peer_busy, motor_busy;
static unsigned requests, moves, motor_stops;
static HAL_StatusTypeDef camera_result = HAL_OK;
static uint8_t configured = 1;
static Camera_DataTypeDef camera_data;
uint32_t HAL_GetTick(void) { return tick; }
uint8_t MaterialVision_IsBusy(void) { return peer_busy; }
uint8_t MechanicalArm_IsBusy(void) { return motor_busy; }
void Camera_SnapshotGet(Camera_SnapshotTypeDef *out)
{ memset(out, 0, sizeof(*out)); out->UsbConfigured = configured; out->RequestActive = 1;
  out->Data = camera_data; out->TargetValid = out->HasValidData = camera_data.Sequence != 0; }
HAL_StatusTypeDef Camera_MaterialStart(Camera_ColorTypeDef color) { (void)color; requests++; return camera_result; }
void Camera_RequestStop(void) {}
MechanicalArm_ResultTypeDef MechanicalArm_Stop(MechanicalArm_AxisTypeDef axis) { (void)axis; motor_stops++; return MECHANICAL_ARM_RESULT_NONE; }
MechanicalArm_ResultTypeDef MechanicalArm_StateRead(MechanicalArm_AxisTypeDef axis) { (void)axis; return MECHANICAL_ARM_RESULT_NONE; }
MechanicalArm_ResultTypeDef MechanicalArm_PositionEx(MechanicalArm_AxisTypeDef axis, int32_t pulses,
    uint16_t rpm, uint8_t acceleration, MechanicalArm_PositionModeTypeDef mode)
{ (void)axis; (void)pulses; (void)rpm; (void)acceleration; (void)mode; moves++; return MECHANICAL_ARM_RESULT_NONE; }
int main(void)
{
  assert(ARM_VISION_CALIBRATION_MATERIAL == 2);
  assert(ARM_VISION_JOB_PICK_MATERIAL == 2);
  for (unsigned owner = 0; owner < 2; owner++)
  {
    ArmVision_Init(); route_busy = owner == 0; motion_busy = owner == 1;
    assert(ArmVision_ReferenceSet() == ARM_VISION_RESULT_BUSY);
    assert(ArmVision_MaterialCalibrationStart(CAMERA_COLOR_BLUE) == ARM_VISION_RESULT_BUSY);
    assert(ArmVision_MaterialStart(CAMERA_COLOR_BLUE, ARM_VISION_JOB_ALIGN_ONLY) == ARM_VISION_RESULT_BUSY);
    assert(!requests && !moves && !ArmVision_IsBusy());
    ArmVision_Stop(); assert(!ArmVision_IsBusy());
  }
  route_busy = motion_busy = false;
  ArmVision_Init(); assert(ArmVision_ReferenceSet() == ARM_VISION_RESULT_OK);
  assert(ArmVision_MaterialCalibrationStart(CAMERA_COLOR_BLUE) == ARM_VISION_RESULT_OK);
  assert(!strcmp(ArmVision_CalibrationSourceNameGet(), "material"));
  route_busy = true; ArmVision_Stop();
  assert(!ArmVision_IsBusy() && motor_stops == 1);
  route_busy = false; requests = 0;
  ArmVision_Init(); peer_busy = 1;
  assert(ArmVision_ReferenceSet() == ARM_VISION_RESULT_BUSY);
  assert(ArmVision_MaterialCalibrationStart(CAMERA_COLOR_BLUE) == ARM_VISION_RESULT_BUSY);
  assert(ArmVision_MaterialStart(CAMERA_COLOR_BLUE, ARM_VISION_JOB_ALIGN_ONLY) == ARM_VISION_RESULT_BUSY);
  assert(!requests && !moves);
  peer_busy = 0; assert(ArmVision_ReferenceSet() == ARM_VISION_RESULT_OK);
  assert(ArmVision_MaterialCalibrationStart(CAMERA_COLOR_BLUE) == ARM_VISION_RESULT_OK);
  tick = ARM_VISION_FIRST_DATA_TIMEOUT_MS + 1; ArmVision_Process();
  assert(ArmVision_ErrorGet() == ARM_VISION_ERROR_CAMERA_FIRST_TIMEOUT);
  ArmVision_Init(); ArmVision_ReferenceSet();
  camera_result = HAL_BUSY;
  assert(ArmVision_MaterialCalibrationStart(CAMERA_COLOR_BLUE) == ARM_VISION_RESULT_ERROR);
  assert(!strcmp(ArmVision_ErrorNameGet(), "camera_busy"));
  camera_result = HAL_ERROR; configured = 0;
  assert(ArmVision_MaterialCalibrationStart(CAMERA_COLOR_BLUE) == ARM_VISION_RESULT_ERROR);
  assert(!strcmp(ArmVision_ErrorNameGet(), "usb_off"));
  configured = 1; camera_result = HAL_OK; ArmVision_MaterialCalibrationStart(CAMERA_COLOR_BLUE);
  camera_data.Sequence = 1; camera_data.Tick = tick; ArmVision_Process();
  assert(ArmVision_SampleIndex == 1); ArmVision_Process(); assert(ArmVision_SampleIndex == 1);
  tick += ARM_VISION_DATA_STALE_TIMEOUT_MS + 1; ArmVision_Process();
  assert(ArmVision_ErrorGet() == ARM_VISION_ERROR_CAMERA_STALE);
  puts("arm_vision_test: OK"); return 0;
}
