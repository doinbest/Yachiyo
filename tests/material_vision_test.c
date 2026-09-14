/* Run the real state machine with a fake clock, camera and motor replies. */
#include <assert.h>
#include <stdio.h>
#include "../template/App/MaterialVision.c"
static bool route_busy, motion_busy;
bool ChassisRoute_IsBusy(void) { return route_busy; }
bool ChassisMotion_IsBusy(void) { return motion_busy; }

static uint32_t tick;
static Camera_DataTypeDef frame;
static uint8_t peer_busy, motor_busy;
static unsigned forward_moves, x_moves, requests, chassis_stops, motor_stops;
static float last_speed;
static MechanicalArm_ResultTypeDef motor_result;
static uint8_t chassis_ok = 1;
uint32_t HAL_GetTick(void) { return tick; }
uint8_t ArmVision_IsBusy(void) { return peer_busy; }
uint8_t ArmVision_IsReferenceValid(void) { return 1; }
uint8_t ArmVision_DataPeek(Camera_DataTypeDef *data) { *data = frame; return frame.Sequence != 0; }
void Camera_SnapshotGet(Camera_SnapshotTypeDef *out)
{ memset(out, 0, sizeof(*out)); out->Data = frame; out->RequestActive = out->UsbConfigured = 1;
  out->HasFrame = out->TargetValid = out->HasValidData = (frame.Sequence != 0); }
uint8_t MechanicalArm_IsBusy(void) { return motor_busy; }
HAL_StatusTypeDef Camera_MaterialStart(Camera_ColorTypeDef color) { (void)color; requests++; return HAL_OK; }
void Camera_RequestStop(void) {}
bool Mecanum_Velocity_Start(float forward, float left, float yaw)
{ assert(left == 0 && yaw == 0); forward_moves++; last_speed = forward; return chassis_ok != 0; }
void Mecanum_VelocityRefresh_Stop(void) {}
bool Mecanum_Test_Stop(void) { chassis_stops++; return true; }
MechanicalArm_ResultTypeDef MechanicalArm_Stop(MechanicalArm_AxisTypeDef axis) { (void)axis; motor_stops++; return MECHANICAL_ARM_RESULT_NONE; }
MechanicalArm_ResultTypeDef MechanicalArm_Home(MechanicalArm_AxisTypeDef axis, uint8_t mode)
{ (void)axis; (void)mode; return motor_result; }
MechanicalArm_ResultTypeDef MechanicalArm_StateRead(MechanicalArm_AxisTypeDef axis)
{ (void)axis; return motor_result; }
MechanicalArm_ResultTypeDef MechanicalArm_PositionEx(MechanicalArm_AxisTypeDef axis, int32_t pulses,
    uint16_t rpm, uint8_t acceleration, MechanicalArm_PositionModeTypeDef mode)
{ assert(axis == MECHANICAL_ARM_AXIS_X && pulses != 0); (void)rpm; (void)acceleration; (void)mode; x_moves++; return motor_result; }

static void reset(void)
{
  MaterialVision_Init(); tick = 0; memset(&frame, 0, sizeof(frame));
  peer_busy = motor_busy = 0; forward_moves = x_moves = requests = 0;
  chassis_stops = motor_stops = 0;
  chassis_ok = 1; motor_result = MECHANICAL_ARM_RESULT_NONE;
  MaterialVision_XTotal = 0; MaterialVision_ChassisTotal = 0;
  MaterialVision_ForwardIntegral = MaterialVision_ForwardPreviousError = 0;
  MaterialVision_XIntegral = MaterialVision_XPreviousError = 0;
}
static void sample(int16_t dx, int16_t dy)
{
  unsigned i;
  for (i = 0; i < MATERIAL_VISION_SAMPLE_COUNT; i++)
  { frame.DX = dx; frame.DY = dy; frame.Sequence++; frame.Tick = ++tick; MaterialVision_Process(); }
}
static void chassis_finish(uint32_t duration)
{
  tick += duration; MaterialVision_Process();
  assert(MaterialVision_StateGet() == MATERIAL_VISION_STATE_SETTLE);
  tick += MATERIAL_VISION_MOVE_SETTLE_MS; MaterialVision_Process();
}
static void x_finish(void)
{
  MechanicalArm_EventTypeDef event = {0};
  event.Axis = MECHANICAL_ARM_AXIS_X; event.Action = MECHANICAL_ARM_ACTION_POSITION;
  event.Result = MECHANICAL_ARM_RESULT_OK;
  assert(MaterialVision_MotorEventHandle(&event));
  tick += 50; MaterialVision_Process();
  event.Action = MECHANICAL_ARM_ACTION_STATE; event.StateFlags[MECHANICAL_ARM_AXIS_X] = 2;
  assert(MaterialVision_MotorEventHandle(&event));
  tick += MATERIAL_VISION_MOVE_SETTLE_MS; MaterialVision_Process();
}
static void calibration(void)
{
  reset(); assert(MaterialVision_CalibrationStart(CAMERA_COLOR_BLUE) == MATERIAL_VISION_RESULT_OK);
  sample(0, 0); assert(forward_moves == 1);
  /* A frame produced during movement must not enter the settled sample group. */
  frame.Sequence++; frame.DX = 200; frame.DY = 200;
  chassis_finish(1000);
  assert(MaterialVision_StateGet() == MATERIAL_VISION_STATE_COLLECT);
  MaterialVision_Process(); assert(MaterialVision_SampleIndex == 0);
  sample(20, 0); assert(forward_moves == 2); chassis_finish(1000);
  sample(0, 0); assert(x_moves == 1); x_finish();
  assert(MaterialVision_StateGet() == MATERIAL_VISION_STATE_COLLECT);
  sample(0, 10); assert(x_moves == 2); x_finish();
  assert(MaterialVision_IsCalibrated());
  assert(MaterialVision_StateGet() == MATERIAL_VISION_STATE_ALIGNED);
}
static void handoff(void)
{
  const int16_t values[] = {-16, -15, -2, -1, 0, 1, 2, 10, 15, 16};
  unsigned i, j;
  for (i = 0; i < sizeof(values)/sizeof(values[0]); i++)
    for (j = 0; j < sizeof(values)/sizeof(values[0]); j++)
    {
      int16_t dx = values[i], dy = values[j]; reset();
      MaterialVision_Task = MATERIAL_VISION_TASK_ALIGN;
      MaterialVision_CollectStart(MATERIAL_VISION_PHASE_ALIGN);
      sample(dx, dy);
      if (abs(dx) <= 1 && abs(dy) <= 1)
      { assert(!forward_moves && !x_moves); sample(dx, dy); sample(dx, dy); assert(MaterialVision_StateGet() == MATERIAL_VISION_STATE_ALIGNED); }
      else if (abs(dx) > 15 || abs(dy) <= 1)
      { assert(forward_moves == 1 && x_moves == 0); assert(last_speed * dx > 0); }
      else assert(x_moves == 1 && forward_moves == 0);
    }
}
static void variable_duration(void)
{
  reset(); MaterialVision_Task = MATERIAL_VISION_TASK_CALIBRATION;
  assert(MaterialVision_ChassisMoveStart(1, 40)); tick = 1999; MaterialVision_Process();
  assert(MaterialVision_StateGet() == MATERIAL_VISION_STATE_CHASSIS_MOVE);
  tick = 2000; MaterialVision_Process(); assert(MaterialVision_StateGet() == MATERIAL_VISION_STATE_SETTLE);
}
static void busy_and_timeout(void)
{
  reset(); peer_busy = 1;
  assert(MaterialVision_Start(CAMERA_COLOR_BLUE) == MATERIAL_VISION_RESULT_BUSY);
  assert(MaterialVision_CalibrationStart(CAMERA_COLOR_BLUE) == MATERIAL_VISION_RESULT_BUSY);
  assert(requests == 0 && !forward_moves && !x_moves);
  peer_busy = 0; motor_busy = 1;
  assert(MaterialVision_CalibrationStart(CAMERA_COLOR_BLUE) == MATERIAL_VISION_RESULT_BUSY);
  motor_busy = 0; MaterialVision_CalibrationStart(CAMERA_COLOR_BLUE);
  tick = 60000; MaterialVision_Process(); assert(MaterialVision_IsBusy());
  MaterialVision_State = MATERIAL_VISION_STATE_HOME_WAIT; MaterialVision_StartTick = tick;
  tick += MATERIAL_VISION_HOME_TIMEOUT_MS + 1; MaterialVision_Process();
  assert(MaterialVision_StateGet() == MATERIAL_VISION_STATE_ERROR);
}
static void failures(void)
{
  reset(); chassis_ok = 0;
  MaterialVision_Task = MATERIAL_VISION_TASK_ALIGN;
  MaterialVision_CollectStart(MATERIAL_VISION_PHASE_ALIGN); sample(10, 0);
  assert(MaterialVision_ErrorGet() == MATERIAL_VISION_ERROR_CHASSIS_TX);
  reset(); motor_result = MECHANICAL_ARM_RESULT_TX_ERROR;
  MaterialVision_Task = MATERIAL_VISION_TASK_ALIGN;
  MaterialVision_CollectStart(MATERIAL_VISION_PHASE_ALIGN); sample(0, 10);
  assert(MaterialVision_ErrorGet() == MATERIAL_VISION_ERROR_MOTOR_TX);
  reset(); MaterialVision_XTotal = MATERIAL_VISION_MAX_X_TOTAL_PULSES;
  assert(!MaterialVision_XMoveStart(5)); assert(MaterialVision_ErrorGet() == MATERIAL_VISION_ERROR_X_LIMIT);
  reset(); MaterialVision_Task = MATERIAL_VISION_TASK_ALIGN;
  assert(MaterialVision_ForwardVelocityStart(8)); tick = 199; MaterialVision_Process();
  assert(MaterialVision_StateGet() == MATERIAL_VISION_STATE_CHASSIS_MOVE);
  tick = 200; MaterialVision_Process(); assert(MaterialVision_StateGet() == MATERIAL_VISION_STATE_SETTLE);
}
int main(void)
{
  for (unsigned owner = 0; owner < 2; owner++)
  {
    reset(); route_busy = owner == 0; motion_busy = owner == 1;
    assert(MaterialVision_Start(CAMERA_COLOR_BLUE) == MATERIAL_VISION_RESULT_BUSY);
    assert(MaterialVision_CalibrationStart(CAMERA_COLOR_BLUE) == MATERIAL_VISION_RESULT_BUSY);
    assert(!requests && !forward_moves && !x_moves && !MaterialVision_IsBusy());
    assert(!chassis_stops && !motor_stops);
    MaterialVision_Stop(); assert(!MaterialVision_IsBusy());
  }
  route_busy = motion_busy = false;
  reset(); assert(MaterialVision_CalibrationStart(CAMERA_COLOR_BLUE) == MATERIAL_VISION_RESULT_OK);
  chassis_stops = motor_stops = 0;
  route_busy = true; MaterialVision_Stop();
  assert(!MaterialVision_IsBusy() && chassis_stops == 1 && motor_stops == 1);
  route_busy = false;
  calibration(); handoff(); variable_duration(); busy_and_timeout(); failures();
  puts("material_vision_test: OK"); return 0;
}
