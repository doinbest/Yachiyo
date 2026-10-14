/* Real unified owner and calibration service with the normal device fixture. */
#define GRAB_REAL_CALIBRATION_SERVICE
#define main legacy_grab_main
#include "grab_task_test.c"
#undef main

static void wait_calibration(GrabCalibration_StateTypeDef state,unsigned limit,int dx,int dy)
{
  while(GrabCalibration_StateGet()!=state && limit--)step(true,0,dx,dy);
  assert(GrabCalibration_StateGet()==state);
}
int main(void)
{
  reset();GrabCalibration_Init();configure(true);setting("travel_mm","0");
  assert(GrabTask_CalibrationReferenceSet());
  assert(GrabTask_CalibrationStart(CAMERA_COLOR_BLUE));
  run_to(GRAB_CALIBRATION,200,0);
  assert(!positions && !base_moves && !openings && !closures);
  wait_calibration(GRAB_CAL_STATE_CHASSIS_MOVE,12,0,0);
  const uint8_t dirs[]={CHASSIS_MOTOR_FRONT_LEFT_FORWARD_DIR,CHASSIS_MOTOR_REAR_LEFT_FORWARD_DIR,
    CHASSIS_MOTOR_REAR_RIGHT_FORWARD_DIR,CHASSIS_MOTOR_FRONT_RIGHT_FORWARD_DIR};
  int64_t units=(int64_t)lroundf(12*65536*CHASSIS_MOTOR_TO_WHEEL_RATIO/
    (3.14159265358979323846f*CHASSIS_WHEEL_DIAMETER_MM));
  for(unsigned i=0;i<4;i++)wheel_position[i]=dirs[i]?-units:units;
  wait_calibration(GRAB_CAL_STATE_COLLECT,40,12,0);
  wait_calibration(GRAB_CAL_STATE_CHASSIS_MOVE,12,12,0);
  for(unsigned i=0;i<4;i++)wheel_position[i]=0;
  wait_calibration(GRAB_CAL_STATE_COLLECT,40,0,0);
  wait_calibration(GRAB_CAL_STATE_X_ACK,12,0,0);
  wait_calibration(GRAB_CAL_STATE_COLLECT,40,0,10);
  wait_calibration(GRAB_CAL_STATE_X_ACK,12,0,10);
  run_to(GRAB_COMPLETE,100,0);
  assert(GrabCalibration_IsCalibrated() && !GrabTask_IsBusy());
  assert(!strcmp(status().result,"calibrated"));
  assert(fabsf(Config[C_J00]-1)<.01f && Config[C_J11]==1 && Config[C_J12]>20);
  assert(!base_moves && !openings && !closures && positions==2);
  float measured=Config[C_J00];
  camera_tx_result=HAL_ERROR;
  assert(GrabTask_CalibrationStart(CAMERA_COLOR_BLUE));run_to(GRAB_ERROR,250,0);
  assert(!strcmp(status().reason,"camera_tx") && Config[C_J00]==measured && GrabCalibration_IsCalibrated());
  assert(!base_moves && !openings && !closures && positions==2);
  camera_tx_result=HAL_OK;
  assert(GrabTask_CalibrationStart(CAMERA_COLOR_BLUE));
  run_to(GRAB_CALIBRATION,200,0);
  wait_calibration(GRAB_CAL_STATE_CHASSIS_MOVE,12,0,0);
  GrabTask_Stop();run_to(GRAB_IDLE,200,0);
  assert(GrabCalibration_IsCalibrated() && Config[C_J00]==measured && GrabTask_CalibrationReferenceValid());
  physical[0]=Config[C_X_MAX];
  unsigned attempts=positions;
  assert(GrabTask_CalibrationStart(CAMERA_COLOR_BLUE));
  run_to(GRAB_ERROR,250,0);
  assert(!strcmp(status().reason,"calibration_x_limit") && positions==attempts && Config[C_J00]==measured);
  puts("grab_calibration_task_test: actual response/shared J, cancellation retention, X boundary PASS");
  return 0;
}
