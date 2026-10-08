/* Real unified owner and calibration service with the normal device fixture. */
#define GRAB_REAL_MATERIAL_SERVICE
#define main legacy_grab_main
#include "grab_task_test.c"
#undef main

static void wait_material(MaterialVision_StateTypeDef state,unsigned limit,int dx,int dy)
{
  while(MaterialVision_StateGet()!=state && limit--)step(true,0,dx,dy);
  assert(MaterialVision_StateGet()==state);
}
int main(void)
{
  material_search_uses_shared_alignment();
  reset();MaterialVision_Init();configure(true);setting("travel_mm","0");
  assert(MaterialVision_ReferenceSet()==MATERIAL_VISION_RESULT_OK);
  assert(MaterialVision_CalibrationStart(CAMERA_COLOR_BLUE)==MATERIAL_VISION_RESULT_OK);
  assert(MaterialVision_StateGet()==MATERIAL_VISION_STATE_COLLECT && MaterialVision_ErrorGet()==MATERIAL_VISION_ERROR_NONE);
  run_to(GRAB_CALIBRATION,200,0);
  assert(!positions && !base_moves && !openings && !closures);
  wait_material(MATERIAL_VISION_STATE_CHASSIS_MOVE,12,0,0);
  const uint8_t dirs[]={CHASSIS_MOTOR_FRONT_LEFT_FORWARD_DIR,CHASSIS_MOTOR_REAR_LEFT_FORWARD_DIR,
    CHASSIS_MOTOR_REAR_RIGHT_FORWARD_DIR,CHASSIS_MOTOR_FRONT_RIGHT_FORWARD_DIR};
  int64_t units=(int64_t)lroundf(12*65536*CHASSIS_MOTOR_TO_WHEEL_RATIO/
    (3.14159265358979323846f*CHASSIS_WHEEL_DIAMETER_MM));
  for(unsigned i=0;i<4;i++)wheel_position[i]=dirs[i]?-units:units;
  wait_material(MATERIAL_VISION_STATE_COLLECT,40,12,0);
  wait_material(MATERIAL_VISION_STATE_CHASSIS_MOVE,12,12,0);
  for(unsigned i=0;i<4;i++)wheel_position[i]=0;
  wait_material(MATERIAL_VISION_STATE_COLLECT,40,0,0);
  wait_material(MATERIAL_VISION_STATE_X_ACK,12,0,0);
  wait_material(MATERIAL_VISION_STATE_COLLECT,40,0,10);
  wait_material(MATERIAL_VISION_STATE_X_ACK,12,0,10);
  run_to(GRAB_COMPLETE,100,0);
  assert(MaterialVision_IsCalibrated() && !MaterialVision_IsBusy());
  assert(!strcmp(status().result,"calibrated"));
  assert(fabsf(Config[C_J00]-1)<.01f && Config[C_J11]==1 && Config[C_J12]>20);
  assert(!base_moves && !openings && !closures && positions==2);
  float measured=Config[C_J00];
  assert(MaterialVision_CalibrationStart(CAMERA_COLOR_BLUE)==MATERIAL_VISION_RESULT_OK);
  assert(MaterialVision_StateGet()==MATERIAL_VISION_STATE_COLLECT && MaterialVision_ErrorGet()==MATERIAL_VISION_ERROR_NONE);
  run_to(GRAB_CALIBRATION,200,0);
  wait_material(MATERIAL_VISION_STATE_CHASSIS_MOVE,12,0,0);
  MaterialVision_Stop();run_to(GRAB_IDLE,200,0);
  assert(MaterialVision_IsCalibrated() && Config[C_J00]==measured && MaterialVision_IsReferenceValid());
  physical[0]=Config[C_X_MAX];
  unsigned attempts=positions;
  assert(MaterialVision_CalibrationStart(CAMERA_COLOR_BLUE)==MATERIAL_VISION_RESULT_OK);
  run_to(GRAB_ERROR,250,0);
  assert(!strcmp(status().reason,"calibration_x_limit") && positions==attempts && Config[C_J00]==measured);
  puts("grab_material_test: search/shared align, actual response/shared J, cancellation retention PASS");
  return 0;
}
