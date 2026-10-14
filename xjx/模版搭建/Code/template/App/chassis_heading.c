#include "chassis_heading.h"
#include "chassis_motion.h"
#include "chassis_localization.h"
#include <math.h>
#include <stdlib.h>
bool ChassisHeading_Apply(float yaw)
{
  ChassisLocalization_Status_t position;
  if(!isfinite(yaw))return false;
  ChassisMotion_HeadingProcess();ChassisLocalization_Process();
  ChassisLocalization_Get(&position);
  uint32_t now=HAL_GetTick();
  if(!position.origin_valid || !position.feedback_valid || !position.speed_valid ||
     now-position.feedback_tick>600U)return false;
  for(unsigned i=0;i<4;i++) {
    const Mecanum_Feedback_t *w=&position.wheels[i];
    if(!w->state_valid || now-w->state_ms>600U || !(w->state_flags&1U) ||
       (w->state_flags&0x0cU) || abs(w->speed_rpm)>1)return false;
  }
  yaw=remainderf(yaw,2*CHASSIS_MODEL_PI);
  if(!ChassisMotion_AnchorCorrect(yaw))return false;
  ChassisLocalization_HeadingSync(yaw);
  return true;
}
