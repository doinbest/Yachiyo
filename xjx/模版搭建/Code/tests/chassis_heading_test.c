/* Real motion + localization; reuse only hardware fixtures. */
#define main legacy_motion_main
#define ChassisLocalization_Get FixtureLocalizationGet
#include "chassis_motion_test.c"
#undef ChassisLocalization_Get
#undef main
void ChassisLocalization_Get(ChassisLocalization_Status_t *out);
#include "chassis_heading.h"
static void sample(unsigned t) {
  reading(t,0);
  for(unsigned i=0;i<4;i++) {
    location.wheels[i].position_valid=location.wheels[i].speed_valid=location.wheels[i].state_valid=true;
    location.wheels[i].position_ms=location.wheels[i].speed_ms=location.wheels[i].state_ms=t;
    location.wheels[i].state_flags=1;
  }
  ChassisMotion_HeadingProcess();ChassisLocalization_Process();
}
int main(void) {
  reset();valid=true;reading(100,0);ChassisLocalization_Init();
  assert(ChassisLocalization_Origin(100,200,0));sample(120);sample(140);
  ChassisLocalization_Status_t before,after;ChassisLocalization_Get(&before);
  assert(ChassisHeading_Apply(.5f*CHASSIS_MODEL_PI/180));
  ChassisLocalization_Get(&after);
  assert(after.generation==before.generation && after.origin.yaw_rad==0);
  assert(after.observer.feedback.x_mm==100 && after.observer.feedback.y_mm==200);
  assert(fabsf(status().map_yaw_rad-.5f*CHASSIS_MODEL_PI/180)<1e-6f);
  assert(fabsf(after.observer.previous_yaw-status().map_yaw_rad)<1e-6f);
  sample(160);ChassisLocalization_Get(&after);assert(after.observer.feedback.x_mm==100);
  reading(180,10);ChassisMotion_HeadingProcess();
  assert(fabsf(status().map_yaw_rad-10.5f*CHASSIS_MODEL_PI/180)<1e-5f);
  location.wheels[0].speed_rpm=10;sample(200);assert(!ChassisHeading_Apply(0));
  puts("heading: atomic map offset, preserved origin/XY/IMU and moving rejection PASS");
}
