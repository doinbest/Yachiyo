/* Source-only regression this round: no compilation or hardware operations. */
#define main existing_motion_main
#define ChassisLocalization_Get FixtureLocalizationGet
#include "chassis_motion_test.c"
#undef ChassisLocalization_Get
#undef main
void ChassisLocalization_Get(ChassisLocalization_Status_t *out);

static ChassisLocalization_Status_t reference_status(void)
{ChassisLocalization_Status_t s;ChassisLocalization_Get(&s);return s;}
static void wheel_sample(uint32_t now)
{
  tick=now;
  for(unsigned i=0;i<4;i++){
    location.wheels[i].position_valid=location.wheels[i].speed_valid=location.wheels[i].state_valid=true;
    location.wheels[i].position_ms=location.wheels[i].speed_ms=location.wheels[i].state_ms=now;
    location.wheels[i].state_flags=3;
  }
  ChassisMotion_HeadingProcess();ChassisLocalization_Process();
}
static void qualified_sample(uint32_t now,float yaw)
{reading(now,yaw);wheel_sample(now);}
static void reference_ready(float yaw)
{
  reset();valid=true;reading(100,yaw);ChassisLocalization_Init();
  assert(ChassisLocalization_Origin(100,200,CHASSIS_MODEL_PI/2));
  qualified_sample(120,yaw);qualified_sample(140,yaw);
  assert(status().map_anchor_valid && reference_status().origin_valid && reference_status().feedback_valid);
}
int main(void)
{
  reference_ready(179);
  uint32_t generation=reference_status().generation;
  float before=status().map_yaw_rad;
  valid=false;qualified_sample(160,-179);
  assert(!status().heading_valid && status().map_anchor_valid);
  assert(reference_status().origin_valid && !reference_status().feedback_valid);
  assert(reference_status().generation==generation+1);
  /* Qualification is still withheld: a new sample alone must not grant use. */
  qualified_sample(180,-179);assert(!status().heading_valid);
  valid=true;qualified_sample(200,-179);qualified_sample(220,-178);
  assert(status().heading_valid && status().map_anchor_valid && reference_status().feedback_valid);
  assert(reference_status().origin_valid && reference_status().origin.x_mm==100 && reference_status().origin.y_mm==200);
  assert(fabsf(status().map_yaw_rad-before-3*CHASSIS_HEADING_TEST_YAW_SIGN*CHASSIS_MODEL_PI/180)<1e-5f);
  assert(reference_status().generation==generation+1 && !sends && !stops);

  reference_ready(0);wheel_sample(140+HWT101_DATA_FRESH_MS+1);
  assert(!status().heading_valid && status().map_anchor_valid && reference_status().origin_valid);
  assert(!reference_status().feedback_valid && status().heading_sample_tick==140);
  qualified_sample(480,0);qualified_sample(500,0);assert(reference_status().feedback_valid);

  reference_ready(0);qualified_sample(160,NAN);
  assert(!status().heading_valid && status().map_anchor_valid && reference_status().origin_valid);
  qualified_sample(180,1);qualified_sample(200,1);assert(reference_status().feedback_valid);

  /* Explicit calibration/zero/verify invalidates the previous mapping. */
  reference_ready(0);cal_busy=true;wheel_sample(160);
  assert(!status().map_anchor_valid && !reference_status().origin_valid);
  cal_busy=false;qualified_sample(180,0);qualified_sample(200,0);
  assert(status().heading_valid && !status().map_anchor_valid && !reference_status().origin_valid);

  reference_ready(0);reading(160,0);angle.update_count=1;wheel_sample(160);
  assert(!status().map_anchor_valid && !reference_status().origin_valid);
  qualified_sample(180,0);assert(!status().map_anchor_valid);

  reference_ready(0);ChassisMotion_AnchorInvalidate();wheel_sample(160);
  assert(!status().map_anchor_valid && !reference_status().origin_valid);
  puts("IMU reference: transient qualification/staleness retains mapping; explicit changes and stream restart revoke it PASS");
  return 0;
}
