#define main legacy_grab_test_main
#include "grab_task_test.c"
#undef main

int main(void)
{
  reset();configure(true);
  StationHeading_Config_t ring,line;
  setting("head_temp_zero","7");setting("head_temp_gain","2");
  GrabTask_LineHeadingConfigGet(&line);
  assert(isnan(line.zero_deg) && isnan(line.gain));
  setting("head_line_zero","1.5");setting("head_line_gain","-1");
  GrabTask_LineHeadingConfigGet(&line);GrabTask_HeadingConfigGet(3,&ring);
  assert(line.zero_deg==1.5f && line.gain==-1 && ring.zero_deg==7 && ring.gain==2);
  Status.base_home_degraded=1;GrabTask_LineHeadingConfigGet(&line);assert(isnan(line.gain));
  Status.base_home_degraded=0;
  setting("z_car_lift","-11");GrabTask_LineHeadingConfigGet(&line);
  assert(isnan(line.gain));
  setting("head_line_gain","-1");GrabTask_ReferenceInvalidate(MECHANICAL_ARM_AXIS_BASE);
  GrabTask_LineHeadingConfigGet(&line);assert(isnan(line.gain));
  puts("heading source config: independent ring/line mapping, height/reference invalidation PASS");
  return 0;
}
