#include "hwt101_drift.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t tick;
static bool ready = true, available = true;
static HWT101_Angle_t angle;
static HWT101_Status_t comm;
uint32_t HAL_GetTick(void) { return tick; }
bool HWT101_Is_Ready(void) { return ready; }
bool HWT101_Angle_Get(HWT101_Angle_t *out) { *out = angle; return available; }
bool HWT101_Status_Get(HWT101_Status_t *out) { *out = comm; return true; }
bool HWT101_Angle_Is_Fresh(const HWT101_Angle_t *a, uint32_t age)
{ return a && a->update_count && (uint32_t)(tick-a->last_update_ms) <= age; }
static float wrap(float x) { while(x>=180)x-=360;while(x< -180)x+=360;return x; }
static HWT101_DriftSnapshot_t snapshot(void)
{ HWT101_DriftSnapshot_t s; assert(HWT101_Drift_Get(&s)); return s; }
static void begin(uint32_t start, float yaw)
{
  HWT101_Drift_Clear(); memset(&comm,0,sizeof(comm));
  ready=available=true; tick=start; angle.yaw=yaw;
  angle.last_update_ms=tick; angle.update_count=1;
  assert(HWT101_Drift_Start()); assert(!HWT101_Drift_Start());
}
static void sample(uint32_t dt, float yaw)
{
  tick+=dt; angle.yaw=wrap(yaw); angle.last_update_ms=tick; angle.update_count++;
  HWT101_Drift_Process();
}
static void calibrate(float bias)
{
  for(unsigned ms=100;ms<=60000;ms+=100) sample(100,179.0f+bias*ms/1000.0f);
  assert(snapshot().state==HWT101_DRIFT_VERIFYING);
  assert(fabsf(snapshot().bias_dps-bias)<0.0001f);
}
int main(void)
{
  HWT101_Drift_Clear(); available=false;
  assert(!HWT101_Drift_Start()); assert(!HWT101_Drift_Get(NULL));
  for(unsigned run=0;run<3;run++)
  {
    float bias=run==0?0.1f:run==1?-0.1f:0.0f;
    begin(run==1?UINT32_MAX-5000:100,179.0f); calibrate(bias);
    for(unsigned ms=100;ms<=120000;ms+=100) sample(100,179+bias*(60+ms/1000.0f));
    HWT101_DriftSnapshot_t s=snapshot();
    assert(s.state==HWT101_DRIFT_DONE && s.compensation_valid && s.verification_passed);
    assert(fabsf(s.residual_dps)<0.0001f && fabsf(s.corrected_deg)<0.001f);
    HWT101_Angle_t control;
    assert(HWT101_Drift_ControlAngleGet(&control));
    assert(fabsf(control.yaw)<0.001f && control.update_count==angle.update_count);
    /* Control getter uses the newest raw read even between 100ms fit samples. */
    sample(10,179+bias*180.01f+1);
    assert(HWT101_Drift_ControlAngleGet(&control) && fabsf(control.yaw-1)<0.001f);
    sample(100,179+bias*180.11f+2); assert(fabsf(snapshot().corrected_deg-2)<0.001f);
    unsigned n=snapshot().samples; HWT101_Drift_Process(); assert(snapshot().samples==n);
    comm.i2c_error_count++; HWT101_Drift_Process();
    assert(snapshot().state==HWT101_DRIFT_FAILED && !snapshot().compensation_valid);
  }
  /* Actual timestamps, not an assumed sample period. */
  begin(100,179); unsigned elapsed=0;
  while(elapsed<60000) { unsigned dt=elapsed%2?170:130;elapsed+=dt;sample(dt,179+0.1f*elapsed/1000); }
  assert(snapshot().state==HWT101_DRIFT_VERIFYING && fabsf(snapshot().bias_dps-0.1f)<0.0001f);
  /* Stable training but changed bias on independent validation must fail. */
  begin(100,179); calibrate(0.1f);
  for(unsigned ms=100;ms<=120000;ms+=100) sample(100,185+0.15f*ms/1000);
  assert(snapshot().state==HWT101_DRIFT_FAILED && !snapshot().verification_passed);
  assert(fabsf(snapshot().residual_dps-0.05f)<0.0001f);
  begin(100,0); sample(301,0); assert(!strcmp(snapshot().reason,"sample_gap"));
  assert(snapshot().gap_ms==301 && snapshot().max_gap_ms==301);
  sample(100,0); assert(snapshot().gap_ms==301); /* Failure snapshot stays frozen. */
  begin(UINT32_MAX-100,0); sample(100,0); sample(250,0);
  assert(snapshot().gap_ms==250 && snapshot().max_gap_ms==250);
  sample(100,0); assert(snapshot().gap_ms==100 && snapshot().max_gap_ms==250);
  HWT101_Drift_Clear(); assert(!snapshot().gap_ms && !snapshot().max_gap_ms);
  begin(100,0); tick+=301;HWT101_Drift_Process();assert(!strcmp(snapshot().reason,"stale"));
  begin(100,0); sample(100,6);assert(!strcmp(snapshot().reason,"motion"));
  begin(100,0);ready=false;HWT101_Drift_Process();assert(!strcmp(snapshot().reason,"i2c"));
  begin(100,0);
  for(unsigned ms=100;ms<=60000;ms+=100)
    sample(100,ms<=30000?0.1f*ms/1000:3+0.2f*(ms-30000)/1000);
  assert(!strcmp(snapshot().reason,"unstable_bias"));
  begin(100,0);
  for(unsigned ms=100;ms<=60000;ms+=100) sample(100,0.1f*ms/1000+((ms/100)%2?0.3f:-0.3f));
  assert(!strcmp(snapshot().reason,"noise"));
  begin(100,0);HWT101_Drift_Clear();assert(snapshot().state==HWT101_DRIFT_IDLE);
  assert(!snapshot().compensation_valid);
  HWT101_Angle_t untouched={0};untouched.yaw=123;
  assert(!HWT101_Drift_ControlAngleGet(NULL));
  assert(!HWT101_Drift_ControlAngleGet(&untouched) && untouched.yaw==123);
  begin(100,179);calibrate(0.1f);
  assert(!HWT101_Drift_ControlAngleGet(&untouched)); /* VERIFYING is not approved. */
  for(unsigned ms=100;ms<=120000;ms+=100)sample(100,185+0.1f*ms/1000);
  tick+=301;assert(!HWT101_Drift_ControlAngleGet(&untouched));
  angle.last_update_ms=tick;angle.update_count++;
  assert(!HWT101_Drift_ControlAngleGet(&untouched)); /* Fresh but excessive gap. */
  HWT101_Drift_Clear();assert(!HWT101_Drift_Restore(NAN));
  assert(!HWT101_Drift_Restore(6.0f));
  assert(HWT101_Drift_Restore(0.1f) && snapshot().restored_from_flash);
  assert(!HWT101_Drift_Restore(0.2f)); /* No origin switch during active compensation. */
  comm.i2c_error_count++;assert(!HWT101_Drift_ControlAngleGet(&untouched));
  puts("hwt101_drift_test: calibration, independent validation, wrap, real motion and faults OK");
  return 0;
}
