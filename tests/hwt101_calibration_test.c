#include "hwt101_calibration.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t tick, unlock_tick, last_write_tick;
static bool route_busy, motion_busy;
bool ChassisRoute_IsBusy(void) { return route_busy; }
bool ChassisMotion_IsBusy(void) { return motion_busy; }
static bool ready, available, unlocked, mismatch, fail_save, fail_restore;
static int fail_read_reg, fail_write_reg;
static unsigned writes, saves, starts, exits, unlocks, zeros;
static uint16_t regs[256];
static HWT101_Angle_t angle;
static HWT101_Status_t comm;
uint32_t HAL_GetTick(void) { return tick; }
bool HWT101_Is_Ready(void) { return ready; }
bool HWT101_Angle_Get(HWT101_Angle_t *out) { if(out)*out=angle; return available; }
bool HWT101_Status_Get(HWT101_Status_t *out) { *out=comm;return true; }
bool HWT101_Angle_Is_Fresh(const HWT101_Angle_t *a,uint32_t age)
{ return a && a->update_count && (uint32_t)(tick-a->last_update_ms)<=age; }
HAL_StatusTypeDef HWT101_ReadRegister(uint8_t reg,uint16_t *out)
{
  if(reg==fail_read_reg){comm.i2c_error_count++;return HAL_ERROR;}
  *out=regs[reg]; if(reg==0x48 && mismatch)*out=7;
  return HAL_OK;
}
HAL_StatusTypeDef HWT101_WriteRegister(uint8_t reg,uint16_t value)
{
  writes++;
  if(reg==0x69){assert(value==0xB588);unlocks++;unlocked=true;unlock_tick=tick;}
  else
  {
    assert(unlocked && (uint32_t)(tick-unlock_tick)>=100 && (uint32_t)(tick-unlock_tick)<10000);
    if(reg==0x48 && value==1) starts++;
    else if(reg==0x48 && value==0) exits++;
    else if(reg==0x00){assert(value==0);saves++;}
    else if(reg==0x76){assert(value==0);zeros++;}
    else assert(!"unexpected register write");
  }
  last_write_tick=tick;
  if(reg==fail_write_reg || (reg==0 && fail_save) || (reg==0x48 && value==0 && fail_restore))
  {comm.i2c_error_count++;return HAL_TIMEOUT;}
  regs[reg]=value;
  if(reg==0x76) angle.yaw=0;
  if(reg==0x48 && value==0) regs[0x4C]=456;
  return HAL_OK;
}
static HWT101_CalStatus_t snapshot(void)
{HWT101_CalStatus_t s;assert(HWT101_Cal_GetStatus(&s));return s;}
static float wrap(float yaw){while(yaw>=180)yaw-=360;while(yaw< -180)yaw+=360;return yaw;}
static void setup(uint32_t now)
{
  tick=now;ready=available=true;unlocked=mismatch=fail_save=fail_restore=false;
  fail_read_reg=fail_write_reg=-1;writes=saves=starts=exits=unlocks=zeros=0;
  memset(regs,0,sizeof(regs));regs[0x2E]=0x1234;regs[0x4C]=123;
  memset(&angle,0,sizeof(angle));memset(&comm,0,sizeof(comm));
  angle.yaw=179.9f;angle.update_count=1;angle.last_update_ms=tick;
  HWT101_Cal_Init();
}
static void sample(uint32_t dt,float step)
{
  tick+=dt;angle.yaw=wrap(angle.yaw+step);angle.last_update_ms=tick;angle.update_count++;
  HWT101_Cal_Process();
}
static void advance(unsigned ms,float dps)
{for(unsigned i=0;i<ms;i+=10)sample(10,dps*0.01f);}
static void until_verify(void)
{
  for(unsigned i=0;i<2200 && snapshot().state!=HWT101_CAL_VERIFYING;i++)sample(10,0);
  assert(snapshot().state==HWT101_CAL_VERIFYING);
}
static void finish(void)
{
  for(unsigned i=0;i<4000 && HWT101_Cal_IsBusy();i++)sample(10,0);
  assert(!HWT101_Cal_IsBusy());
}
int main(void)
{
  for (unsigned owner = 0; owner < 2; owner++)
  {
    setup(100); route_busy = owner == 0; motion_busy = owner == 1;
    assert(!HWT101_Cal_Start()); assert(!HWT101_Cal_VerifyStart(30000));
    assert(!snapshot().busy && !writes);
    assert(HWT101_Cal_RefreshRegisters() == HAL_OK);
    HWT101_Cal_Cancel(); assert(!HWT101_Cal_IsBusy());
  }
  route_busy = motion_busy = false;
  setup(100);assert(HWT101_Cal_VerifyStart(10000));
  advance(100,0);assert(zeros==0 && snapshot().samples==0);
  until_verify();assert(zeros==1 && saves==0 && starts==0 && exits==0);
  finish();assert(snapshot().verified && fabsf(angle.yaw)<0.001f);
  setup(100);fail_write_reg=0x76;assert(HWT101_Cal_VerifyStart(10000));finish();
  assert(!snapshot().verified && !strcmp(snapshot().reason,"yaw_zero_write") && saves==0);
  setup(100);assert(HWT101_Cal_VerifyStart(10000));sample(10,0);
  HWT101_Cal_Cancel();finish();assert(zeros==0 && saves==0);
  setup(100); assert(HWT101_Cal_Start()); advance(1000, 0);
  route_busy = true; HWT101_Cal_Cancel(); finish();
  assert(regs[0x48] == 0 && exits == 1 && !snapshot().busy);
  route_busy = false;
  HWT101_Angle_t output={0};output.yaw=42;
  setup(100);assert(!HWT101_Cal_GetStatus(NULL));assert(!HWT101_Cal_ControlAngleGet(NULL));
  assert(!HWT101_Cal_ControlAngleGet(&output) && output.yaw==42);
  assert(HWT101_Cal_RefreshRegisters()==HAL_OK && writes==0);
  assert(snapshot().version==0x1234 && snapshot().bias_before==123);
  available=false;assert(!HWT101_Cal_Start() && writes==0);
  setup(100);regs[0x48]=1;assert(!HWT101_Cal_Start() && writes==0);
  HWT101_Cal_Cancel();finish();assert(regs[0x48]==0 && exits==1);
  setup(100);fail_read_reg=0x48;assert(HWT101_Cal_RefreshRegisters()==HAL_ERROR);
  fail_read_reg=-1;regs[0x48]=1;HWT101_Cal_Cancel();finish();assert(regs[0x48]==0 && exits==1);
  setup(100);fail_read_reg=0x4C;assert(!HWT101_Cal_Start() && writes==0);
  setup(100);assert(!HWT101_Cal_VerifyStart(5000));

  for(unsigned run=0;run<2;run++)
  {
    setup(run?UINT32_MAX-5000:100);assert(HWT101_Cal_Start());
    assert(!HWT101_Cal_Start());assert(HWT101_Cal_RefreshRegisters()==HAL_BUSY);
    advance(19000,0);assert(starts==1 && exits==0 && saves==0);
    until_verify();assert(exits==1 && unlocks==3 && regs[0x48]==0);
    assert(!HWT101_Cal_ControlAngleGet(&output));
    advance(29500,0);assert(saves==0);finish();
    assert(snapshot().verified && !strcmp(snapshot().result,"PASS"));
    assert(saves==1 && unlocks==4 && snapshot().save_requested && snapshot().save_readback_ok);
    assert(snapshot().bias_before==123 && snapshot().bias_after==456);
    assert(HWT101_Cal_ControlAngleGet(&output) && output.yaw==angle.yaw);
    sample(10,10);assert(HWT101_Cal_ControlAngleGet(&output) && output.yaw==angle.yaw);
    comm.i2c_error_count++;HWT101_Cal_Process();assert(!HWT101_Cal_ControlAngleGet(&output));
  }

  /* Ten seconds is a short check, with unchanged drift/error limits. */
  setup(100);assert(HWT101_Cal_VerifyStart(10000));advance(9500,0.005f);
  assert(!snapshot().verified && HWT101_Cal_IsBusy());advance(1000,0.005f);
  assert(snapshot().verified && snapshot().verify_ms==10000 && writes==2 && zeros==1 && saves==0);
  setup(UINT32_MAX-3000);assert(HWT101_Cal_VerifyStart(10000));advance(10500,0.1f);
  assert(!snapshot().verified && !strcmp(snapshot().result,"FAIL") && writes==2 && zeros==1 && saves==0);
  setup(100);assert(HWT101_Cal_VerifyStart(10000));until_verify();sample(10,0);sample(301,0);
  assert(!snapshot().verified && !strcmp(snapshot().reason,"sample_gap"));

  /* Validation resets yaw once, but never changes bias or subtracts the slope. */
  setup(100);assert(HWT101_Cal_VerifyStart(30000));advance(30500,0.005f);
  assert(snapshot().verified && writes==2 && zeros==1 && saves==0 && fabsf(snapshot().drift_dps-0.005f)<0.0005f);
  assert(HWT101_Cal_ControlAngleGet(&output) && output.yaw==angle.yaw);
  tick+=301;assert(!HWT101_Cal_Start());assert(!snapshot().verified && !strcmp(snapshot().result,"FAIL"));
  setup(100);assert(!HWT101_Cal_VerifyStart(120000));
  assert(!snapshot().busy && writes==0);
  setup(100);assert(HWT101_Cal_VerifyStart(30000));advance(30500,0.1f);
  assert(!snapshot().verified && !strcmp(snapshot().result,"FAIL") && writes==2 && zeros==1 && saves==0);
  setup(100);assert(HWT101_Cal_VerifyStart(30000));until_verify();sample(10,0);
  for(unsigned i=0;i<301;i++)sample(100,(i%2)?-0.4f:0.4f);
  assert(!snapshot().verified && snapshot().rms_deg>0.1f && writes==2 && zeros==1 && saves==0);
  setup(UINT32_MAX-1000);assert(HWT101_Cal_VerifyStart(30000));until_verify();angle.yaw=-179.99f;sample(10,0);
  for(unsigned i=0;HWT101_Cal_IsBusy() && i<300;i++)
  {unsigned dt=i%2?170:130;sample(dt,-0.005f*dt/1000.0f);}
  assert(snapshot().verified && fabsf(snapshot().drift_dps+0.005f)<0.0001f && writes==2 && zeros==1 && saves==0);
  sample(301,0);assert(!snapshot().control_ready);
  sample(10,0);assert(!snapshot().control_ready); /* 新鲜数据不自动恢复验证资格。 */
  setup(100);assert(HWT101_Cal_Start());until_verify();advance(30500,0.1f);finish();
  assert(!snapshot().verified && saves==0 && regs[0x48]==0);

  setup(100);assert(HWT101_Cal_VerifyStart(30000));until_verify();sample(10,0);sample(301,0);
  assert(!strcmp(snapshot().reason,"sample_gap") && snapshot().max_gap_ms==301);
  setup(100);assert(HWT101_Cal_VerifyStart(30000));until_verify();sample(10,0);tick+=301;HWT101_Cal_Process();
  assert(!strcmp(snapshot().reason,"stale"));
  setup(100);assert(HWT101_Cal_VerifyStart(30000));until_verify();sample(10,0);sample(100,6);
  assert(!strcmp(snapshot().reason,"motion"));
  setup(100);assert(HWT101_Cal_VerifyStart(30000));until_verify();sample(10,0);comm.i2c_error_count++;sample(10,0);
  assert(!snapshot().verified && !strcmp(snapshot().reason,"i2c"));

  setup(100);assert(HWT101_Cal_Start());advance(1000,0);HWT101_Cal_Cancel();finish();
  assert(regs[0x48]==0 && exits==1 && saves==0 && !snapshot().verified);
  setup(100);fail_write_reg=0x48;assert(HWT101_Cal_Start());finish();
  assert(starts==1 && exits==3 && !strcmp(snapshot().reason,"normal_mode_unconfirmed"));
  fail_write_reg=-1;HWT101_Cal_Cancel();finish();assert(regs[0x48]==0 && snapshot().normal_mode_confirmed);
  setup(100);assert(HWT101_Cal_Start());advance(1000,0);fail_restore=true;HWT101_Cal_Cancel();finish();
  assert(exits==3 && !strcmp(snapshot().reason,"normal_mode_unconfirmed") && saves==0);
  setup(100);assert(HWT101_Cal_Start());advance(100,0);mismatch=true;finish();
  assert(!snapshot().verified && saves==0);

  setup(100);assert(HWT101_Cal_Start());until_verify();fail_save=true;finish();
  assert(snapshot().verified && !strcmp(snapshot().result,"PASS") && !strcmp(snapshot().save_state,"ERROR"));
  assert(!snapshot().save_requested && !snapshot().save_readback_ok);
  setup(100);assert(HWT101_Cal_Start());until_verify();fail_save=fail_restore=true;finish();
  assert(!strcmp(snapshot().reason,"normal_mode_unconfirmed") && !strcmp(snapshot().result,"FAIL"));
  setup(100);assert(HWT101_Cal_Start());until_verify();
  while(!saves)sample(10,0);
  fail_read_reg=0x4C;finish();
  assert(snapshot().verified && snapshot().save_requested && !snapshot().save_readback_ok);
  setup(100);assert(HWT101_Cal_Start());until_verify();finish();
  assert(snapshot().verified && snapshot().save_requested);
  tick+=301;assert(!HWT101_Cal_Start());
  assert(!snapshot().save_requested && !snapshot().save_readback_ok && snapshot().samples==0);
  assert(snapshot().verify_elapsed_ms==0 && !strcmp(snapshot().save_state,"NOT_REQUESTED"));
  puts("hwt101_calibration_test: native sequence, validation, no subtraction, cancel/recovery and save faults PASS");
  return 0;
}
