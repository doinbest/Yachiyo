/* Real angle driver + calibration state machine; only HAL transactions are fixtures. */
#include "hwt101_calibration.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t tick;
static uint16_t regs[256];
static bool stuck, disconnected;
static unsigned angle_failures;
static int fail_read_reg;
static unsigned register_failures;
static unsigned wrong_mode_reads;
static uint16_t wrong_mode_value;
static unsigned starts, exits, zeros, saves, reinits;
uint32_t HAL_GetTick(void) { return tick; }
bool ChassisRoute_IsBusy(void) { return false; }
bool ChassisMotion_IsBusy(void) { return false; }
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *bus) { (void)bus; return stuck?0x20U:0U; }
uint32_t HAL_I2C_GetState(I2C_HandleTypeDef *bus) { (void)bus; return 0x20U; }
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *bus) { (void)bus; return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *bus)
{ (void)bus; reinits++;stuck=false;return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *bus,uint16_t addr,uint32_t tries,uint32_t timeout)
{ (void)bus;assert(addr==0xa0 && tries==1 && timeout==5);return disconnected?HAL_ERROR:(stuck?HAL_BUSY:HAL_OK); }
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *bus,uint16_t addr,uint16_t reg,
 uint16_t size,uint8_t *data,uint16_t len,uint32_t timeout)
{
  (void)bus;assert(addr==0xa0 && size==1 && len==2 && timeout==5);
  if(disconnected)return HAL_ERROR;
  if(stuck)return HAL_BUSY;
  if(reg==0x3f && angle_failures){angle_failures--;return HAL_ERROR;}
  if(reg==fail_read_reg && register_failures){register_failures--;return HAL_ERROR;}
  if(reg==0x48 && wrong_mode_reads){wrong_mode_reads--;data[0]=(uint8_t)wrong_mode_value;data[1]=(uint8_t)(wrong_mode_value>>8);return HAL_OK;}
  data[0]=(uint8_t)regs[reg];data[1]=(uint8_t)(regs[reg]>>8);return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *bus,uint16_t addr,uint16_t reg,
 uint16_t size,uint8_t *data,uint16_t len,uint32_t timeout)
{
  (void)bus;assert(addr==0xa0 && size==1 && len==2 && timeout==5);
  if(disconnected)return HAL_ERROR;
  if(stuck)return HAL_BUSY;
  uint16_t value=(uint16_t)data[0]|((uint16_t)data[1]<<8);
  if(reg==0x48 && value==1)starts++;
  if(reg==0x48 && value==0)exits++;
  if(reg==0x76)zeros++;
  if(reg==0x00)saves++;
  regs[reg]=value;return HAL_OK;
}
static void advance(unsigned ms)
{ for(unsigned i=0;i<ms;i+=10){tick+=10;HWT101_Process();HWT101_Cal_Process();} }
static HWT101_CalStatus_t status(void)
{ HWT101_CalStatus_t s;assert(HWT101_Cal_GetStatus(&s));return s; }
static void setup(uint32_t now)
{
 static I2C_HandleTypeDef bus;
 tick=now;stuck=disconnected=false;angle_failures=0;
 fail_read_reg=-1;register_failures=wrong_mode_reads=0;
 wrong_mode_value=0;
 starts=exits=zeros=saves=reinits=0;
 memset(regs,0,sizeof regs);regs[0x2e]=0x1234;regs[0x4c]=0xff47;
 assert(HWT101_Init(&bus));HWT101_Process();HWT101_Cal_Init();
}
int main(void)
{
 /* Real 1 -> 2 -> 0 sequence: early normal waits out the acquisition, never skips validation. */
 for(unsigned wrap=0;wrap<2;wrap++) {
  setup(wrap?UINT32_MAX-11000U:100);regs[0x4b]=regs[0x4d]=10;
  assert(HWT101_Cal_Start());advance(10000);regs[0x48]=2;stuck=true;advance(3000);
  advance(2800);regs[0x48]=0;stuck=true;advance(3000);
  assert(status().busy && status().state==HWT101_CAL_CALIBRATING && !status().verified);
  assert(status().timing_valid && status().peak_time_s==10 && status().bias_time_s==10);
  assert(status().native_wait_ms==20000 && status().native_modes_seen==7);
  assert(starts==1 && !zeros && !saves && !exits);
  advance(2200);assert(status().state==HWT101_CAL_VERIFYING && zeros==1 && starts==1);
  advance(29000);assert(!status().verified && !saves);
  advance(2000);assert(status().verified && status().control_ready && saves==1);
 }
 /* Longer configured acquisition is honored; early normal never shortens its budget. */
 setup(100);regs[0x4b]=10;regs[0x4d]=30;assert(HWT101_Cal_Start());
 advance(10000);regs[0x48]=2;advance(6000);regs[0x48]=0;advance(14000);
 assert(status().state==HWT101_CAL_CALIBRATING && !zeros && !saves);
 assert(status().total_ms==70000 && status().native_wait_ms==40000);
 advance(11000);assert(status().state==HWT101_CAL_VERIFYING && zeros==1 && !saves);
 /* Real drift still fails the full verification after an early normal transition. */
 setup(100);assert(HWT101_Cal_Start());advance(10000);regs[0x48]=2;advance(6000);
 regs[0x48]=0;advance(5000);assert(status().state==HWT101_CAL_VERIFYING);
 for(unsigned i=0;i<310;i++){regs[0x3f]++;advance(100);}
 assert(!status().verified && !saves && !strcmp(status().reason,"drift_or_noise"));
 /* Observed HWT101 sequence: peak-to-peak mode changes to zero-bias mode near 10 s. */
 for(unsigned wrap=0;wrap<2;wrap++) {
  setup(wrap?UINT32_MAX-11000U:100);
  assert(HWT101_Cal_Start());advance(10000);regs[0x48]=2;stuck=true;
  advance(3000);
  assert(status().busy && status().state==HWT101_CAL_CALIBRATING && !status().read_recovering);
  assert(regs[0x48]==2 && starts==1 && exits==0 && zeros==0 && saves==0);
  assert(status().recovery_mode_first==2 && status().recovery_mode_last==2);
  assert(status().recovery_mode_mask==0x0006 && status().recovery_mode_reads==1);
  advance(6000);assert(status().state==HWT101_CAL_CALIBRATING && exits==0);
  advance(2000);assert(status().state==HWT101_CAL_VERIFYING && starts==1 && exits==1 && zeros==1);
  advance(31000);assert(status().verified && status().control_ready && saves==1);
 }
 /* A delayed initial readback may already observe the next valid native phase. */
 setup(100);assert(HWT101_Cal_Start());advance(110);assert(starts==1);
 regs[0x48]=2;advance(200);
 assert(status().busy && status().state==HWT101_CAL_CALIBRATING && !exits);
 /* Zero-bias mode remains forbidden during normal-mode verification. */
 setup(100);assert(HWT101_Cal_VerifyStart(5000));advance(1000);
 angle_failures=1;advance(10);regs[0x48]=2;advance(1500);
 assert(!status().busy && !status().verified && !saves);
 assert(!strcmp(status().reason,"recovery_mode_changed"));
 assert(status().recovery_mode_mask==0x0001);
 /* Normal/scale-factor modes cannot masquerade as an active native calibration. */
 for(unsigned bad=0;bad<=3;bad+=3) {
  setup(100);assert(HWT101_Cal_Start());advance(1000);
  angle_failures=1;advance(10);regs[0x48]=(uint16_t)bad;advance(1500);
  assert(!status().busy && !status().verified && !saves && starts==1);
  assert(!strcmp(status().reason,"recovery_mode_changed"));
 }
 /* A single inconsistent mode read after link recovery must not abort or replay native start. */
 for(unsigned wrap=0;wrap<2;wrap++) {
 setup(wrap?UINT32_MAX-2050U:100);assert(HWT101_Cal_Start());advance(1000);
 angle_failures=1;advance(10);wrong_mode_reads=1;
 for(unsigned waited=0;!status().recovery_mode_reads && waited<1500;waited+=10)advance(10);
 assert(status().recovery_mode_reads==1 && status().busy && status().read_recovering);
 assert(!strcmp(status().reason,"recovery_mode_recheck"));
 advance(90);assert(status().recovery_mode_reads==1 && exits==0);
 advance(10);assert(status().recovery_mode_reads==2 && !status().read_recovering);
 assert(status().recovery_mode_first==0 && status().recovery_mode_last==1);
 assert(status().recovery_expected_mode==1 && !strcmp(status().recovery_step,"CAL_WAIT"));
 assert(status().busy && status().state==HWT101_CAL_CALIBRATING);
 assert(starts==1 && exits==0 && zeros==0 && saves==0);
 advance(20000);assert(status().state==HWT101_CAL_VERIFYING && starts==1 && exits==1 && zeros==1);
 advance(31000);assert(status().verified && status().control_ready && saves==1);
 }
 /* A recheck during verification discards partial statistics without repeating yaw zero. */
 setup(100);assert(HWT101_Cal_VerifyStart(5000));advance(4200);
 angle_failures=1;advance(10);wrong_mode_value=1;wrong_mode_reads=1;
 advance(1500);assert(status().busy && zeros==1 && status().samples<10);
 advance(4000);assert(status().busy && !status().verified);
 advance(1500);assert(status().verified && zeros==1 && saves==0);
 for(unsigned wrap=0;wrap<2;wrap++) {
  setup(wrap?UINT32_MAX-11000U:100);
  assert(HWT101_Cal_Start());advance(10000);stuck=true;advance(1500);
  assert(status().busy && status().state==HWT101_CAL_CALIBRATING);
  assert(!exits && !zeros && !saves && !status().verified);
  advance(1500);assert(status().busy && regs[0x48]==1 && starts==1 && reinits==1);
  advance(10000);assert(status().state==HWT101_CAL_VERIFYING && starts==1 && exits==1 && zeros==1);
  advance(31000);assert(status().verified && status().control_ready && saves==1);
  assert(status().i2c_errors==1 && status().recovery_count==1 && !status().read_recovering);
 }
 /* One failed angle read discards the partial verification window, never re-zeroes. */
 setup(100);assert(HWT101_Cal_VerifyStart(5000));advance(4200);
 assert(status().samples>30);angle_failures=1;advance(10);
 assert(status().busy && status().read_recovering && !status().verified);
 regs[0x3f]=40; /* Recovery samples must not overwrite the original post-zero sample. */
 advance(1500);assert(status().busy && status().samples<10 && zeros==1 && !saves);
 advance(4000);assert(status().busy && !status().verified);
 advance(1500);assert(status().verified && status().samples==51 && zeros==1 && !saves);
 assert(status().zero_after_deg==0.0f);
 /* Recoverable errors at normal-mode/bias readback must not replay exit writes. */
 for(unsigned bias=0;bias<2;bias++) {
  setup(100);assert(HWT101_Cal_Start());
  while(!exits)advance(10);
  fail_read_reg=bias?0x4c:0x48;register_failures=1;
  advance(2000);assert(status().busy && !status().read_recovering && exits==1 && zeros==1);
  advance(31000);assert(status().verified && saves==1 && starts==1 && exits==1);
 }
 /* A fault near the end does not restart the sensor's twenty-second timer. */
 setup(100);assert(HWT101_Cal_Start());advance(19500);stuck=true;advance(3000);
 assert(status().state==HWT101_CAL_VERIFYING && starts==1 && exits==1 && zeros==1);
 /* Changing module mode during native recovery cannot silently restart calibration. */
 setup(100);assert(HWT101_Cal_Start());advance(1000);angle_failures=1;advance(10);regs[0x48]=0x1234;
 advance(2000);assert(!status().busy && !status().verified && starts==1 && !saves);
 assert(!strcmp(status().reason,"recovery_mode_changed"));
 assert(status().mode==0 && status().normal_mode_confirmed);
 assert(status().recovery_mode_first==0x1234 && status().recovery_mode_last==0x1234);
 assert(status().recovery_mode_seen && status().recovery_mode_valid && status().recovery_mode_reads==2);
 assert(status().recovery_expected_mode==1 && !strcmp(status().recovery_step,"CAL_WAIT"));
 assert(HWT101_Cal_RefreshRegisters()==HAL_OK && status().recovery_mode_last==0x1234);
 /* Cancellation during recheck prevents continuation and all SAVE writes. */
 setup(100);assert(HWT101_Cal_Start());advance(1000);angle_failures=1;advance(10);wrong_mode_reads=1;
 for(unsigned waited=0;!status().recovery_mode_reads && waited<1500;waited+=10)advance(10);
 HWT101_Cal_Cancel();advance(1000);
 assert(status().state==HWT101_CAL_CANCELLED && !saves && starts==1);
 /* Real outage has one deadline, despite repeated probes; exit stays independently bounded. */
 setup(100);assert(HWT101_Cal_Start());advance(1000);disconnected=true;advance(11000);
 assert(!status().busy && !status().verified && !saves && !status().normal_mode_confirmed);
 assert(!strcmp(status().first_reason,"i2c"));
 /* Cancel while waiting never resumes native calibration or saves on recovery. */
 setup(100);assert(HWT101_Cal_Start());advance(1000);stuck=true;advance(100);HWT101_Cal_Cancel();
 advance(3000);assert(!status().busy && status().state==HWT101_CAL_CANCELLED && !saves && starts==1);
 puts("IMU read recovery: native continuation, full fresh verification, mode check, timeout, cancel and tick wrap PASS");
}
