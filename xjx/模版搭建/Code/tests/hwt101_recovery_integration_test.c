/* Real angle driver + calibration state machine; only HAL transactions are fixtures. */
#include "hwt101_calibration.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t tick;
static uint16_t regs[256];
static bool stuck;
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
{ (void)bus;assert(addr==0xa0 && tries==1 && timeout==5);return stuck?HAL_BUSY:HAL_OK; }
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *bus,uint16_t addr,uint16_t reg,
 uint16_t size,uint8_t *data,uint16_t len,uint32_t timeout)
{
  (void)bus;assert(addr==0xa0 && size==1 && len==2 && timeout==5);
  if(stuck)return HAL_BUSY;
  data[0]=(uint8_t)regs[reg];data[1]=(uint8_t)(regs[reg]>>8);return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *bus,uint16_t addr,uint16_t reg,
 uint16_t size,uint8_t *data,uint16_t len,uint32_t timeout)
{
  (void)bus;assert(addr==0xa0 && size==1 && len==2 && timeout==5);
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
int main(void)
{
  I2C_HandleTypeDef bus={0};HWT101_CalStatus_t cal;HWT101_Status_t comm;
  memset(regs,0,sizeof regs);regs[0x2e]=0x1234;regs[0x4c]=0xff7c;
  assert(HWT101_Init(&bus));HWT101_Process();HWT101_Cal_Init();
  assert(HWT101_Cal_Start());advance(10000);assert(regs[0x48]==1 && starts==1);
  /* The module keeps calibrating while only the host I2C recovers. */
  regs[0x4c]=0xff47;stuck=true;advance(1500);
  assert(HWT101_Cal_GetStatus(&cal) && cal.state==HWT101_CAL_CALIBRATING);
  assert(!exits && !zeros && !saves && !cal.verified);
  advance(1500);
  assert(HWT101_Cal_GetStatus(&cal) && cal.busy && !cal.read_recovering);
  assert(cal.first_hal==HAL_BUSY && cal.first_register==0x3f && cal.first_hal_error==0x20);
  assert(regs[0x48]==1 && starts==1 && exits==0 && reinits==1);
  advance(40000);
  assert(HWT101_Cal_GetStatus(&cal) && cal.verified && cal.control_ready);
  assert(cal.normal_mode_confirmed && cal.i2c_errors==1 && cal.recovery_count==1);
  assert(zeros==1 && starts==1 && exits==1 && saves==1 && reinits==1);
  assert(HWT101_Status_Get(&comm) && comm.reinit_count==1 && comm.valid_read_count>1000);
  puts("HWT101 real driver + calibration: BUSY recovery continues native run and full verification PASS");
}
