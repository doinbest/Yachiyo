/* Reuse sensor fixture; exercise real drift, store and SPI driver together. */
#define main drift_regression_unused
#include "hwt101_drift_test.c"
#undef main
#include "hwt101_store.h"
#include "w25q128.h"

GPIO_TypeDef test_gpioa;
static SPI_HandleTypeDef spi;
static uint8_t memory[8192];
static GPIO_PinState cs=GPIO_PIN_SET;
static bool wel, stuck_busy, fail_spi, wrong_id, corrupt_program;
static uint32_t busy_until;
static unsigned erases, programs;
void HAL_GPIO_WritePin(GPIO_TypeDef *p,uint16_t pin,GPIO_PinState s)
{assert(p==GPIOA && pin==GPIO_PIN_4 && s!=cs);cs=s;}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h,uint8_t *tx,uint8_t *rx,uint16_t size,uint32_t timeout)
{
  bool busy=stuck_busy || (int32_t)(busy_until-tick)>0;
  uint32_t addr;
  assert(h==&spi && cs==GPIO_PIN_RESET && timeout==20);
  memset(rx,0xFF,size);
  if(fail_spi)return HAL_TIMEOUT;
  if(tx[0]==0x05){assert(size==2);rx[1]=(busy?1:0)|(wel?2:0);return HAL_OK;}
  assert(!busy);
  if(tx[0]==0x9F){assert(size==4);rx[1]=wrong_id?0:0xEF;rx[2]=0x40;rx[3]=0x18;return HAL_OK;}
  if(tx[0]==0x06){assert(size==1);wel=true;return HAL_OK;}
  assert(size>=4);
  addr=((uint32_t)tx[1]<<16)|((uint32_t)tx[2]<<8)|tx[3];
  assert(addr>=0xFFE000 && addr<0x1000000);addr-=0xFFE000;
  if(tx[0]==0x03){assert(addr+size-4<=sizeof(memory));memcpy(rx+4,memory+addr,size-4);}
  else if(tx[0]==0x20)
  {assert(wel && !(addr&4095) && size==4);memset(memory+addr,0xFF,4096);wel=false;busy_until=tick+400;erases++;}
  else if(tx[0]==0x02)
  {
    assert(wel && size>4 && (addr&255)+size-4<=256);
    for(unsigned i=4;i<size;i++)memory[addr+i-4]&=tx[i];
    if(corrupt_program)memory[addr]^=1;
    wel=false;busy_until=tick+3;programs++;
  }
  else assert(0);
  return HAL_OK;
}
static HWT101_StoreStatus_t stored(void)
{HWT101_StoreStatus_t s;HWT101_Store_StatusGet(&s);return s;}
static void reboot(void)
{
  /* Flash survives; MCU yaw/time origin must not. */
  busy_until=0;tick=10;memset(&comm,0,sizeof(comm));ready=available=true;
  angle.yaw=-70;angle.update_count=1;angle.last_update_ms=tick;
  HWT101_Drift_Clear();HWT101_Store_Init(&spi);HWT101_Store_Process();
}
static void pump(unsigned duration)
{
  for(unsigned n=0;n<duration;n+=10)
  {sample(10,angle.yaw+0.001f);HWT101_Store_Process();}
}
static void pass_cal(float bias)
{
  begin(tick+100,179);HWT101_Store_Process();
  for(unsigned ms=100;ms<=180000;ms+=100)
  {sample(100,179+bias*ms/1000);HWT101_Store_Process();}
  assert(snapshot().state==HWT101_DRIFT_DONE && !snapshot().restored_from_flash);
}
int main(void)
{
  uint8_t old[8192], out[8]={1,2,3};
  memset(memory,0xFF,sizeof(memory));reboot();
  assert(!stored().has_saved && snapshot().state==HWT101_DRIFT_IDLE && erases==0);
  assert(W25Q128_ReadData(NULL,0xFFE000,out,1)==HAL_ERROR);
  assert(W25Q128_ProgramStart(&spi,0xFFFFFF,out,2)==HAL_ERROR);
  assert(W25Q128_EraseSectorStart(&spi,0xFFE001)==HAL_ERROR);
  fail_spi=true;assert(W25Q128_ReadData(&spi,0xFFE000,out,3)==HAL_TIMEOUT && out[0]==1 && cs==GPIO_PIN_SET);fail_spi=false;
  stuck_busy=true;assert(W25Q128_EraseSectorStart(&spi,0xFFE000)==HAL_BUSY);stuck_busy=false;
  pass_cal(0.1f);assert(stored().busy);pump(500);
  assert(!strcmp(stored().state,"saved") && erases==1 && programs==2);
  assert(snapshot().state==HWT101_DRIFT_DONE); /* Erase polling did not stop sampling. */
  memcpy(old,memory,sizeof(memory));pump(1000);assert(erases==1);
  reboot();assert(snapshot().restored_from_flash && stored().has_saved);
  assert(fabsf(snapshot().corrected_deg)<0.0001f && fabsf(snapshot().bias_dps-0.1f)<0.0001f);
  pump(100);assert(fabsf(snapshot().corrected_deg)<0.001f && erases==1);
  HWT101_Angle_t control;assert(HWT101_Drift_ControlAngleGet(&control));
  /* Interrupted update: old sector remains readable, new body has no commit. */
  pass_cal(0.12f);pump(400);assert(stored().busy);reboot();
  assert(fabsf(snapshot().bias_dps-0.1f)<0.0001f);
  pass_cal(0.12f);pump(500);reboot();assert(fabsf(snapshot().bias_dps-0.12f)<0.0001f);
  /* Corrupt newer record -> fall back to older CRC-valid record. */
  memory[4096+16]^=1;reboot();assert(fabsf(snapshot().bias_dps-0.1f)<0.0001f);
  assert(HWT101_Store_Forget()==HAL_OK);HWT101_Drift_Clear();pump(500);
  assert(!strcmp(stored().state,"forgotten"));reboot();assert(!stored().has_saved && snapshot().state==HWT101_DRIFT_IDLE);
  /* Both invalid/empty, wrong ID, SPI fault and busy timeout never restore. */
  memset(memory,0,sizeof(memory));reboot();assert(!stored().has_saved);
  memcpy(memory,old,sizeof(memory));wrong_id=true;reboot();assert(!strcmp(stored().state,"error") && snapshot().state==HWT101_DRIFT_IDLE);wrong_id=false;
  fail_spi=true;reboot();assert(!strcmp(stored().state,"error"));fail_spi=false;
  stuck_busy=true;reboot();tick+=1001;HWT101_Store_Process();assert(!strcmp(stored().state,"error"));stuck_busy=false;
  reboot();corrupt_program=true;pass_cal(0.11f);pump(500);assert(!strcmp(stored().state,"error"));corrupt_program=false;
  reboot();assert(fabsf(snapshot().bias_dps-0.1f)<0.0001f);
  /* Clear before the sensor is ready cancels deferred restore for this boot. */
  HWT101_Drift_Clear();available=false;HWT101_Store_Init(&spi);HWT101_Store_Process();
  assert(!strcmp(stored().state,"waiting_imu"));HWT101_Store_CancelRestore();available=true;HWT101_Store_Process();
  assert(snapshot().state==HWT101_DRIFT_IDLE);
  puts("hwt101_store_test: real SPI commands, auto-save/restore, CRC fallback, interrupted update, forget and failures OK");
}
