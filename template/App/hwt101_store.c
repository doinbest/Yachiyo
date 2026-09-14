#include "hwt101_store.h"
#include "hwt101_drift.h"
#include "w25q128.h"
#include <math.h>
#include <string.h>

/* W25Q128最后8KiB专用。新记录提交前始终保留另一扇区的旧记录。 */
#define STORE_BASE 0xFFE000UL
#define STORE_SECTOR_SIZE 4096UL
#define STORE_MAGIC 0x424D5549UL
#define STORE_COMMIT 0x54494D43UL
#define STORE_RECORD_SIZE 28U
#define STORE_WAIT_MS 1000U
typedef enum { STORE_BOOT, STORE_READY, STORE_ERASING, STORE_BODY, STORE_COMMITTING, STORE_DISABLED } StorePhase_t;
static SPI_HandleTypeDef *bus;
static StorePhase_t phase;
static HWT101_StoreStatus_t status;
static HWT101_DriftState_t last_drift_state;
static uint32_t wait_tick;
static int active_slot;
static uint8_t target_slot, pending[STORE_RECORD_SIZE];
static bool restore_pending, restore_cancelled;

static uint32_t Read32(const uint8_t *p)
{ return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24); }
static void Write32(uint8_t *p, uint32_t n)
{ p[0]=(uint8_t)n; p[1]=(uint8_t)(n>>8); p[2]=(uint8_t)(n>>16); p[3]=(uint8_t)(n>>24); }
static uint32_t Crc32(const uint8_t *p, unsigned size)
{
  uint32_t crc=0xFFFFFFFFUL;
  unsigned i, bit;
  for (i=0; i<size; i++)
  {
    crc ^= p[i];
    for (bit=0; bit<8; bit++) crc=(crc>>1)^((crc&1U)?0xEDB88320UL:0U);
  }
  return ~crc;
}
static float RecordBias(const uint8_t *p)
{
  uint32_t bits=Read32(p+16);
  float bias;
  memcpy(&bias, &bits, sizeof(bias));
  return bias;
}
static bool Valid(const uint8_t *p)
{
  float bias=RecordBias(p);
  return Read32(p)==STORE_MAGIC && Read32(p+4)==1U && Read32(p+12)<=1U &&
         Read32(p+20)==Crc32(p,20) && Read32(p+24)==STORE_COMMIT &&
         isfinite(bias) && fabsf(bias)<=5.0f;
}
static uint32_t TargetAddress(void) { return STORE_BASE + target_slot*STORE_SECTOR_SIZE; }
static void Fail(HAL_StatusTypeDef result)
{
  status.state="error"; status.hal=result; status.busy=false;
  phase=(phase==STORE_BOOT)?STORE_DISABLED:STORE_READY;
}
/* 每次只读一次SR1，超时不占用CPU等待。 */
static HAL_StatusTypeDef Poll(void)
{
  uint8_t sr;
  HAL_StatusTypeDef result=W25Q128_ReadStatus(bus,&sr);
  if (result==HAL_BUSY)
    return (uint32_t)(HAL_GetTick()-wait_tick)>=STORE_WAIT_MS ? HAL_TIMEOUT : HAL_BUSY;
  if (result!=HAL_OK) return result;
  if (!(sr&1U)) return HAL_OK;
  return (uint32_t)(HAL_GetTick()-wait_tick)>=STORE_WAIT_MS ? HAL_TIMEOUT : HAL_BUSY;
}
static void Load(void)
{
  uint8_t id[3], records[2][STORE_RECORD_SIZE];
  bool valid0, valid1;
  HAL_StatusTypeDef result=W25Q128_ReadJedecId(bus,id);
  if (result!=HAL_OK) { Fail(result); return; }
  if (id[0]!=0xEF || id[1]!=0x40 || id[2]!=0x18) { Fail(HAL_ERROR); return; }
  result=W25Q128_ReadData(bus,STORE_BASE,records[0],STORE_RECORD_SIZE);
  if (result==HAL_OK) result=W25Q128_ReadData(bus,STORE_BASE+STORE_SECTOR_SIZE,records[1],STORE_RECORD_SIZE);
  if (result!=HAL_OK) { Fail(result); return; }
  valid0=Valid(records[0]); valid1=Valid(records[1]);
  active_slot=valid0?0:-1;
  if (valid1 && (!valid0 || (Read32(records[1]+8)!=Read32(records[0]+8) &&
      (uint32_t)(Read32(records[1]+8)-Read32(records[0]+8))<0x80000000UL))) active_slot=1;
  status.state="empty";
  if (active_slot>=0)
  {
    status.sequence=Read32(records[active_slot]+8);
    status.has_saved=Read32(records[active_slot]+12)==1U;
    status.bias_dps=RecordBias(records[active_slot]);
    status.state=status.has_saved?(restore_cancelled?"stored_not_applied":"waiting_imu"):"forgotten";
  }
  restore_pending=status.has_saved && !restore_cancelled;
  status.busy=false; phase=STORE_READY;
}
static HAL_StatusTypeDef BeginWrite(bool save, float bias)
{
  uint32_t bits;
  if (phase==STORE_DISABLED || !bus) return HAL_ERROR;
  if (phase!=STORE_READY) return HAL_BUSY;
  if (!isfinite(bias) || fabsf(bias)>5.0f) return HAL_ERROR;
  target_slot=(active_slot==0)?1U:0U;
  memset(pending,0,sizeof(pending));
  Write32(pending,STORE_MAGIC); Write32(pending+4,1);
  Write32(pending+8,status.sequence+1U); Write32(pending+12,save?1U:0U);
  memcpy(&bits,&bias,sizeof(bits)); Write32(pending+16,bits);
  Write32(pending+20,Crc32(pending,20)); Write32(pending+24,STORE_COMMIT);
  status.hal=W25Q128_EraseSectorStart(bus,TargetAddress());
  if (status.hal!=HAL_OK) { Fail(status.hal); return status.hal; }
  wait_tick=HAL_GetTick(); phase=STORE_ERASING;
  status.busy=true; status.state=save?"saving":"forgetting";
  return HAL_OK;
}
void HWT101_Store_Init(SPI_HandleTypeDef *hspi)
{
  bus=hspi; memset(&status,0,sizeof(status));
  active_slot=-1; restore_pending=false; restore_cancelled=false;
  last_drift_state=HWT101_DRIFT_IDLE;
  status.state="loading"; status.busy=true;
  phase=STORE_BOOT; wait_tick=HAL_GetTick();
  if (!bus) Fail(HAL_ERROR);
}
void HWT101_Store_Process(void)
{
  HWT101_DriftSnapshot_t drift;
  HAL_StatusTypeDef result;
  uint8_t verify[STORE_RECORD_SIZE];
  (void)HWT101_Drift_Get(&drift);
  if (phase==STORE_BOOT)
  {
    result=Poll();
    if (result==HAL_BUSY) return;
    if (result!=HAL_OK) { Fail(result); return; }
    Load();
  }
  if (restore_pending)
  {
    if (drift.state!=HWT101_DRIFT_IDLE) restore_pending=false;
    else if (HWT101_Drift_Restore(status.bias_dps))
    {
      restore_pending=false; status.state="restored";
      (void)HWT101_Drift_Get(&drift);
    }
  }
  if (drift.state==HWT101_DRIFT_DONE && last_drift_state!=HWT101_DRIFT_DONE &&
      !drift.restored_from_flash && drift.compensation_valid && drift.verification_passed)
  {
    result=BeginWrite(true,drift.bias_dps);
    if (result!=HAL_OK && !status.busy) { status.state="error"; status.hal=result; }
  }
  last_drift_state=drift.state;
  if (phase!=STORE_ERASING && phase!=STORE_BODY && phase!=STORE_COMMITTING) return;
  result=Poll();
  if (result==HAL_BUSY) return;
  if (result!=HAL_OK) { Fail(result); return; }
  if (phase==STORE_ERASING)
  {
    result=W25Q128_ProgramStart(bus,TargetAddress(),pending,24);
    if (result!=HAL_OK) { Fail(result); return; }
    phase=STORE_BODY; wait_tick=HAL_GetTick();
  }
  else if (phase==STORE_BODY)
  {
    /* 正文已完成，再单独提交；掉电未提交的记录不会被采用。 */
    result=W25Q128_ProgramStart(bus,TargetAddress()+24,pending+24,4);
    if (result!=HAL_OK) { Fail(result); return; }
    phase=STORE_COMMITTING; wait_tick=HAL_GetTick();
  }
  else
  {
    result=W25Q128_ReadData(bus,TargetAddress(),verify,sizeof(verify));
    if (result!=HAL_OK || memcmp(verify,pending,sizeof(verify)) || !Valid(verify))
    { Fail(result==HAL_OK?HAL_ERROR:result); return; }
    active_slot=target_slot;
    status.sequence=Read32(verify+8); status.has_saved=Read32(verify+12)==1U;
    status.bias_dps=RecordBias(verify); status.busy=false; status.hal=HAL_OK;
    status.state=status.has_saved?"saved":"forgotten"; phase=STORE_READY;
  }
}
HAL_StatusTypeDef HWT101_Store_Forget(void)
{
  HAL_StatusTypeDef result=BeginWrite(false,0.0f);
  if (result==HAL_OK) HWT101_Store_CancelRestore();
  return result;
}
void HWT101_Store_CancelRestore(void)
{
  if (restore_pending) status.state="stored_not_applied";
  restore_pending=false; restore_cancelled=true;
}
void HWT101_Store_StatusGet(HWT101_StoreStatus_t *out) { if (out) *out=status; }
