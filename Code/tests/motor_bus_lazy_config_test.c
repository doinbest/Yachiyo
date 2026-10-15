#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_bus.h"
#include "mechanical_arm.h"
#include "turntable.h"

static uint32_t tick;
static unsigned sends;
static uint8_t frame[40];
static UART_HandleTypeDef uart={.Instance=UART5};
UART_HandleTypeDef huart5;
uint32_t HAL_GetTick(void){return tick;}
bool ChassisRoute_IsBusy(void){return false;}
bool ChassisMotion_IsBusy(void){return false;}
bool Mecanum_Test_Stop(void){return true;}
bool ConsoleTx_Write(const uint8_t *p,uint16_t n){(void)p;(void)n;return true;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{(void)u;assert(n<=sizeof frame);memcpy(frame,p,n);sends++;return HAL_OK;}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{(void)u;(void)p;(void)n;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u){(void)u;return HAL_OK;}
static void reply(const uint8_t *p,uint16_t n)
{
  MotorBus_TxCpltCallback(&uart);MotorBus_RxBytes(p,n);MotorBus_Process();
  MechanicalArm_Process();tick+=3;MotorBus_Process();
}
int main(int argc,char **argv)
{
  /* Same ordering as production main: ID8 initialization precedes bus/arm init. */
  Turntable_Init();assert(MotorBus_Init(&uart)==HAL_OK);
  assert(MechanicalArm_Init(&uart)==HAL_OK);
  for(tick=0;tick<1000;tick++)MotorBus_Process();
  assert(sends==0 && !MotorBus_IsQuarantined());
  if(argc>1 && !strcmp(argv[1],"turntable_home")) {
    Turntable_BootHomeProcess(false);assert(!Turntable_IsBusy());
    Turntable_BootHomeProcess(true);assert(Turntable_IsBusy());
    Turntable_Process();MotorBus_Process();
    assert(sends==1 && frame[0]==8 && frame[1]==0xf3);
    assert(!MotorBus_IsQuarantined());
    puts("lazy configuration: ID8 boot HOME starts enable without Both preflight PASS");
    return 0;
  }
  /* HOME must not wait for unrelated FD/Both configuration. */
  assert(MechanicalArm_Home(MECHANICAL_ARM_AXIS_Z,2)==MECHANICAL_ARM_RESULT_NONE);
  MotorBus_Process();assert(sends==1 && frame[0]==6 && frame[1]==0x9a);
  const uint8_t home_ack[]={6,0x9a,2,0x6b};reply(home_ack,sizeof home_ack);
  MechanicalArm_EventTypeDef event;assert(MechanicalArm_ResultGet(&event));
  assert(event.Result==MECHANICAL_ARM_RESULT_OK && !MotorBus_IsQuarantined());
  /* The first actual fixed X move still reads, preserves, RAM-selects Both,
   * verifies, and only then admits FD; no other axis is preconfigured. */
  assert(MechanicalArm_Position(MECHANICAL_ARM_AXIS_X,100)==MECHANICAL_ARM_RESULT_BUSY);
  MotorBus_Process();assert(frame[0]==7 && frame[1]==0x42);
  uint8_t cfg[33]={7,0x42,33,21};
  for(unsigned i=4;i<32;i++)cfg[i]=(uint8_t)i;
  cfg[22]=1;cfg[32]=0x6b;reply(cfg,sizeof cfg);
  assert(frame[0]==7 && frame[1]==0x48 && frame[3]==0 && frame[22]==3);
  for(unsigned i=4;i<32;i++)if(i!=22)assert(frame[i]==cfg[i]);
  const uint8_t config_ack[]={7,0x48,2,0x6b};reply(config_ack,sizeof config_ack);
  assert(frame[0]==7 && frame[1]==0x42);
  cfg[22]=3;reply(cfg,sizeof cfg);
  assert(MechanicalArm_Position(MECHANICAL_ARM_AXIS_X,100)==MECHANICAL_ARM_RESULT_NONE);
  MotorBus_Process();assert(frame[0]==7 && frame[1]==0xfd);
  puts("lazy configuration: quiet boot, independent HOME, fixed X read/RAM-Both/verify before FD PASS");
  return 0;
}
