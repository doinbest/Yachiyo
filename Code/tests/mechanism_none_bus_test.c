/* Actual bus/arm/tray with drivers that never ACK actions in None mode. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "motor_bus.h"
#include "mechanical_arm.h"
#include "turntable.h"

static UART_HandleTypeDef uart={.Instance=UART5};
UART_HandleTypeDef huart5;
static uint32_t tick=1,last_done;
static uint8_t wire[40],config[8][33],mode[8];
static unsigned reads[8],writes[8],actions[8],queries,wire_size;
static int32_t position[8];
static bool enabled[8],pending,previous_none,drop_query,drop_config,bad_verify;
static unsigned home_reads[8];
uint32_t HAL_GetTick(void){return tick;}
bool ChassisRoute_IsBusy(void){return false;}
bool ChassisMotion_IsBusy(void){return false;}
bool Mecanum_Test_Stop(void){return true;}
bool ConsoleTx_Write(const uint8_t *p,uint16_t n){(void)p;(void)n;return true;}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{(void)u;(void)p;(void)n;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u){(void)u;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{
  (void)u;assert(!pending && n<=sizeof wire);
  if(previous_none)assert(tick-last_done>=2);
  memcpy(wire,p,n);wire_size=n;pending=true;return HAL_OK;
}
static void service(void)
{
  if(!pending)return;
  unsigned id=wire[0]-1U;assert(id<8);
  uint8_t r[40]={wire[0],wire[1],2,0x6b};unsigned n=4;
  previous_none=false;
  if(wire[1]==0x42){
    assert(wire_size==4 && wire[2]==0x6c);reads[id]++;
    memcpy(r,config[id],33);r[22]=mode[id];n=drop_config?0:33;
    if(bad_verify && writes[id])r[10]^=1;
  }else if(wire[1]==0x48){
    assert(wire_size==33 && wire[2]==0xd1 && wire[3]==0 && wire[22]==0);
    for(unsigned i=4;i<32;i++)if(i!=22)assert(wire[i]==config[id][i]);
    mode[id]=0;writes[id]++;
  }else if(wire[1]==0x35 || wire[1]==0x36 || wire[1]==0x3a || wire[1]==0x3b){
    queries++;memset(r+2,0,8);n=wire[1]==0x35?6:wire[1]==0x36?8:4;
    if(wire[1]==0x36){
      uint32_t mag=position[id]<0?(uint32_t)-(int64_t)position[id]:(uint32_t)position[id];
      r[2]=position[id]<0;r[3]=mag>>24;r[4]=mag>>16;r[5]=mag>>8;r[6]=mag;
    }else if(wire[1]==0x3a)r[2]=enabled[id]?3:2;
    else if(wire[1]==0x3b){r[2]=3;home_reads[id]++;}
    r[n-1]=0x6b;if(drop_query)n=0;
  }else if(wire[1]==0x93){
    /* Explicit origin persistence is configuration, not a motion ACK. */
    assert(wire[2]==0x88 && wire[3]==1);
  }else{
    assert(wire[1]==0xf3 || wire[1]==0xfd || wire[1]==0x9a ||
           wire[1]==0xfe || wire[1]==0x9c || wire[1]==0x0a ||
           wire[1]==0xf6 || wire[1]==0xff);
    if(wire[1]==0xf3 && wire[3])assert(mode[id]==0 && reads[id]>=2);
    if(wire[1]==0xfd || wire[1]==0x9a)assert(mode[id]==0 && reads[id]>=2);
    if(wire[1]==0xf3)enabled[id]=wire[3]!=0;
    if(wire[1]==0xfd){
      uint32_t pulses=((uint32_t)wire[6]<<24)|((uint32_t)wire[7]<<16)|((uint32_t)wire[8]<<8)|wire[9];
      position[id]=(int32_t)lround((double)pulses*65536/3200)*(wire[2]?-1:1);
    }
    if(wire[1]==0x9a || wire[1]==0x0a)position[id]=0;
    actions[id]++;n=0;previous_none=true;
  }
  pending=false;last_done=tick;MotorBus_TxCpltCallback(&uart);
  if(n)MotorBus_RxBytes(r,n);
}
static void step(void)
{tick++;service();MotorBus_Process();MechanicalArm_Process();Turntable_Process();}
static void run(unsigned n){while(n--)step();}
static MechanicalArm_EventTypeDef arm_done(void)
{
  MechanicalArm_EventTypeDef e;bool done=false;
  for(unsigned i=0;i<1200 && !done;i++){step();done=MechanicalArm_ResultGet(&e);}
  assert(done && e.Result==MECHANICAL_ARM_RESULT_OK && e.Transmitted);
  assert(!e.Acknowledged && !e.Diagnostic.ReplyReceived && !e.Diagnostic.ReplyLength);
  assert(!MotorBus_IsQuarantined());return e;
}
static void ensure_enable(MechanicalArm_AxisTypeDef axis)
{
  MechanicalArm_ResultTypeDef result=MECHANICAL_ARM_RESULT_BUSY;
  for(unsigned i=0;i<1200 && result==MECHANICAL_ARM_RESULT_BUSY;i++){
    result=MechanicalArm_Enable(axis);step();
  }
  assert(result==MECHANICAL_ARM_RESULT_NONE);arm_done();
}
static void tray_done(void)
{
  for(unsigned i=0;i<2000 && Turntable_IsBusy();i++)step();
  assert(!Turntable_IsBusy() && !MotorBus_IsQuarantined());
}
int main(int argc,char **argv)
{
  const char *kind=argc>1?argv[1]:"normal";
  for(unsigned id=0;id<8;id++){
    mode[id]=id&1?3:1;enabled[id]=true;
    config[id][0]=id+1;config[id][1]=0x42;config[id][2]=33;config[id][3]=21;
    for(unsigned i=4;i<32;i++)config[id][i]=(uint8_t)(i+id);
    config[id][22]=mode[id];config[id][32]=0x6b;
  }
  Turntable_Init();assert(MechanicalArm_Init(&uart)==HAL_OK);run(20);assert(!pending);
  if(!strcmp(kind,"config_missing") || !strcmp(kind,"config_mismatch")){
    drop_config=!strcmp(kind,"config_missing");bad_verify=!drop_config;
    MechanicalArm_ResultTypeDef r=MECHANICAL_ARM_RESULT_BUSY;
    for(unsigned i=0;i<1200 && r==MECHANICAL_ARM_RESULT_BUSY;i++){
      r=MechanicalArm_Enable(MECHANICAL_ARM_AXIS_Z);step();
    }
    assert(r==MECHANICAL_ARM_RESULT_TX_ERROR && !actions[5]);
    puts("None preflight blocks action on missing/mismatched full readback PASS");return 0;
  }
  ensure_enable(MECHANICAL_ARM_AXIS_ALL);
  assert(MechanicalArm_Home(MECHANICAL_ARM_AXIS_ALL,2)==MECHANICAL_ARM_RESULT_NONE);
  arm_done();
  for(unsigned id=4;id<7;id++)assert(reads[id]==2 && writes[id]==1 && actions[id]==2);
  for(unsigned axis=0;axis<3;axis++){
    assert(MechanicalArm_PositionEx((MechanicalArm_AxisTypeDef)axis,100,20,axis?0:1,
      MECHANICAL_ARM_POSITION_ABSOLUTE)==MECHANICAL_ARM_RESULT_NONE);
    MotorBus_Process();for(unsigned n=0;n<10 && !pending;n++)step();
    assert(pending && !MechanicalArm_ResultGet(&(MechanicalArm_EventTypeDef){0}));
    arm_done();
  }
  assert(MechanicalArm_PositionTrackingEx(MECHANICAL_ARM_AXIS_X,120,25,0,
    MECHANICAL_ARM_POSITION_ABSOLUTE)==MECHANICAL_ARM_RESULT_NONE);arm_done();
  /* A receive-only NE during action TX must not demand an absent motion ACK. */
  assert(MechanicalArm_Position(MECHANICAL_ARM_AXIS_Z,100)==MECHANICAL_ARM_RESULT_NONE);
  for(unsigned i=0;i<10 && !pending;i++)step();
  assert(pending);
  uart.ErrorCode=HAL_UART_ERROR_NE;MotorBus_ErrorCallback(&uart);
  MotorBus_Process();MechanicalArm_Process();
  assert(!MotorBus_IsQuarantined() && !MechanicalArm_ResultGet(&(MechanicalArm_EventTypeDef){0}));
  arm_done();run(120);assert(!MotorBus_IsQuarantined());
  /* Cancel an action still on wire; allow that TX to drain, never replay it. */
  assert(MechanicalArm_Position(MECHANICAL_ARM_AXIS_X,130)==MECHANICAL_ARM_RESULT_NONE);
  for(unsigned i=0;i<10 && !pending;i++)step();
  assert(pending);
  unsigned old_actions=actions[6];MotorBus_Cancel(MOTOR_BUS_ARM);
  run(10);MechanicalArm_EventTypeDef cancelled;
  assert(MechanicalArm_ResultGet(&cancelled) && cancelled.Result!=MECHANICAL_ARM_RESULT_OK);
  assert(cancelled.Transmitted && !cancelled.Acknowledged && actions[6]==old_actions+1);
  assert(!MotorBus_IsQuarantined());
  assert(MechanicalArm_StopScheduled(MECHANICAL_ARM_AXIS_X)==MECHANICAL_ARM_RESULT_NONE);arm_done();
  assert(MechanicalArm_HomeAbort(MECHANICAL_ARM_AXIS_Z)==MECHANICAL_ARM_RESULT_NONE);arm_done();
  assert(MechanicalArm_Disable(MECHANICAL_ARM_AXIS_ALL)==MECHANICAL_ARM_RESULT_NONE);arm_done();
  ensure_enable(MECHANICAL_ARM_AXIS_ALL);
  for(unsigned id=4;id<7;id++)assert(reads[id]==2 && writes[id]==1);
  assert(Turntable_Home());uint32_t started=tick;tray_done();
  Turntable_Status_t status;Turntable_StatusGet(&status);
  assert(status.arrived && status.reference_valid && tick-started>=200 && home_reads[7]);
  assert(reads[7]==2 && writes[7]==1);
  assert(Turntable_Index(2));started=tick;tray_done();Turntable_StatusGet(&status);
  assert(status.arrived && status.slot==2 && tick-started>=200);
  Turntable_Stop();tray_done();run(120);assert(!MotorBus_IsQuarantined());
  /* A stale FD ACK must not complete the next actual position query. */
  assert(MechanicalArm_PositionRead(MECHANICAL_ARM_AXIS_X)==MECHANICAL_ARM_RESULT_NONE);
  for(unsigned i=0;i<10 && !pending;i++)step();
  assert(pending && wire[1]==0x36);
  pending=false;previous_none=false;MotorBus_TxCpltCallback(&uart);MotorBus_Process();
  MechanicalArm_EventTypeDef e;
  const uint8_t stale_ack[]={7,0xfd,2,0x6b};
  MotorBus_RxBytes(stale_ack,sizeof stale_ack);MotorBus_Process();MechanicalArm_Process();
  assert(!MechanicalArm_ResultGet(&e));
  const uint8_t position_reply[]={7,0x36,0,0,0,0,42,0x6b};
  MotorBus_RxBytes(position_reply,sizeof position_reply);MotorBus_Process();MechanicalArm_Process();
  assert(MechanicalArm_ResultGet(&e) && e.CurrentPositionRaw==42);
  assert(e.Action==MECHANICAL_ARM_ACTION_POSITION_READ && e.Acknowledged);
  if(!strcmp(kind,"query_missing")){
    drop_query=true;assert(MechanicalArm_PositionRead(MECHANICAL_ARM_AXIS_X)==MECHANICAL_ARM_RESULT_NONE);
    MotorBus_Process();unsigned before=queries;run(150);
    assert(MotorBus_IsQuarantined() && queries==before+1);
    assert(MechanicalArm_ResultGet(&e) && e.Result==MECHANICAL_ARM_RESULT_ACK_TIMEOUT);
    puts("None still requires real query replies and original timeout PASS");return 0;
  }
  /* Recovery must not expect FE/9C replies from the verified None mechanisms. */
  assert(MotorBus_RecoveryStart());MotorBus_Recovery_t recovery;
  for(unsigned i=0;i<7000;i++){step();MotorBus_RecoveryGet(&recovery);if(!recovery.active)break;}
  assert(recovery.success && !MotorBus_IsQuarantined());
  ensure_enable(MECHANICAL_ARM_AXIS_Z);assert(writes[5]==1 && reads[5]==3);
  assert(MechanicalArm_Stop(MECHANICAL_ARM_AXIS_ALL)==MECHANICAL_ARM_RESULT_NONE);
  arm_done();run(150);assert(!MotorBus_IsQuarantined());
  printf("mechanism_none_bus_test: no action ACK, full RAM verify, all axes, X tracking, ID8 stable/HOME, queries, stop/recovery PASS (%u queries)\n",queries);
}
