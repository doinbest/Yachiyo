/* Production bus + wheel batching. Driver motion ACKs deliberately absent. */
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "motor_bus.h"
#include "mecanum_chassis.h"
#include "hwt101_calibration.h"
static UART_HandleTypeDef uart;
static uint32_t tick=1,last_done;
static bool pending,previous_no_ack;
static uint8_t wire[40],wire_length,mode[8]={1,1,1,1,1,1,1,1};
static uint8_t config[8][33];
static int rpm[8],cached[8];
static unsigned configs[8],writes[8],moves,syncs,queries,stops,hold_fault;
static bool drop_query,noise;
static bool receive_mode;
bool ChassisRoute_IsBusy(void){return false;}
bool ChassisMotion_IsBusy(void){return false;}
bool HWT101_Cal_IsBusy(void){return false;}
bool HWT101_Cal_ControlAngleGet(HWT101_Angle_t *out){(void)out;return false;}
uint32_t HAL_GetTick(void){return tick;}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{(void)u;(void)p;(void)n;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u){(void)u;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{
  (void)u;assert(!pending && n<=sizeof wire);
  if(previous_no_ack)assert(tick-last_done>=2); /* Also before the next query. */
  memcpy(wire,p,n);wire_length=(uint8_t)n;pending=true;return HAL_OK;
}
static void service_wire(void)
{
  if(!pending)return;
  uint8_t reply[40]={0};unsigned length=0;
  unsigned id=wire[0]?wire[0]-1U:0;
  reply[0]=(uint8_t)(id+1);reply[1]=wire[1];
  previous_no_ack=false;
  if(wire[1]==0x42){
    assert(wire_length==4 && wire[2]==0x6c);configs[id]++;
    memcpy(reply,config[id],33);reply[0]=(uint8_t)(id+1);reply[1]=0x42;
    reply[2]=33;reply[3]=21;reply[22]=mode[id];reply[32]=0x6b;length=33;
  }else if(wire[1]==0x48){
    assert(wire_length==33 && wire[3]==0); /* RAM only, preserve every field. */
    for(unsigned n=4;n<32;n++)if(n!=22)assert(wire[n]==config[id][n]);
    mode[id]=wire[22];memcpy(config[id]+4,wire+4,28);writes[id]++;
    reply[2]=2;reply[3]=0x6b;length=4;
  }else if(wire[1]==0xf6){
    assert(mode[id]==(receive_mode?1:0) && configs[id]>=(receive_mode?1U:2U));
    cached[id]=((unsigned)wire[3]<<8)|wire[4];if(wire[2])cached[id]=-cached[id];
    if(!wire[6])rpm[id]=cached[id];
    moves++;previous_no_ack=!receive_mode;
    if(receive_mode){reply[2]=2;reply[3]=0x6b;length=4;}
  }else if(wire[1]==0xff){
    assert(mode[id]==(receive_mode?1:0));syncs++;previous_no_ack=!receive_mode;
    if(receive_mode){reply[2]=2;reply[3]=0x6b;length=4;}
    for(unsigned i=wire[0]?id:0;i<(wire[0]?id+1:4);i++)rpm[i]=cached[i];
  }else if(wire[1]==0xfe || wire[1]==0x9c){
    rpm[id]=0;stops++;
    if(mode[id]){reply[2]=2;reply[3]=0x6b;length=4;}
    else previous_no_ack=true;
  }else if(wire[1]==0x35 || wire[1]==0x36 || wire[1]==0x3a || wire[1]==0x3b){
    queries++;length=wire[1]==0x35?6:wire[1]==0x36?8:4;
    if(wire[1]==0x35){unsigned mag=(unsigned)abs(rpm[id]);reply[2]=rpm[id]<0;reply[3]=mag>>8;reply[4]=mag;}
    if(wire[1]==0x3a)reply[2]=hold_fault?9:3;
    if(wire[1]==0x3b)reply[2]=3;
    reply[length-1]=0x6b;if(drop_query)length=0;
  }else assert(0);
  pending=false;last_done=tick;MotorBus_TxCpltCallback(&uart);
  if(noise && wire[1]==0xf6){uart.ErrorCode=HAL_UART_ERROR_NE;MotorBus_ErrorCallback(&uart);noise=false;}
  if(length)MotorBus_RxBytes(reply,(uint16_t)length);
}
static void step(void){tick++;service_wire();MotorBus_Process();Mecanum_Process();MotorBus_Process();}
static void run(unsigned duration){while(duration--)step();}
int main(int argc,char **argv)
{
  receive_mode=argc>1 && !strcmp(argv[1],"receive");
  for(unsigned i=0;i<8;i++)for(unsigned n=4;n<32;n++)config[i][n]=(uint8_t)(n+i+10);
  assert(MotorBus_Init(&uart)==HAL_OK);
  Mecanum_Status_t status;Mecanum_StatusGet(&status);assert(status.ack_profile==MECANUM_ACK_NONE);
  if(receive_mode)assert(Mecanum_AckProfile_Set(MECANUM_ACK_RECEIVE));
  assert(Mecanum_Velocity_Request(100,0,0,0));
  for(unsigned n=0;n<1200;n++){step();Mecanum_StatusGet(&status);if(status.tx_complete)break;}
  assert(status.tx_complete && status.acknowledged==receive_mode && !status.locked);
  assert(moves==4 && syncs==1);
  for(unsigned i=0;i<4;i++)assert(mode[i]==(receive_mode?1:0) && configs[i]==(receive_mode?1U:2U) && writes[i]==(receive_mode?0U:1U));
  uart.ErrorCode=HAL_UART_ERROR_NE;MotorBus_ErrorCallback(&uart);run(5);
  Mecanum_Feedback_Enable(true);run(500);
  if(queries<=20 || MotorBus_IsQuarantined()) {
    MotorBus_Diagnostic_t fault;MotorBus_DiagnosticGet(&fault);
    fprintf(stderr,"queries=%u fault=%s id=%u fn=%02X active=%u wait=%u tick=%lu\n",
      queries,fault.reason,fault.address,fault.function,fault.active,fault.waiting_reply,(unsigned long)tick);
  }
  assert(queries>20 && !MotorBus_IsQuarantined());
  Mecanum_Feedback_t feedback;for(unsigned i=0;i<4;i++)assert(Mecanum_FeedbackGet(i,&feedback) && feedback.speed_valid);
  assert(Mecanum_Test_Stop());run(250);Mecanum_StatusGet(&status);
  assert(!status.stop_pending && status.tx_complete && !status.locked && !MotorBus_IsQuarantined());
  for(unsigned i=0;i<4;i++)assert(!rpm[i]);
  Mecanum_Feedback_Enable(false);run(30);
  assert(MotorBus_RecoveryStart());MotorBus_Recovery_t recovery;
  for(unsigned n=0;n<7000;n++){step();MotorBus_RecoveryGet(&recovery);if(!recovery.active)break;}
  assert(recovery.success && !MotorBus_IsQuarantined());
  assert(Mecanum_RecoveryAfterBusCheck());
  /* Parameter mode caches are invalidated by recovery; re-read, never rewrite unchanged None. */
  noise=!receive_mode;
  assert(Mecanum_Velocity_Request(50,0,0,0));run(400);Mecanum_StatusGet(&status);
  assert(status.tx_complete && !status.locked);
  for(unsigned i=0;i<4;i++)assert(writes[i]==(receive_mode?0U:1U) && configs[i]>=(receive_mode?2U:3U));
  assert(Mecanum_Test_Stop());run(150);
  if(!receive_mode) {
    /* Selecting the compatibility profile is not proof that RAM has changed.
     * A stop before its next configuration must still handle physical None. */
    assert(Mecanum_AckProfile_Set(MECANUM_ACK_RECEIVE));
    assert(Mecanum_Test_Stop());run(250);assert(!MotorBus_IsQuarantined());
    assert(Mecanum_AckProfile_Set(MECANUM_ACK_NONE));
  }
  /* Queries still require replies and still detect a genuinely missing response. */
  drop_query=true;Mecanum_Feedback_Enable(true);run(350);assert(MotorBus_IsQuarantined());
  printf("chassis_none_bus_test: %s, RAM modes, 2ms gaps, feedback, NE, stop, recovery and query timeout PASS (%u queries)\n",receive_mode?"Receive compatibility":"None without motion ACKs",queries);
}
