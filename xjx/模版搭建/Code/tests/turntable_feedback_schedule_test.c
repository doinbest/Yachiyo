#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../template/App/turntable.c"

static uint32_t tick;
static unsigned queries;
static unsigned releases;
static MotorBus_Event_t response;
static bool response_ready;
static bool reached_reply;
uint32_t HAL_GetTick(void){return tick;}
bool MotorBus_IsQuarantined(void){return false;}
void MotorBus_DiagnosticGet(MotorBus_Diagnostic_t *out){memset(out,0,sizeof *out);}
bool MotorBus_PositionReached(uint8_t address,uint32_t accepted){(void)address;(void)accepted;return reached_reply;}
int MotorBus_PositionReplyEnsure(uint8_t address){(void)address;return 1;}
bool MotorBus_Reserve(MotorBus_Owner_t owner){(void)owner;return true;}
void MotorBus_Release(MotorBus_Owner_t owner){(void)owner;releases++;}
void MotorBus_Cancel(MotorBus_Owner_t owner){(void)owner;response_ready=false;}
bool ConsoleTx_Write(const uint8_t *p,uint16_t size){(void)p;(void)size;return true;}
bool MotorBus_Submit(MotorBus_Owner_t owner,const uint8_t *data,uint8_t length,
                     uint8_t reply_length,uint32_t token,bool priority)
{
  (void)priority;assert(owner==MOTOR_BUS_TURNTABLE && length==3 && data[0]==8);
  assert(data[1]==0x36 || data[1]==0x35 || data[1]==0x3a);
  queries++;memset(&response,0,sizeof response);
  response.result=MOTOR_BUS_REPLY;response.length=reply_length;response.token=token;
  response.tick_ms=tick;response.data[0]=8;response.data[1]=data[1];
  response.data[reply_length-1]=0x6b;
  if(data[1]==0x3a)response.data[2]=3;
  response_ready=true;return true;
}
bool MotorBus_EventGet(MotorBus_Owner_t owner,MotorBus_Event_t *out)
{
  (void)owner;if(!response_ready)return false;
  *out=response;response_ready=false;return true;
}
static void seed(void)
{
  tick=1000;queries=releases=0;response_ready=false;reached_reply=false;Turntable_Init();
  Op=OP_INDEX;Step=TT_PHASE_SAMPLE;MotionStarted=true;Started=tick;MotionBudget=5000;
  LastCycle=tick;Status.reference_valid=true;TargetRaw=OriginRaw=PositionRaw=0;
  Status.target_deg=0;
}
int main(void)
{
  seed();Op=OP_VERIFY;MotionStarted=false;
  for(unsigned n=0;n<160;n++){tick++;Turntable_Process();}
  assert(Turntable_IsBusy() && !Status.arrived && releases>=2);
  seed();reached_reply=true;Turntable_Process();
  assert(Turntable_IsBusy() && !Status.arrived); /* Receive ignores old Both completion. */
  seed();
  for(unsigned n=0;n<40;n++){tick++;Turntable_Process();}
  assert(queries>=3); /* A complete group before the old 50ms idle expires. */
  assert(Turntable_IsBusy() && !Status.arrived); /* Keep 200ms stability. */
  uint32_t first_stable=StableSince;
  while(tick-first_stable<200U){tick++;Turntable_Process();assert(!Status.arrived);}
  while(Turntable_IsBusy() && tick-first_stable<260U){tick++;Turntable_Process();}
  assert(Status.arrived && tick-first_stable>=200U);
  unsigned before=queries;
  assert(Turntable_AngleStart(0,1,false));
  assert(!Turntable_IsBusy() && Status.arrived && queries==before);

  seed();
  for(unsigned n=0;n<100;n++){tick++;Turntable_Process();}
  assert(Stable && !Status.arrived);
  tick+=2100; /* A missing feedback interval cannot count as stability. */
  response_ready=false;Pending=false;
  for(unsigned n=0;n<40;n++){tick++;Turntable_Process();}
  assert(Turntable_IsBusy() && !Status.arrived);

  seed();Op=OP_VERIFY;
  for(unsigned n=0;n<100;n++){tick++;Turntable_Process();}
  assert(Turntable_IsBusy() && !Status.arrived); /* No prior same-target proof. */
  for(unsigned n=0;n<250 && Turntable_IsBusy();n++){tick++;Turntable_Process();}
  assert(Status.arrived);

  seed();Op=OP_HOME;
  for(unsigned n=0;n<49;n++){tick++;Turntable_Process();}
  assert(!queries); /* HOME retains its independent cadence and verification. */
  seed();Pending=true;PendingToken=7;
  for(unsigned n=0;n<200;n++){tick++;Turntable_Process();}
  assert(!queries); /* Never overlap a queued/sent transaction. */
  puts("turntable_feedback_schedule_test: faster fixed-motion groups, unchanged hold/HOME, no overlap PASS");
}
