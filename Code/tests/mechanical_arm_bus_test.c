#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mechanical_arm.h"
#include "motor_bus.h"
#include "Emm_V5.h"
static uint32_t tick;
static bool route_busy, motion_busy;
bool ChassisRoute_IsBusy(void) { return route_busy; }
bool ChassisMotion_IsBusy(void) { return motion_busy; }
static UART_HandleTypeDef uart = {.Instance=UART5};
UART_HandleTypeDef huart5;
static uint8_t frame[32];
static unsigned tx_count;
static HAL_StatusTypeDef tx_outcome;
uint32_t HAL_GetTick(void)
{
  return tick;
}
bool Mecanum_Test_Stop(void)
{
  return true;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{
  (void)u;
  memcpy(frame, p, n);
  tx_count++;
  return tx_outcome;
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{
  (void)u;
  (void)p;
  (void)n;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u)
{
  (void)u;
  return HAL_OK;
}

#include "motor_arrival_fixture.h"
int main(void)
{
  MechanicalArm_EventTypeDef e;
  uint8_t other[] = {1, 0x35, 0x6b}, ack[] = {5, 0x3a, 3, 0x6b};
  assert(MotorBus_Init(&uart) == HAL_OK);
  assert(MechanicalArm_Init(&uart) == HAL_OK);
  MotorBus_WheelReceiveSet(true);
  fixture_none_modes(&uart,&tick);
  /* Nearest HOME uses the saved mechanical origin directly, without Both
   * preparation or direction-parameter writes. Already-at-zero is accepted
   * as a command response; task-level HOME verification is still required. */
  for(unsigned sample=0;sample<3;sample++) {
    const uint8_t code=sample==0?2:sample==1?0x12:0xe2;
    const uint8_t home_reply[]={5,0x9a,code,0x6b};
    unsigned before=tx_count;
    assert(MechanicalArm_Home(MECHANICAL_ARM_AXIS_BASE,0)==MECHANICAL_ARM_RESULT_NONE);
    tick+=10;MotorBus_Process();
    assert(tx_count==before+1 && frame[0]==5 && frame[1]==0x9a && frame[2]==0 && frame[3]==0);
    MotorBus_TxCpltCallback(&uart);MotorBus_RxBytes(home_reply,sizeof home_reply);
    MotorBus_Process();MechanicalArm_Process();
    assert(MechanicalArm_ResultGet(&e) && e.HomeMode==0 && e.MotorCode==0);
    assert(e.Result==MECHANICAL_ARM_RESULT_OK);
    assert(e.Transmitted && !e.Acknowledged && e.Diagnostic.HomeStage==MECHANICAL_ARM_HOME_TRIGGER);
    assert(e.Diagnostic.Function==0x9a && !e.Diagnostic.ReplyLength);
    assert(!e.Diagnostic.ReplyReceived && tx_count==before+1);
  }
  fixture_none_modes(&uart,&tick);
  /* Base direction HOME: read, preserve settings, set negative direction,
   * verify, then HOME. Driver names need not match the installed top view. */
  {
    tick+=3;MotorBus_Process(); /* None TX handover also applies before parameter reads. */
    uint8_t params[18]={5,0x22,0,1,0,30,0,0,0x27,0x10,0,20,3,0xe8,0,60,0,0x6b};
    const uint8_t set_ack[]={5,0x4c,2,0x6b},home_ack[]={5,0x9a,2,0x6b};
    assert(MechanicalArm_Home(MECHANICAL_ARM_AXIS_BASE,1)==MECHANICAL_ARM_RESULT_NONE);
    MotorBus_Process();assert(frame[1]==0x22);
    MotorBus_TxCpltCallback(&uart);MotorBus_RxBytes(params,sizeof params);
    MotorBus_Process();MechanicalArm_Process();tick+=10;MotorBus_Process();
    assert(frame[1]==0x4c && frame[3]==0 && frame[4]==1 && frame[5]==0);
    assert(!memcmp(frame+6,params+4,13));
    MotorBus_TxCpltCallback(&uart);MotorBus_RxBytes(set_ack,sizeof set_ack);
    MotorBus_Process();MechanicalArm_Process();tick+=10;MotorBus_Process();
    assert(frame[1]==0x22 && !MechanicalArm_ResultGet(&e));
    params[2]=1;params[3]=0;
    MotorBus_TxCpltCallback(&uart);MotorBus_RxBytes(params,sizeof params);
    MotorBus_Process();MechanicalArm_Process();tick+=10;MotorBus_Process();
    assert(frame[1]==0x9a && frame[2]==1);
    MotorBus_TxCpltCallback(&uart);MotorBus_RxBytes(home_ack,sizeof home_ack);
    MotorBus_Process();MechanicalArm_Process();
    assert(MechanicalArm_ResultGet(&e) && e.Result==MECHANICAL_ARM_RESULT_OK && e.HomeMode==1);
    assert(e.Diagnostic.HomeStage==MECHANICAL_ARM_HOME_TRIGGER && e.Diagnostic.Function==0x9a);
    assert(e.Diagnostic.WireTransmitted && !e.Diagnostic.ReplyReceived);
    assert(!e.Diagnostic.ReplyLength && !e.Acknowledged);
    tick+=10;tx_count=0;
  }
  /* Rejected settings, mismatching readback and a stop must never launch HOME. */
  for(unsigned scenario=0;scenario<4;scenario++) {
    uint8_t params[18]={5,0x22,0,1,0,30,0,0,0x27,0x10,0,20,3,0xe8,0,60,0,0x6b};
    uint8_t reply[4]={5,0x4c,scenario==0?0xee:scenario==1?0xe2:2,0x6b};
    assert(MechanicalArm_Home(MECHANICAL_ARM_AXIS_BASE,1)==MECHANICAL_ARM_RESULT_NONE);
    MotorBus_Process();assert(frame[1]==0x22);
    MotorBus_TxCpltCallback(&uart);MotorBus_RxBytes(params,sizeof params);
    MotorBus_Process();MechanicalArm_Process();tick+=10;MotorBus_Process();assert(frame[1]==0x4c);
    MotorBus_TxCpltCallback(&uart);MotorBus_RxBytes(reply,sizeof reply);
    MotorBus_Process();MechanicalArm_Process();
    if(scenario<2) {
      assert(MechanicalArm_ResultGet(&e) && e.Result==MECHANICAL_ARM_RESULT_MOTOR_ERROR && !e.Transmitted);
      assert(e.MotorCode==reply[2] && !e.Acknowledged);
      assert(e.Diagnostic.HomeStage==MECHANICAL_ARM_HOME_PARAMS_WRITE && e.Diagnostic.Function==0x4c);
      assert(e.Diagnostic.BusEventValid && e.Diagnostic.BusResult==MOTOR_BUS_REPLY);
      assert(e.Diagnostic.WireTransmitted && e.Diagnostic.ReplyReceived);
      assert(e.Diagnostic.ReplyLength==4 && !memcmp(e.Diagnostic.Reply,reply,4));
    } else if(scenario==2) {
      tick+=10;MotorBus_Process();assert(frame[1]==0x22);
      MotorBus_TxCpltCallback(&uart);MotorBus_RxBytes(params,sizeof params); /* Old settings. */
      MotorBus_Process();MechanicalArm_Process();
      assert(MechanicalArm_ResultGet(&e) && e.Result==MECHANICAL_ARM_RESULT_MOTOR_ERROR && !e.Transmitted);
      assert(e.MotorCode==0 && !e.Acknowledged);
      assert(e.Diagnostic.HomeStage==MECHANICAL_ARM_HOME_PARAMS_VERIFY && e.Diagnostic.Function==0x22);
      assert(e.Diagnostic.VerifyMismatch && e.Diagnostic.VerifyOffset==0);
      assert(e.Diagnostic.VerifyExpected==1 && e.Diagnostic.VerifyActual==0);
      assert(e.Diagnostic.ReplyLength==18 && !memcmp(e.Diagnostic.Reply,params,18));
    } else {
      assert(MechanicalArm_Stop(MECHANICAL_ARM_AXIS_BASE)==MECHANICAL_ARM_RESULT_NONE);
      for(unsigned n=0;n<20 && MechanicalArm_IsBusy();n++) {
        tick+=10;MotorBus_Process();assert(frame[1]!=0x9a);
        MotorBus_TxCpltCallback(&uart);MotorBus_Process();MechanicalArm_Process();
      }
      assert(MechanicalArm_ResultGet(&e) && e.Action==MECHANICAL_ARM_ACTION_STOP);
      const uint8_t stopped[]={5,0xfe,2,0x6b};
      MotorBus_RxBytes(stopped,sizeof stopped);MotorBus_Process();
    }
    tick+=10;
  }
  assert(MotorBus_Init(&uart)==HAL_OK);
  assert(MechanicalArm_Init(&uart)==HAL_OK);
  tx_count=0;
  /* A full ARM submit slot is not a failed HOME transmission. No retry may
   * create an extra on-wire HOME after the slot becomes available. */
  {
    const uint8_t held[]={6,0x3b,0x6b},home_ack[]={6,0x9a,2,0x6b};
    MotorBus_Event_t held_event;
    assert(MotorBus_Submit(MOTOR_BUS_ARM,held,3,4,0,false));
    assert(MechanicalArm_Home(MECHANICAL_ARM_AXIS_Z,2)==MECHANICAL_ARM_RESULT_BUSY);
    assert(Emm_V5_TxStatusGet()==HAL_BUSY && !tx_count && !MechanicalArm_IsBusy());
    MotorBus_Cancel(MOTOR_BUS_ARM);assert(MotorBus_EventGet(MOTOR_BUS_ARM,&held_event));
    assert(MechanicalArm_Home(MECHANICAL_ARM_AXIS_Z,2)==MECHANICAL_ARM_RESULT_NONE);
    MotorBus_Process();assert(tx_count==1 && frame[0]==6 && frame[1]==0x9a && frame[2]==2);
    MotorBus_TxCpltCallback(&uart);MotorBus_RxBytes(home_ack,sizeof home_ack);
    MotorBus_Process();MechanicalArm_Process();
    assert(MechanicalArm_ResultGet(&e) && e.Result==MECHANICAL_ARM_RESULT_OK);
    tick+=10;
  }
  for (unsigned owner = 0; owner < 2; owner++)
  {
    route_busy = owner == 0; motion_busy = owner == 1;
    assert(MechanicalArm_Enable(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_BUSY);
    assert(MechanicalArm_Disable(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_BUSY);
    assert(MechanicalArm_Position(MECHANICAL_ARM_AXIS_BASE, 10) == MECHANICAL_ARM_RESULT_NONE);
    assert(MechanicalArm_Position(MECHANICAL_ARM_AXIS_X, 10) == MECHANICAL_ARM_RESULT_BUSY);
    MotorBus_Cancel(MOTOR_BUS_ARM);MechanicalArm_Process();assert(MechanicalArm_ResultGet(&e));
    for(unsigned axis=MECHANICAL_ARM_AXIS_BASE;axis<=MECHANICAL_ARM_AXIS_X;axis++)
    {
      assert(MechanicalArm_PositionEx((MechanicalArm_AxisTypeDef)axis,10,5,8,
        MECHANICAL_ARM_POSITION_RELATIVE_CURRENT)==MECHANICAL_ARM_RESULT_NONE);
      MotorBus_Cancel(MOTOR_BUS_ARM);MechanicalArm_Process();assert(MechanicalArm_ResultGet(&e));
    }
    assert(MechanicalArm_HomeStateRead(MECHANICAL_ARM_AXIS_BASE)==MECHANICAL_ARM_RESULT_NONE);
    MotorBus_Cancel(MOTOR_BUS_ARM);MechanicalArm_Process();assert(MechanicalArm_ResultGet(&e));
    assert(MechanicalArm_Home(MECHANICAL_ARM_AXIS_BASE, 0) == MECHANICAL_ARM_RESULT_BUSY);
    assert(MechanicalArm_OriginSet(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_BUSY);
    assert(MechanicalArm_Zero(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_BUSY);
    assert(!MechanicalArm_IsBusy());
    assert(MechanicalArm_PositionRead(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_NONE);
    MotorBus_Cancel(MOTOR_BUS_ARM); MechanicalArm_Process();
    assert(!MechanicalArm_IsBusy()); assert(MechanicalArm_ResultGet(&e));
    assert(MechanicalArm_StateRead(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_NONE);
    MotorBus_Cancel(MOTOR_BUS_ARM); MechanicalArm_Process();
    assert(!MechanicalArm_IsBusy()); assert(MechanicalArm_ResultGet(&e));
    assert(MechanicalArm_Stop(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_NONE);
    MotorBus_Cancel(MOTOR_BUS_ARM); MechanicalArm_Process();
    assert(!MechanicalArm_IsBusy()); assert(MechanicalArm_ResultGet(&e));
    /* Position requests can coexist with a route/turn. The bus still
     * queues behind a reserved wheel batch and completes on the matching ACK. */
    tick+=200;MotorBus_Process();
    assert(MotorBus_Reserve(MOTOR_BUS_CHASSIS));
    unsigned before=tx_count;
    assert(MechanicalArm_PositionEx(MECHANICAL_ARM_AXIS_BASE,800,5,8,
      MECHANICAL_ARM_POSITION_ABSOLUTE)==MECHANICAL_ARM_RESULT_NONE);
    MotorBus_Process();assert(tx_count==before);
    MotorBus_Release(MOTOR_BUS_CHASSIS);
    tick+=10;MotorBus_Process();
    assert(tx_count==before+1 && frame[0]==5 && frame[1]==0xfd);
    assert(frame[6]==0 && frame[7]==0 && frame[8]==3 && frame[9]==0x20 && frame[10]==1);
    const uint8_t position_ack[]={5,0xfd,2,0x6b};
    MotorBus_TxCpltCallback(&uart);MotorBus_RxBytes(position_ack,sizeof position_ack);
    MotorBus_Process();MechanicalArm_Process();
    assert(MechanicalArm_ResultGet(&e) && e.Result==MECHANICAL_ARM_RESULT_OK);
    assert(e.Transmitted && !e.Acknowledged && e.PositionPulses==800);
  }
  route_busy = motion_busy = false;
  /* Routine X holding must not cancel a four-wheel Receive transaction.
   * Reproduce the tracking zero-speed transition without emergency preemption. */
  {
    const uint8_t velocity[]={1,0xf6,0,0,0,20,1,0x6b};
    const uint8_t velocity_ack[]={1,0xf6,2,0x6b};
    MotorBus_Event_t wheel_event;
    tick+=10;
    assert(MotorBus_Submit(MOTOR_BUS_CHASSIS,velocity,sizeof velocity,4,0,false));
    MotorBus_Process();MotorBus_TxCpltCallback(&uart);MotorBus_Process();
    assert(MechanicalArm_StopScheduled(MECHANICAL_ARM_AXIS_X)==MECHANICAL_ARM_RESULT_NONE);
    MotorBus_RxBytes(velocity_ack,sizeof velocity_ack);MotorBus_Process();
    assert(MotorBus_EventGet(MOTOR_BUS_CHASSIS,&wheel_event));
    assert(wheel_event.result==MOTOR_BUS_REPLY);
    tick+=10;MotorBus_Process();
    assert(frame[0]==7 && frame[1]==0xfe && frame[2]==0x98 && frame[3]==0);
    MotorBus_TxCpltCallback(&uart);MotorBus_Process();MechanicalArm_Process();
    assert(MechanicalArm_ResultGet(&e) && e.Result==MECHANICAL_ARM_RESULT_OK && !e.Acknowledged);
    { const uint8_t hold_ack[]={7,0xfe,2,0x6b};MotorBus_RxBytes(hold_ack,4); }
    MotorBus_Process();MechanicalArm_Process();
    assert(!MechanicalArm_ResultGet(&e)); /* Late ACK cannot create a second completion. */
    assert(!MotorBus_IsQuarantined());
    tick+=10;
  }
  assert(MechanicalArm_StopScheduled(MECHANICAL_ARM_AXIS_ALL)==MECHANICAL_ARM_RESULT_PARAM_ERROR);
  assert(MotorBus_Submit(MOTOR_BUS_CHASSIS, other, 3, 0, 0, false));
  assert(MechanicalArm_StateRead(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_NONE);
  MotorBus_Process();
  MotorBus_TxCpltCallback(&uart);
  { const uint8_t other_reply[]={1,0x35,0,0,0,0x6b}; MotorBus_RxBytes(other_reply,6); }
  MotorBus_Process();
  MechanicalArm_Process();
  assert(!MechanicalArm_ResultGet(&e));
  tick += 10;
  MotorBus_Process();
  assert(frame[0] == 5 && frame[1] == 0x3a);
  MotorBus_TxCpltCallback(&uart);
  MotorBus_Process();
  MechanicalArm_Process();
  assert(!MechanicalArm_ResultGet(&e));
  MotorBus_RxBytes(ack, 2);
  MotorBus_Process();
  MechanicalArm_Process();
  assert(!MechanicalArm_ResultGet(&e));
  MotorBus_RxBytes(ack + 2, 2);
  MotorBus_Process();
  MechanicalArm_Process();
  assert(MechanicalArm_ResultGet(&e) && e.Result == MECHANICAL_ARM_RESULT_OK);
  /* Installed Z: protocol CCW moves down; positive/up must transmit CW.
   * Exercise console and explicit-speed paths with both signs. */
  for (unsigned explicit_speed = 0; explicit_speed < 2; explicit_speed++)
  {
    for (unsigned negative = 0; negative < 2; negative++)
    {
      const int32_t pulses = negative ? -89 : 89;
      const uint8_t position_ack[] = {6, 0xfd, 2, 0x6b};
      assert((explicit_speed ? MechanicalArm_PositionEx(MECHANICAL_ARM_AXIS_Z,
                 pulses, 10, 20, MECHANICAL_ARM_POSITION_RELATIVE_CURRENT)
               : MechanicalArm_Position(MECHANICAL_ARM_AXIS_Z, pulses)) == MECHANICAL_ARM_RESULT_NONE);
      tick += 10;
      MotorBus_Process();
      assert(frame[0] == 6 && frame[1] == 0xfd && frame[2] == negative);
      /* Default motion must carry the selected speed/acceleration on wire;
       * explicit-speed callers retain their own parameters. */
      assert((((unsigned)frame[3] << 8) | frame[4]) == (explicit_speed ? 10U : 2250U));
      MechanicalArm_ConfigTypeDef selected_config;
      assert(MechanicalArm_ConfigGet(MECHANICAL_ARM_AXIS_Z,&selected_config));
      assert(frame[5] == (explicit_speed ? 20U : selected_config.Acceleration));
      assert(frame[6] == 0 && frame[7] == 0 && frame[8] == 0 && frame[9] == 89);
      MotorBus_TxCpltCallback(&uart);
      MotorBus_RxBytes(position_ack, sizeof(position_ack));
      MotorBus_Process(); MechanicalArm_Process();
      assert(MechanicalArm_ResultGet(&e) && e.Result == MECHANICAL_ARM_RESULT_OK);
    }
  }
  /* Absolute origin is a valid destination; a relative zero is still a no-op error. */
  assert(MechanicalArm_PositionEx(MECHANICAL_ARM_AXIS_X, 0, 8, 8,
           MECHANICAL_ARM_POSITION_RELATIVE_CURRENT) == MECHANICAL_ARM_RESULT_PARAM_ERROR);
  assert(MechanicalArm_PositionEx(MECHANICAL_ARM_AXIS_BASE, 0, 8, 0,
           MECHANICAL_ARM_POSITION_ABSOLUTE) == MECHANICAL_ARM_RESULT_PARAM_ERROR);
  assert(MechanicalArm_PositionEx(MECHANICAL_ARM_AXIS_X, 0, 8, 0,
           MECHANICAL_ARM_POSITION_ABSOLUTE) == MECHANICAL_ARM_RESULT_NONE);
  tick += 10; MotorBus_Process();
  assert(frame[0] == 7 && frame[1] == 0xfd && frame[10] == 1 && frame[5] == 0);
  assert(frame[6] == 0 && frame[7] == 0 && frame[8] == 0 && frame[9] == 0);
  MotorBus_TxCpltCallback(&uart);
  const uint32_t x_tx_tick=tick;
  tick+=3; /* ACK arrival must not be reported as UART TX completion. */
  { const uint8_t zero_ack[] = {7, 0xfd, 2, 0x6b}; MotorBus_RxBytes(zero_ack, 4); }
  MotorBus_Process(); MechanicalArm_Process();
  assert(MechanicalArm_ResultGet(&e) && e.Result == MECHANICAL_ARM_RESULT_OK);
  assert(e.Transmitted && e.TxDoneMs==x_tx_tick && e.SpeedRpm==8 && e.PositionPulses==0);
  /* Emm home flags are raw flags, not a command ACK code. */
  assert(MechanicalArm_HomeStateRead(MECHANICAL_ARM_AXIS_Z)==MECHANICAL_ARM_RESULT_NONE);
  tick+=10; MotorBus_Process(); assert(frame[0]==6 && frame[1]==0x3b);
  MotorBus_TxCpltCallback(&uart);
  { const uint8_t flags[]={6,0x3b,7,0x6b}; MotorBus_RxBytes(flags,4); }
  MotorBus_Process(); MechanicalArm_Process();
  assert(MechanicalArm_ResultGet(&e) && e.Result==MECHANICAL_ARM_RESULT_OK && e.StateFlags[MECHANICAL_ARM_AXIS_Z]==7);
  assert(MechanicalArm_HomeAbort(MECHANICAL_ARM_AXIS_Z)==MECHANICAL_ARM_RESULT_NONE);
  tick+=10; MotorBus_Process(); assert(frame[0]==6 && frame[1]==0x9c && frame[2]==0x48);
  MotorBus_TxCpltCallback(&uart);
  { const uint8_t ack_home[]={6,0x9c,2,0x6b}; MotorBus_RxBytes(ack_home,4); }
  MotorBus_Process(); MechanicalArm_Process();
  assert(MechanicalArm_ResultGet(&e) && e.Result==MECHANICAL_ARM_RESULT_OK);
  /* Read actual signed speed through the normal ARM bus owner (0x35). */
  for(unsigned sample=0;sample<3;sample++) {
    int32_t expected=sample==0?0:sample==1?30:-30;
    assert(MechanicalArm_SpeedRead(MECHANICAL_ARM_AXIS_BASE)==MECHANICAL_ARM_RESULT_NONE);
    tick+=10;MotorBus_Process();assert(frame[0]==5 && frame[1]==0x35);
    MotorBus_TxCpltCallback(&uart);
    const uint8_t speed[]={5,0x35,sample==2?1:0,0,sample==0?0:30,0x6b};
    MotorBus_RxBytes(speed,sizeof speed);MotorBus_Process();MechanicalArm_Process();
    assert(MechanicalArm_ResultGet(&e) && e.Result==MECHANICAL_ARM_RESULT_OK);
    assert(e.Action==MECHANICAL_ARM_ACTION_SPEED_READ && e.CurrentSpeedRpm==expected);
  }
  assert(MechanicalArm_SpeedRead(MECHANICAL_ARM_AXIS_ALL)==MECHANICAL_ARM_RESULT_PARAM_ERROR);
  assert(MechanicalArm_StateRead(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_NONE);
  tick += 10;
  MotorBus_Process();
  MotorBus_TxCpltCallback(&uart);
  MotorBus_Process();
  tick += 100;
  MotorBus_Process();
  MechanicalArm_Process();
  assert(MechanicalArm_ResultGet(&e) && e.Result == MECHANICAL_ARM_RESULT_ACK_TIMEOUT);
  assert(e.Diagnostic.BusEventValid && e.Diagnostic.BusResult==MOTOR_BUS_TIMEOUT);
  assert(e.Diagnostic.Function==0x3a && e.Diagnostic.HomeStage==MECHANICAL_ARM_HOME_NONE);
  assert(e.Diagnostic.WireTransmitted && !e.Diagnostic.ReplyReceived && !e.Diagnostic.ReplyLength);
  unsigned before_locked_home=tx_count;
  assert(MechanicalArm_Home(MECHANICAL_ARM_AXIS_Z,2)==MECHANICAL_ARM_RESULT_TX_ERROR);
  assert(tx_count==before_locked_home && !MechanicalArm_IsBusy()); /* Rejected before Emm TX. */
  assert(MechanicalArm_Stop(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_NONE);
  tick += 10;
  MotorBus_Process();
  assert(frame[1] == 0xfe);
  MotorBus_TxCpltCallback(&uart);
  MotorBus_Process();
  MechanicalArm_Process();
  assert(MechanicalArm_ResultGet(&e) && e.Result == MECHANICAL_ARM_RESULT_OK);
  /* Even after a reply timeout quarantines ordinary traffic, homing abort must transmit. */
  assert(MechanicalArm_HomeAbort(MECHANICAL_ARM_AXIS_Z)==MECHANICAL_ARM_RESULT_NONE);
  tick+=10; MotorBus_Process(); assert(frame[1]==0x9c && frame[2]==0x48);
  MotorBus_TxCpltCallback(&uart); MotorBus_Process(); MechanicalArm_Process();
  assert(MechanicalArm_ResultGet(&e) && e.Result==MECHANICAL_ARM_RESULT_OK && e.Transmitted && !e.Acknowledged);
  puts(
      "mechanical_arm_bus_test: unrelated TX cannot complete arm, fragmented own ACK completes OK");
  return 0;
}
