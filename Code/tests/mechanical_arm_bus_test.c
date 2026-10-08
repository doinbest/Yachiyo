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
int main(void)
{
  MechanicalArm_EventTypeDef e;
  uint8_t other[] = {1, 0x35, 0x6b}, ack[] = {5, 0xf3, 2, 0x6b};
  assert(MotorBus_Init(&uart) == HAL_OK);
  assert(MechanicalArm_Init(&uart) == HAL_OK);
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
    assert(e.Transmitted && e.Acknowledged && e.PositionPulses==800);
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
    assert(!MechanicalArm_ResultGet(&e));
    { const uint8_t hold_ack[]={7,0xfe,2,0x6b};MotorBus_RxBytes(hold_ack,4); }
    MotorBus_Process();MechanicalArm_Process();
    assert(MechanicalArm_ResultGet(&e) && e.Result==MECHANICAL_ARM_RESULT_OK && e.Transmitted);
    assert(!MotorBus_IsQuarantined());
    tick+=10;
  }
  assert(MechanicalArm_StopScheduled(MECHANICAL_ARM_AXIS_ALL)==MECHANICAL_ARM_RESULT_PARAM_ERROR);
  assert(MotorBus_Submit(MOTOR_BUS_CHASSIS, other, 3, 0, 0, false));
  assert(MechanicalArm_Enable(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_NONE);
  MotorBus_Process();
  MotorBus_TxCpltCallback(&uart);
  { const uint8_t other_reply[]={1,0x35,0,0,0,0x6b}; MotorBus_RxBytes(other_reply,6); }
  MotorBus_Process();
  MechanicalArm_Process();
  assert(!MechanicalArm_ResultGet(&e));
  tick += 10;
  MotorBus_Process();
  assert(frame[0] == 5 && frame[1] == 0xf3);
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
      assert((((unsigned)frame[3] << 8) | frame[4]) == (explicit_speed ? 10U : 500U));
      assert(frame[5] == (explicit_speed ? 20U : 120U));
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
  assert(MechanicalArm_PositionEx(MECHANICAL_ARM_AXIS_X, 0, 8, 8,
           MECHANICAL_ARM_POSITION_ABSOLUTE) == MECHANICAL_ARM_RESULT_NONE);
  tick += 10; MotorBus_Process();
  assert(frame[0] == 7 && frame[1] == 0xfd && frame[10] == 1);
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
  assert(MechanicalArm_Enable(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_NONE);
  tick += 10;
  MotorBus_Process();
  MotorBus_TxCpltCallback(&uart);
  MotorBus_Process();
  tick += 100;
  MotorBus_Process();
  MechanicalArm_Process();
  assert(MechanicalArm_ResultGet(&e) && e.Result == MECHANICAL_ARM_RESULT_ACK_TIMEOUT);
  assert(MechanicalArm_Home(MECHANICAL_ARM_AXIS_Z,2)==MECHANICAL_ARM_RESULT_TX_ERROR);
  assert(Emm_V5_TxStatusGet()==HAL_ERROR); /* A locked bus must not retry as BUSY. */
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
