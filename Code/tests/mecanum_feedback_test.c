#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mecanum_chassis.h"
#include "motor_bus.h"
#include "hwt101_calibration.h"
#include "chassis_localization.h"
#include "chassis_motion.h"
static uint32_t tick;
static UART_HandleTypeDef uart;
static uint8_t frame[32];
static unsigned count;
static bool flying;
static bool drop_replies;
static bool acknowledge_motion;
static bool record_arrival;
static unsigned arrival_speed_reads,arrival_position_reads,arrival_state_reads;
static unsigned stop_count;
static bool all_wheels, reply_pending;
static uint8_t reply_address, reply_function;
static uint32_t reply_due, reply_delay;
static ChassisMotion_Status_t motion;
bool ChassisRoute_IsBusy(void) { return false; }
bool ChassisMotion_IsBusy(void) { return false; }
void ChassisMotion_StatusGet(ChassisMotion_Status_t *out) { *out = motion;out->heading_sample_tick=tick; }
bool ChassisMotion_AnchorSet(float yaw)
{
  motion.heading_valid = motion.map_anchor_valid = true;
  motion.map_yaw_rad = yaw;
  return true;
}
uint32_t HAL_GetTick(void)
{
  return tick;
}
bool HWT101_Cal_IsBusy(void)
{
  return false;
}
bool HWT101_Cal_ControlAngleGet(HWT101_Angle_t *a)
{
  (void)a;
  return false;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{
  (void)u;
  if (n == 3 && !all_wheels)
    assert(p[0] == 2);
  if (p[1] == 0xfe)
    stop_count++;
  memcpy(frame, p, n);
  if(record_arrival && n==3){
    if(p[1]==0x35)arrival_speed_reads++;
    else if(p[1]==0x36)arrival_position_reads++;
    else if(p[1]==0x3a)arrival_state_reads++;
  }
  count++;
  #ifdef UART5_DEBUG
  if(tick>690 && tick<1300)fprintf(stderr,"wire t=%lu id=%u fn=%02x\n",(unsigned long)tick,p[0],p[1]);
  #endif
  flying = true;
  return HAL_OK;
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
static void step(void)
{
  uint8_t p[8] = {2, 0, 1, 0xff, 0xff, 0xff, 0xff, 0x6b};
  unsigned n;
  tick++;
  if (flying)
  {
    flying = false;
    MotorBus_TxCpltCallback(&uart);
    if(frame[1]==0xfe) {
      const uint8_t stopped[]={frame[0],0xfe,2,0x6b};MotorBus_RxBytes(stopped,4);
    }
    else if (!drop_replies && (frame[1] < 0x40 || (acknowledge_motion && (frame[1]==0xf6 || frame[1]==0xfd || frame[1]==0xff))))
    {
      reply_pending = true;
      reply_due = tick + reply_delay;
      reply_address = frame[0]?frame[0]:1;
      reply_function = frame[1];
    }
  }
  if (reply_pending && tick >= reply_due)
  {
    reply_pending = false;
    p[0] = reply_address;
    p[1] = reply_function;
    n = p[1] == 0x36 ? 8 : p[1] == 0x35 ? 6 : 4;
    if(p[1]>=0xf3)p[2]=2;
    p[n - 1] = 0x6b;
    MotorBus_RxBytes(p, 2);
    MotorBus_RxBytes(p + 2, (uint16_t)(n - 2));
  }
  MotorBus_Process();
  Mecanum_Process();
}
/* Actual bus arbitration + feedback parser + observer, with a simulated driver.
 * TX completes after 1ms; reply_delay models driver turnaround, not real hardware. */
static bool request_planned_stop(void)
{ return Mecanum_RequestPlannedStop(); }
static void continuous_feedback_test(uint32_t delay)
{
  ChassisLocalization_Status_t position;
  uint32_t generation, sequence, previous_tick, max_group_gap = 0, max_span = 0;
  unsigned groups = 0;
  Mecanum_Feedback_Enable(false);
  for (unsigned i = 0; i < 30; i++) step();
  assert(Mecanum_RecoveryAfterReset());
  assert(Mecanum_Feedback_Select(0));
  all_wheels = true; drop_replies = false; reply_delay = delay; acknowledge_motion=true;
  ChassisLocalization_Init();
  assert(ChassisLocalization_ConfirmPositionUnits(65536));
  assert(ChassisLocalization_Origin(0, 0, 0));
  Mecanum_Feedback_Enable(true);
  for (unsigned i = 0; i < 600; i++)
  {
    step(); ChassisLocalization_Process();
  }
  ChassisLocalization_Get(&position);
  assert(position.feedback_valid && position.speed_valid);
  generation = position.generation;
  sequence = position.feedback_sequence;
  previous_tick = position.feedback_tick;
  for (unsigned i = 0; i < 2400; i++)
  {
    if (i % 20 == 0) assert(Mecanum_Velocity_Request(50, 0, 0, 0));
    step(); ChassisLocalization_Process(); ChassisLocalization_Get(&position);
    /* Every foreground snapshot must remain usable, including partial groups. */
    assert(position.feedback_valid && position.speed_valid);
    assert(position.generation == generation);
    assert(tick - position.feedback_tick <= 600);
    for (unsigned wheel = 0; wheel < 4; wheel++)
    {
      assert(tick - position.wheels[wheel].position_ms <= 600);
      assert(tick - position.wheels[wheel].speed_ms <= 600);
    }
    if (position.feedback_sequence != sequence)
    {
      uint32_t gap = position.feedback_tick - previous_tick;
      assert(position.feedback_sequence == sequence + 1);
      assert(position.observer.feedback_span_ms <= 250);
      if (gap > max_group_gap) max_group_gap = gap;
      if (position.observer.feedback_span_ms > max_span) max_span = position.observer.feedback_span_ms;
      previous_tick = position.feedback_tick;
      sequence = position.feedback_sequence;
      groups++;
    }
  }
  assert(groups >= 5 && max_group_gap <= 600);
  assert(!Mecanum_RequestPlannedStop()); /* Never treat a nonzero snapshot as a stop. */
  /* Send a zero snapshot; wait for completion and an active query to drain.
   * The planned stop must not cancel the protocol's unsequenced query reply. */
  {
    Mecanum_Status_t status;
    bool zero_sent = false, saw_query_busy = false, clean_stop = false;
    assert(Mecanum_Velocity_Request(0, 0, 0, 0));
    for (unsigned i = 0; i < 1000; i++)
    {
      step();
      Mecanum_StatusGet(&status);
      zero_sent = status.stage == MECANUM_STAGE_SENT && status.tx_complete &&
                  status.sent_velocity_valid && status.sent_rpm[0] == 0 &&
                  status.sent_rpm[1] == 0 && status.sent_rpm[2] == 0 && status.sent_rpm[3] == 0;
      if (zero_sent && MotorBus_OwnerInFlight(MOTOR_BUS_FEEDBACK))
      {
        saw_query_busy = true;
        assert(!Mecanum_CanStopCleanly());
        unsigned before=stop_count;
        assert(request_planned_stop()); /* Accept intent while a query is active. */
        assert(stop_count==before); /* Do not send FE over that query's reply. */
        clean_stop=true;
        break;
      }
    }
    assert(clean_stop && saw_query_busy);
    for (unsigned i = 0; i < 100; i++) {
      step();ChassisLocalization_Process();ChassisLocalization_Get(&position);
      /* Cancelling a queued query to send stop is not loss of the last
       * valid measurement. It must not destroy the odometry generation. */
      assert(position.feedback_valid && position.speed_valid);
      assert(position.generation==generation);
    }
    Mecanum_StatusGet(&status);
    assert(status.stage == MECANUM_STAGE_STOPPED && !status.locked);
    assert(status.error == MECANUM_ERROR_NONE && !MotorBus_IsQuarantined());
  }
  printf("feedback schedule: reply delay=%lums, groups=%u, max gap=%lums, max span=%lums PASS\n",
         (unsigned long)delay, groups, (unsigned long)max_group_gap, (unsigned long)max_span);
}
/* Selection changes the NEXT poll, not the identity of an outstanding read.
 * Cover queued TX, driver reply wait and a reply waiting for its consumer. */
static void select_with_outstanding_feedback(unsigned phase)
{
  Mecanum_Feedback_t old;
  unsigned saved_count, saved_stops;
  uint32_t selected_at;
  Mecanum_Feedback_Enable(false);
  for(unsigned i=0;i<60;i++)step();
  assert(Mecanum_RecoveryAfterReset());
  all_wheels=true;drop_replies=false;reply_delay=8;acknowledge_motion=true;
  assert(Mecanum_Feedback_Select(2));
  Mecanum_Feedback_Enable(true);
  Mecanum_Process(); /* First speed query queued for ID 2. */
  assert(MotorBus_OwnerBusy(MOTOR_BUS_FEEDBACK));
  if(phase)
  {
    for(unsigned i=0;i<30 && !reply_pending;i++)step();
    assert(reply_pending && reply_address==2 && reply_function==0x35);
  }
  if(phase==2)
  {
    const uint8_t reply[]={2,0x35,0,0,42,0x6b};
    tick=reply_due;reply_pending=false;
    MotorBus_RxBytes(reply,sizeof reply);MotorBus_Process();
    assert(!MotorBus_OwnerInFlight(MOTOR_BUS_FEEDBACK));
    assert(MotorBus_OwnerBusy(MOTOR_BUS_FEEDBACK)); /* Unconsumed reply. */
  }
  selected_at=tick;saved_count=count;saved_stops=stop_count;
  assert(Mecanum_Feedback_Select(4)); /* Must not wait for an idle polling gap. */
  assert(count==saved_count && stop_count==saved_stops);
  for(unsigned i=0;i<40;i++)step();
  assert(!MotorBus_OwnerBusy(MOTOR_BUS_FEEDBACK));
  assert(Mecanum_FeedbackGet(1,&old) && old.speed_valid);
  assert(old.speed_ms>=selected_at); /* Old answer still belongs to ID 2. */
  if(phase==2)assert(old.speed_rpm==42);
  saved_count=count;
  for(unsigned i=0;i<20;i++)step();
  assert(count==saved_count); /* Select retains its explicit Enable contract. */
  Mecanum_Feedback_Enable(true);
  for(unsigned i=0;i<120;i++)step();
  assert(frame[0]==4 && count>saved_count && stop_count==saved_stops);
  assert(Mecanum_FeedbackGet(1,&old) && old.speed_ms>=selected_at);
  selected_at=tick;
  assert(MotorBus_OwnerBusy(MOTOR_BUS_FEEDBACK));
  assert(Mecanum_Feedback_Select(0));Mecanum_Feedback_Enable(true);
  for(unsigned i=0;i<220;i++)step();
  for(unsigned w=0;w<4;w++)
  {
    assert(Mecanum_FeedbackGet(w,&old));
    assert(old.speed_valid && old.position_valid && old.state_valid);
    assert(old.speed_ms>selected_at && old.position_ms>selected_at && old.state_ms>selected_at);
  }
  saved_count=count;
  assert(!Mecanum_Feedback_Select(5));
  for(unsigned i=0;i<40;i++)step();
  assert(count>saved_count && stop_count==saved_stops);
  printf("feedback selection: pending phase=%u, old reply retained, single/all wheel resume PASS\n",phase);
}
int main(void)
{
  Mecanum_Feedback_t f;
  MotorBus_Init(&uart);
  for (unsigned i = 0; i < 300; i++)
    step();
  assert(count == 0);
  assert(Mecanum_Feedback_Select(2));
  Mecanum_Feedback_Enable(true);
  for (unsigned i = 0; i < 210; i++)
    step();
  assert(Mecanum_FeedbackGet(1, &f));
  assert(f.speed_valid && f.position_valid && f.state_valid);
  assert(f.speed_rpm == -65535 && f.position_raw == -4294967295LL && f.state_flags == 1);
  assert(f.speed_ms <= tick && f.position_ms <= tick && f.state_ms <= tick);
  assert(Mecanum_FeedbackGet(0, &f) && !f.speed_valid && !f.position_valid);
  Mecanum_Feedback_Enable(false);
  for (unsigned i = 0; i < 20; i++)
    step();
  /* A planned stop cancels unsent feedback queries. Keep the last sample,
   * including its original timestamp; cancellation is not a bad reply. */
  {
    Mecanum_Feedback_t before, after;
    const uint8_t query[] = {2, 0x36, 0x6b};
    assert(Mecanum_FeedbackGet(1, &before) && before.position_valid);
    assert(!MotorBus_OwnerBusy(MOTOR_BUS_FEEDBACK));
    assert(MotorBus_Submit(MOTOR_BUS_FEEDBACK, query, 3, 8, 4, false));
    MotorBus_Cancel(MOTOR_BUS_FEEDBACK);
    Mecanum_Process();
    assert(Mecanum_FeedbackGet(1, &after));
    assert(after.position_valid);
    assert(after.position_raw == before.position_raw);
    assert(after.position_ms == before.position_ms);
  }
  unsigned saved = count;
  for (unsigned i = 0; i < 300; i++)
    step();
  assert(count == saved);
  Mecanum_Status_t status;
  assert(Mecanum_AckProfile_Set(MECANUM_ACK_NONE));
  assert(Mecanum_Feedback_Select(2));
  drop_replies = true;
  Mecanum_Feedback_Enable(true);
  for (unsigned i = 0; i < 130; i++)
    step();
  Mecanum_StatusGet(&status);
  assert(status.locked && status.error == MECANUM_ERROR_TIMEOUT);
  assert(!Mecanum_Velocity_Request(100, 0, 0, 0));
  assert(stop_count == 0); /* Idle feedback failure does not issue motion commands. */
  Mecanum_Feedback_Enable(false);
  assert(Mecanum_RecoveryAfterReset());
  Mecanum_StatusGet(&status);
  assert(!status.locked && status.error == MECANUM_ERROR_NONE);
  assert(Mecanum_Velocity_Request(100, 0, 0, 0));
  for (unsigned i = 0; i < 60; i++)
    step();
  Mecanum_Feedback_Enable(true);
  for (unsigned i = 0; i < 180; i++)
    step();
  Mecanum_StatusGet(&status);
  assert(status.locked && status.error == MECANUM_ERROR_TIMEOUT);
  assert(status.stage == MECANUM_STAGE_STOPPED && stop_count == 4);
  assert(!Mecanum_Velocity_Request(100, 0, 0, 0));
  continuous_feedback_test(0);
  continuous_feedback_test(8);
  /* Emergency Stop during an actual read reply wait must resume feedback. */
  reply_delay = 30;
  for (unsigned i = 0; i < 300 && !reply_pending; i++) step();
  assert(reply_pending && MotorBus_OwnerBusy(MOTOR_BUS_FEEDBACK));
  saved = stop_count;
  assert(Mecanum_Test_Stop());
  for (unsigned i = 0; i < 800; i++) step();
  Mecanum_StatusGet(&status);
  assert(stop_count == saved + 4);
  assert(!status.locked && status.error == MECANUM_ERROR_NONE);
  for (unsigned i = 0; i < 4; i++)
  {
    assert(Mecanum_FeedbackGet(i, &f));
    assert(f.speed_valid && f.position_valid && f.state_valid);
    assert(tick - f.speed_ms < 600 && tick - f.position_ms < 600);
  }
  for(unsigned phase=0;phase<3;phase++)select_with_outstanding_feedback(phase);
  /* A normal position group delays only the next velocity batch, never FE.
   * Stop halfway through ID1..4 and prove later explicit motion still works. */
  reply_delay=2;
  assert(Mecanum_Velocity_Request(100,0,0,0));
  for(unsigned i=0;i<2000 && !(reply_pending && reply_function==0x36 && reply_address==2);i++) step();
  assert(reply_pending && reply_function==0x36 && reply_address==2);
  saved=stop_count;assert(Mecanum_Test_Stop());
  for(unsigned i=0;i<800;i++) step();
  Mecanum_StatusGet(&status);
  assert(stop_count==saved+4 && !status.locked && status.tx_complete);
  assert(Mecanum_Velocity_Request(0,0,0,0));
  for(unsigned i=0;i<200;i++) step();
  Mecanum_StatusGet(&status);
  assert(status.tx_complete && status.sent_velocity_valid && !status.locked);
  /* Normal arrival prioritizes actual speed/position; manual stop retains its scan. */
  all_wheels=true;drop_replies=false;reply_delay=0;
  assert(Mecanum_Feedback_Select(0));Mecanum_Feedback_Enable(true);
  Mecanum_Feedback_Arrival(true);for(unsigned i=0;i<30;i++)step();
  arrival_speed_reads=arrival_position_reads=arrival_state_reads=0;record_arrival=true;
  for(unsigned i=0;i<200;i++)step();
  assert(arrival_speed_reads>=8 && arrival_position_reads>=8 && !arrival_state_reads);
  Mecanum_Feedback_StopMonitor(true);
  for(unsigned i=0;i<200;i++)step();
  assert(arrival_state_reads>0);
  Mecanum_Feedback_StopMonitor(false);Mecanum_Feedback_Arrival(false);record_arrival=false;
  puts("feedback position group: urgent stop and later explicit command PASS");
  puts("mecanum_feedback_test: opt-in single wheel, signed speed, full 32-bit magnitude, "
       "validity/timestamps OK");
  return 0;
}
