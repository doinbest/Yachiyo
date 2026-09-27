#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mecanum_chassis.h"
#include "motor_bus.h"
#include "hwt101_calibration.h"
static uint32_t tick;
static UART_HandleTypeDef uart;
static uint8_t frame[100][32], lengths[100];
static unsigned count;
static bool in_flight;
static HAL_StatusTypeDef tx_result;
bool ChassisRoute_IsBusy(void) { return false; }
bool ChassisMotion_IsBusy(void) { return false; }
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
  memcpy(frame[count], p, n);
  lengths[count++] = (uint8_t)n;
  in_flight = tx_result == HAL_OK;
  return tx_result;
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
  tick++;
  if (in_flight)
  {
    in_flight = false;
    MotorBus_TxCpltCallback(&uart);
  }
  MotorBus_Process();
  Mecanum_Process();
}
static void advance(unsigned n)
{
  while (n--)
    step();
}
int main(void)
{
  Mecanum_Status_t s;
  MotorBus_Init(&uart);
  Mecanum_StatusGet(&s); assert(s.ack_profile == MECANUM_ACK_RECEIVE);
  assert(Mecanum_AckProfile_Set(MECANUM_ACK_NONE));
  assert(Mecanum_Velocity_Request(20000, 0, 0, 0));
  Mecanum_StatusGet(&s);assert(s.motion_sequence == 1 && s.stop_sequence == 0);
  advance(15);
  assert(Mecanum_Velocity_Request(200, 0, 0, 0));
  Mecanum_StatusGet(&s);assert(s.motion_sequence == 2);
  advance(50);
  assert(count == 5);
  for (unsigned i = 0; i < 4; i++)
  {
    assert(frame[i][0] == i + 1 && frame[i][1] == 0xf6 && frame[i][6] == 1);
    assert(frame[i][3] == 0x0b && frame[i][4] == 0xb8); /* Emm F6 maximum: 3000 RPM. */
  }
  assert(frame[4][0] == 0 && frame[4][1] == 0xff);
  advance(90);
  assert(count == 10);
  assert(frame[5][4] == 48);
  assert(Mecanum_Test_Stop());
  Mecanum_StatusGet(&s);assert(s.stop_sequence == 1 && !s.tx_complete);
  assert(Mecanum_Test_Stop());
  Mecanum_StatusGet(&s);assert(s.stop_sequence == 1); /* Same pending dispatch. */
  assert(Mecanum_IsBusy());
  advance(50);
  Mecanum_StatusGet(&s);
  assert(s.stage == MECANUM_STAGE_STOPPED && !s.stop_pending);
  assert(s.tx_complete && s.sent_ms > 0 && s.stop_sequence == 1 && s.motion_sequence == 2);
  assert(!Mecanum_IsBusy());
  assert(Mecanum_Move_Control(10, 0, 0));
  Mecanum_StatusGet(&s);assert(s.motion_sequence == 3);
  advance(15);
  unsigned before = count;
  assert(Mecanum_Test_Stop());
  advance(60);
  for (unsigned i = before; i < count; i++)
    assert(frame[i][1] != 0xff);
  Mecanum_StatusGet(&s);
  assert(s.locked);
  assert(!Mecanum_Move_Control(10, 0, 0));
  assert(Mecanum_RecoveryAfterReset());
  assert(Mecanum_Velocity_Request(100, 0, 0, 0));
  advance(12);
  tx_result = HAL_ERROR;
  advance(80);
  Mecanum_StatusGet(&s);
  assert(s.locked && s.stage == MECANUM_STAGE_FAULT);
  /* Failed first stop must not prevent attempts to stop the other three wheels. */
  unsigned stop_attempts = 0;
  for (unsigned i = 0; i < count; i++)
    if (frame[i][1] == 0xfe)
      stop_attempts++;
  assert(stop_attempts == 12);
  tx_result = HAL_OK;
  assert(Mecanum_Test_Stop());
  advance(60);
  assert(Mecanum_RecoveryAfterReset());
  assert(Mecanum_AckProfile_Set(MECANUM_ACK_RECEIVE));
  unsigned base = count;
  assert(Mecanum_Velocity_Request(100, 0, 0, 0));
  advance(15);
  assert(count == base + 1);
  uint8_t wrong[] = {2, 0xf6, 2, 0x6b, 1, 0xfd, 2, 0x6b};
  MotorBus_RxBytes(wrong, sizeof(wrong));
  MotorBus_Process();
  Mecanum_Process();
  assert(count == base + 1);
  for (unsigned i = 0; i < 4; i++)
  {
    uint8_t ack[] = {(uint8_t)(i + 1), 0xf6, 2, 0x6b};
    assert(count == base + i + 1 && frame[base + i][0] == i + 1);
    MotorBus_RxBytes(ack, 2);
    MotorBus_Process();
    Mecanum_Process();
    assert(count == base + i + 1);
    MotorBus_RxBytes(ack + 2, 2);
    MotorBus_Process();
    Mecanum_Process();
    advance(15);
  }
  assert(count == base + 5 && frame[base + 4][1] == 0xff);
  Mecanum_StatusGet(&s);
  assert(s.tx_complete && s.acknowledged && s.acknowledged_wheels == 4);
  assert(s.sent_velocity_valid && s.sent_rpm[0] == 24);
  assert(Mecanum_Test_Stop());
  advance(60);
  assert(Mecanum_RecoveryAfterReset());
  base = count;
  assert(Mecanum_Velocity_Request(100, 0, 0, 0));
  advance(15);
  uint8_t nack[] = {1, 0xf6, 0xe2, 0x6b};
  MotorBus_RxBytes(nack, 4);
  MotorBus_Process();
  Mecanum_Process();
  advance(60);
  Mecanum_StatusGet(&s);
  assert(s.locked && s.stage == MECANUM_STAGE_STOPPED && s.error == MECANUM_ERROR_ACK);
  for (unsigned i = base; i < count; i++)
    assert(frame[i][1] != 0xff);
  assert(count == base + 5); /* Failed batch automatically attempts all four stops. */
  assert(Mecanum_RecoveryAfterReset());
  assert(Mecanum_Velocity_Request(20, 0, 0, 0));
  advance(15); /* First velocity frame is waiting for its ACK. */
  assert(Mecanum_Test_Stop());
  advance(60);
  Mecanum_StatusGet(&s);
  assert(s.locked && s.error == MECANUM_ERROR_CANCELLED && s.tx_complete);
  assert(Mecanum_Feedback_Select(1));
  Mecanum_Feedback_Enable(true);
  before = count;
  advance(15);
  assert(count == before + 1 && frame[before][1] == 0x35);
  uint8_t old_ack[] = {1, 0xf6, 2, 0x6b};
  MotorBus_RxBytes(old_ack, sizeof(old_ack));
  MotorBus_Process(); Mecanum_Process();
  Mecanum_Feedback_t feedback;
  assert(Mecanum_FeedbackGet(0, &feedback) && !feedback.speed_valid);
  uint8_t speed[] = {1, 0x35, 0, 0, 0, 0x6b};
  MotorBus_RxBytes(speed, sizeof(speed));
  MotorBus_Process(); Mecanum_Process();
  assert(Mecanum_FeedbackGet(0, &feedback) && feedback.speed_valid && feedback.speed_rpm == 0);
  Mecanum_StatusGet(&s);
  assert(s.locked && s.error == MECANUM_ERROR_CANCELLED && s.stage == MECANUM_STAGE_STOPPED);
  assert(!Mecanum_Velocity_Request(20, 0, 0, 0));
  Mecanum_Feedback_Enable(false);
  assert(Mecanum_Feedback_Select(0));
  Mecanum_Feedback_Enable(true);
  for (unsigned i = 0; i < 12; i++)
  {
    before = count;
    advance(16);
    assert(count == before + 1);
    uint8_t response[8] = {0};
    response[0] = frame[before][0];
    response[1] = frame[before][1];
    unsigned size = response[1] == 0x35 ? 6 : response[1] == 0x36 ? 8 : 4;
    response[size - 1] = 0x6b;
    MotorBus_RxBytes(response, size);
    MotorBus_Process(); Mecanum_Process();
  }
  for (unsigned i = 0; i < 4; i++)
  {
    assert(Mecanum_FeedbackGet(i, &feedback));
    assert(feedback.speed_valid && feedback.position_valid && feedback.state_valid);
  }
  Mecanum_StatusGet(&s);
  assert(s.locked && s.tx_complete && !Mecanum_Velocity_Request(20, 0, 0, 0));
  MotorBus_ErrorCallback(&uart);
  MotorBus_Process();
  before = count;
  advance(30);
  assert(count == before); /* Real UART faults revoke diagnostic read permission. */
  Mecanum_Feedback_Enable(false);
  puts("mecanum_bus_test: frozen batch, latest target, synchronized TX, stop and fault lock OK");
}
