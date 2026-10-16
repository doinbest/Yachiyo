#include "mecanum_chassis.h"
#include "motor_bus.h"
#include "hwt101_calibration.h"
#include <assert.h>
#include <stdio.h>
static uint32_t tick=10;
static UART_HandleTypeDef uart;
uint32_t HAL_GetTick(void){return tick;}
bool ChassisRoute_IsBusy(void){return false;}
bool ChassisMotion_IsBusy(void){return false;}
bool HWT101_Cal_IsBusy(void){return false;}
bool HWT101_Cal_ControlAngleGet(HWT101_Angle_t *a){(void)a;return false;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{(void)u;(void)p;(void)n;return HAL_OK;}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{(void)u;(void)p;(void)n;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u){(void)u;return HAL_OK;}
static uint32_t read_start(uint8_t function,uint8_t length,uint32_t token)
{
  tick+=3;const uint8_t q[]={2,function,0x6b};
  assert(MotorBus_Submit(MOTOR_BUS_FEEDBACK,q,3,length,token,false));MotorBus_Process();
  tick++;MotorBus_TxCpltCallback(&uart);MotorBus_Process();return tick;
}
static void reply(const uint8_t *p,unsigned n)
{tick++;MotorBus_RxBytes(p,(uint16_t)n);MotorBus_Process();Mecanum_Process();}
int main(void)
{
  Mecanum_Feedback_t f;Mecanum_Status_t s;
  const uint8_t speed[]={2,0x35,0,0,0,0x6b},position[]={2,0x36,0,0,0,0,0,0x6b},state[]={2,0x3a,3,0x6b};
  assert(MotorBus_Init(&uart)==HAL_OK);
  read_start(0x35,6,3);reply(speed,6);uint32_t speed_rx=tick;
  read_start(0x36,8,4);reply(position,8);uint32_t position_rx=tick;
  uint32_t first=read_start(0x3a,4,5);tick+=100;MotorBus_Process();
  Mecanum_Process();Mecanum_StatusGet(&s);assert(!s.locked);
  tick++;MotorBus_TxCpltCallback(&uart);MotorBus_Process();reply(state,4);
  assert(Mecanum_FeedbackGet(1,&f));
  assert(f.state_valid && f.state_flags==3 && f.state_ms==first && f.state_ms<tick);
  assert(f.speed_valid && f.speed_ms==speed_rx && f.position_valid && f.position_ms==position_rx);
  /* A later ordinary query retains actual RX freshness with no retry penalty. */
  read_start(0x3a,4,5);reply(state,4);
  assert(Mecanum_FeedbackGet(1,&f) && f.state_ms==tick && !MotorBus_IsQuarantined());
  puts("mecanum status retry timestamp PASS");return 0;
}
