#include "turntable_direct.h"
#include "turntable_config.h"
#include "turntable_key.h"
#include <string.h>
volatile Turntable_DirectStatus_t Turntable_DirectStatus;
static UART_HandleTypeDef *motor_uart;
static uint32_t boot_tick, phase_tick, step_wait_ms;
static uint8_t first_step, remainder;

static uint8_t Send(uint8_t *data,uint16_t size)
{
  Turntable_DirectStatus.tx_attempts++;
  Turntable_DirectStatus.tx_result=HAL_UART_Transmit(motor_uart,data,size,20);
  return Turntable_DirectStatus.tx_result==HAL_OK;
}
static void Stop(void)
{
  uint8_t data[]={TURNTABLE_MOTOR_ID,0xfe,0x98,0,0x6b};
  Turntable_DirectStatus.automatic=0;
  /* Cancel pending motion even if TX fails; another PA15 press retries STOP. */
  Turntable_DirectStatus.state=DIRECT_STOP_PENDING;
  if(Send(data,sizeof(data))) Turntable_DirectStatus.state=DIRECT_IDLE;
}
static void Enable(uint8_t automatic)
{
  uint8_t data[]={TURNTABLE_MOTOR_ID,0xf3,0xab,1,0,0x6b};
  if(Send(data,sizeof(data))) {
    Turntable_DirectStatus.automatic=automatic;
    Turntable_DirectStatus.state=DIRECT_ENABLE_STEP;
    phase_tick=HAL_GetTick();
  }
}
static void Step(void)
{
  uint32_t numerator=(first_step ? TURNTABLE_PULSES_PER_REV : 2U*TURNTABLE_PULSES_PER_REV)+remainder;
  uint32_t pulses=numerator/6U;
  uint8_t data[]={TURNTABLE_MOTOR_ID,0xfd,TURNTABLE_DIRECTION,
    (uint8_t)(TURNTABLE_SPEED_RPM>>8),(uint8_t)TURNTABLE_SPEED_RPM,
    TURNTABLE_ACCELERATION,0,0,0,0,0,0,0x6b};
  /* Mode 0: increment the preceding target. Do not use mode 2 here:
     the user's drive behaved as absolute positioning with that value. */
  data[6]=(uint8_t)(pulses>>24); data[7]=(uint8_t)(pulses>>16);
  data[8]=(uint8_t)(pulses>>8); data[9]=(uint8_t)pulses;
  if(Send(data,sizeof(data))) {
    remainder=(uint8_t)(numerator%6U); first_step=0;
    /* Local busy estimate only; no physical arrival/stop confirmation. */
    step_wait_ms=(pulses*60000U+TURNTABLE_PULSES_PER_REV*TURNTABLE_SPEED_RPM-1U)/
                 (TURNTABLE_PULSES_PER_REV*TURNTABLE_SPEED_RPM)+500U;
    phase_tick=HAL_GetTick(); Turntable_DirectStatus.state=DIRECT_STEP;
  } else {
    Turntable_DirectStatus.state=DIRECT_IDLE;
    Turntable_DirectStatus.automatic=0;
  }
}
void Turntable_DirectInit(UART_HandleTypeDef *uart)
{
  motor_uart=uart; boot_tick=HAL_GetTick(); first_step=1; remainder=3;
  memset((void *)&Turntable_DirectStatus,0,sizeof(Turntable_DirectStatus));
  Turntable_DirectStatus.tx_result=HAL_BUSY;
  Key_Init();
}
void Turntable_DirectProcess(void)
{
  uint8_t keys=Key_Scan(); uint32_t now=HAL_GetTick();
  if(keys&KEY_START) Turntable_DirectStatus.pa15_events++;
  if(keys&KEY_STEP) Turntable_DirectStatus.pb3_events++;
  if((uint32_t)(now-boot_tick)<TURNTABLE_POWERUP_WAIT_MS) return;
  if(keys&KEY_START) {
    if(Turntable_DirectStatus.state==DIRECT_IDLE) Enable(1);
    else Stop();
    return;
  }
  if((keys&KEY_STEP) && Turntable_DirectStatus.state==DIRECT_IDLE) { Enable(0); return; }
  if(Turntable_DirectStatus.state==DIRECT_ENABLE_STEP && (uint32_t)(now-phase_tick)>=100U) {
    Step();
  } else if(Turntable_DirectStatus.state==DIRECT_STEP && (uint32_t)(now-phase_tick)>=step_wait_ms) {
    Turntable_DirectStatus.state=Turntable_DirectStatus.automatic ? DIRECT_DWELL : DIRECT_IDLE;
    phase_tick=now;
  } else if(Turntable_DirectStatus.state==DIRECT_DWELL && (uint32_t)(now-phase_tick)>=TURNTABLE_DWELL_MS) {
    Step();
  }
}
