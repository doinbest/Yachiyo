#include "turntable.h"
#include "turntable_config.h"
#include "turntable_key.h"
#include "turntable_motor.h"
#include <string.h>

typedef enum { PHASE_NONE, PHASE_ORIGIN, PHASE_ENABLE, PHASE_MOVE_ACK,
  PHASE_POLL_DELAY, PHASE_FLAGS, PHASE_POSITION, PHASE_SPEED,
  PHASE_STOP_ACK, PHASE_STOP_DELAY, PHASE_STOP_SPEED } Phase_t;
volatile Turntable_Status_t Turntable_Status;
static Phase_t phase;
static uint32_t boot_tick, action_tick, poll_tick, dwell_tick, sixths;
static int64_t origin_pulses;
static uint8_t origin_valid, pending_target, flags, position_ok, stable_count;

static int64_t Abs64(int64_t value) { return value<0 ? -value : value; }
static void Fault(Turntable_Error_t error)
{
  (void)Motor_Stop();
  Turntable_Status.error=error;
  Turntable_Status.state=TURNTABLE_FAULT;
  Turntable_Status.automatic=0; Turntable_Status.stop_confirmed=0;
  phase=PHASE_NONE;
}
static uint8_t CheckSend(uint8_t sent)
{
  if(!sent) Fault(TURNTABLE_ERROR_COMM);
  return sent;
}
static void Move(void)
{
  int64_t target;
  if(!pending_target) {
    if(sixths>1000000U) { Fault(TURNTABLE_ERROR_RANGE); return; }
    sixths=sixths==0 ? 1U : sixths+2U;
    target=((int64_t)sixths*TURNTABLE_PULSES_PER_REV+3)/6;
    if(TURNTABLE_DIRECTION) target=-target;
    target+=origin_pulses;
    /* Keep target inside the protocol's 32-bit magnitude feedback range. */
    if(Abs64(target)*TURNTABLE_POSITION_PER_REV/TURNTABLE_PULSES_PER_REV>0xffffffffLL) {
      Fault(TURNTABLE_ERROR_RANGE); return;
    }
    Turntable_Status.target_pulses=(int32_t)target;
    pending_target=1;
  }
  if(CheckSend(Motor_Move(Turntable_Status.target_pulses))) {
    Turntable_Status.state=TURNTABLE_MOVING; phase=PHASE_MOVE_ACK;
    Turntable_Status.stop_confirmed=0;
    action_tick=HAL_GetTick(); stable_count=0;
  }
}
static void Start(uint8_t automatic)
{
  Turntable_Status.automatic=automatic;
  Turntable_Status.stop_confirmed=0;
  Turntable_Status.state=TURNTABLE_STARTING;
  if(!origin_valid) {
    if(CheckSend(Motor_Read(0x36))) phase=PHASE_ORIGIN;
  } else if(CheckSend(Motor_Enable())) phase=PHASE_ENABLE;
}
static void Stop(void)
{
  Turntable_Status.automatic=0;
  Turntable_Status.stop_confirmed=0;
  if(CheckSend(Motor_Stop())) {
    Turntable_Status.state=TURNTABLE_STOPPING; phase=PHASE_STOP_ACK;
    action_tick=HAL_GetTick(); stable_count=0;
  }
}
void Turntable_Init(UART_HandleTypeDef *uart)
{
  memset((void *)&Turntable_Status,0,sizeof(Turntable_Status));
  phase=PHASE_NONE; origin_valid=pending_target=stable_count=0;
  sixths=0; origin_pulses=0; boot_tick=HAL_GetTick();
  Motor_Init(uart); Key_Init();
}
void Turntable_Process(void)
{
  uint32_t now=HAL_GetTick(); uint8_t keys=Key_Scan();
  int64_t value=0, target_units; Motor_Result_t result;
  if(Turntable_Status.state==TURNTABLE_FAULT) return;
  if((uint32_t)(now-boot_tick)<TURNTABLE_POWERUP_WAIT_MS) return;
  /* Start/stop wins if both keys become active together. */
  if(keys&KEY_START) {
    if(Turntable_Status.state==TURNTABLE_IDLE) Start(1);
    else if(Turntable_Status.state!=TURNTABLE_STOPPING) Stop();
    return;
  }
  if((keys&KEY_STEP) && Turntable_Status.state==TURNTABLE_IDLE) { Start(0); return; }
  if(Turntable_Status.state==TURNTABLE_DWELL) {
    if((uint32_t)(now-dwell_tick)>=TURNTABLE_DWELL_MS) Move();
    return;
  }
  if(Turntable_Status.state==TURNTABLE_MOVING &&
     (uint32_t)(now-action_tick)>=TURNTABLE_MOVE_TIMEOUT_MS) { Fault(TURNTABLE_ERROR_MOTION_TIMEOUT); return; }
  if(Turntable_Status.state==TURNTABLE_STOPPING &&
     (uint32_t)(now-action_tick)>=TURNTABLE_STOP_TIMEOUT_MS) { Fault(TURNTABLE_ERROR_MOTION_TIMEOUT); return; }
  if(phase==PHASE_NONE) return;
  if(phase==PHASE_POLL_DELAY || phase==PHASE_STOP_DELAY) {
    if((uint32_t)(now-poll_tick)<TURNTABLE_POLL_MS) return;
    if(phase==PHASE_STOP_DELAY) { if(CheckSend(Motor_Read(0x35))) phase=PHASE_STOP_SPEED; }
    else if(CheckSend(Motor_Read(0x3a))) phase=PHASE_FLAGS;
    return;
  }
  result=Motor_Poll(&value);
  if(result==MOTOR_WAIT) return;
  if(result!=MOTOR_OK) { Fault(result==MOTOR_REJECTED ? TURNTABLE_ERROR_DRIVER : TURNTABLE_ERROR_COMM); return; }
  switch(phase) {
    case PHASE_ORIGIN:
      Turntable_Status.position_units=value;
      origin_pulses=(Abs64(value)*TURNTABLE_PULSES_PER_REV+TURNTABLE_POSITION_PER_REV/2)/TURNTABLE_POSITION_PER_REV;
      if(value<0) origin_pulses=-origin_pulses;
      origin_valid=1;
      if(CheckSend(Motor_Enable())) phase=PHASE_ENABLE;
      break;
    case PHASE_ENABLE: Move(); break;
    case PHASE_MOVE_ACK: poll_tick=now; phase=PHASE_POLL_DELAY; break;
    case PHASE_FLAGS:
      flags=(uint8_t)value;
      if(!(flags&1U) || (flags&0x0cU)) { Fault(TURNTABLE_ERROR_DRIVER); break; }
      if(CheckSend(Motor_Read(0x36))) phase=PHASE_POSITION;
      break;
    case PHASE_POSITION:
      Turntable_Status.position_units=value;
      target_units=(int64_t)Turntable_Status.target_pulses*TURNTABLE_POSITION_PER_REV/TURNTABLE_PULSES_PER_REV;
      position_ok=Abs64(value-target_units)<=TURNTABLE_POSITION_WINDOW;
      if(CheckSend(Motor_Read(0x35))) phase=PHASE_SPEED;
      break;
    case PHASE_SPEED:
      if((flags&2U) && position_ok && value==0) stable_count++;
      else stable_count=0;
      if(stable_count>=2) {
        pending_target=0; Turntable_Status.completed_moves++;
        Turntable_Status.stop_confirmed=1; phase=PHASE_NONE;
        Turntable_Status.state=Turntable_Status.automatic ? TURNTABLE_DWELL : TURNTABLE_IDLE;
        dwell_tick=now;
      } else { phase=PHASE_POLL_DELAY; poll_tick=now; }
      break;
    case PHASE_STOP_ACK: phase=PHASE_STOP_DELAY; poll_tick=now; break;
    case PHASE_STOP_SPEED:
      if(value==0) stable_count++; else stable_count=0;
      if(stable_count>=2) {
        Turntable_Status.state=TURNTABLE_IDLE; Turntable_Status.stop_confirmed=1; phase=PHASE_NONE;
      } else { phase=PHASE_STOP_DELAY; poll_tick=now; }
      break;
    default: break;
  }
}
