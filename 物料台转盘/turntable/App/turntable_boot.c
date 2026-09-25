#include "turntable_boot.h"
#include "turntable_config.h"
#include <string.h>
volatile Turntable_BootStatus_t Turntable_BootStatus;
static UART_HandleTypeDef *motor_uart;
static uint32_t phase_tick;
static uint8_t phase;
void Turntable_BootInit(UART_HandleTypeDef *uart)
{
  motor_uart=uart; phase_tick=HAL_GetTick(); phase=0;
  memset((void *)&Turntable_BootStatus,0,sizeof(Turntable_BootStatus));
  Turntable_BootStatus.enable_result=HAL_BUSY;
  Turntable_BootStatus.velocity_result=HAL_BUSY;
}
void Turntable_BootProcess(void)
{
  uint8_t enable[]={TURNTABLE_MOTOR_ID,0xf3,0xab,1,0,0x6b};
  uint8_t velocity[]={TURNTABLE_MOTOR_ID,0xf6,TURNTABLE_DIRECTION,
    (uint8_t)(TURNTABLE_SPEED_RPM>>8),(uint8_t)TURNTABLE_SPEED_RPM,
    TURNTABLE_ACCELERATION,0,0x6b};
  if(phase==0 && (uint32_t)(HAL_GetTick()-phase_tick)>=TURNTABLE_POWERUP_WAIT_MS) {
    Turntable_BootStatus.enable_result=HAL_UART_Transmit(motor_uart,enable,sizeof(enable),20);
    Turntable_BootStatus.enable_sent=1;
    phase_tick=HAL_GetTick(); phase=1;
  } else if(phase==1 && (uint32_t)(HAL_GetTick()-phase_tick)>=100U) {
    /* Diagnostic mode: no RX/ACK prerequisite, even if the preceding TX failed. */
    Turntable_BootStatus.velocity_result=HAL_UART_Transmit(motor_uart,velocity,sizeof(velocity),20);
    Turntable_BootStatus.velocity_sent=1; phase=2;
  }
}
