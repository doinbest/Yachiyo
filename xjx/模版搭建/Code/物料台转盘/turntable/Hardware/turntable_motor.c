/* Independently implemented from ZDT_X42S V1.0.5 Emm protocol chapters.
 * No F407 multi-motor dependencies or persistent driver setting writes. */
#include "turntable_motor.h"
#include "turntable_config.h"
#define RX_SIZE 128U
static UART_HandleTypeDef *port;
static uint8_t rx_byte, rx[RX_SIZE], frame[8], used, expected, function, active;
static volatile uint16_t head, tail;
static volatile uint8_t rx_error;
static uint32_t sent_tick;
static uint8_t Send(uint8_t *data, uint16_t size, uint8_t length)
{
  uint32_t mask=__get_PRIMASK();
  __disable_irq(); tail=head; __set_PRIMASK(mask);
  used=0; function=data[1]; expected=length; active=0;
  if(rx_error) return 0;
  sent_tick=HAL_GetTick();
  if(HAL_UART_Transmit(port,data,size,20)!=HAL_OK) return 0;
  active=1;
  return 1;
}
void Motor_Init(UART_HandleTypeDef *uart)
{
  port=uart; head=tail=0; rx_error=0; used=active=0;
  if(HAL_UART_Receive_IT(port,&rx_byte,1)!=HAL_OK) rx_error=1;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
  uint16_t next;
  if(uart!=port) return;
  next=(uint16_t)((head+1U)%RX_SIZE);
  if(next==tail) rx_error=1;
  else { rx[head]=rx_byte; head=next; }
  if(HAL_UART_Receive_IT(port,&rx_byte,1)!=HAL_OK) rx_error=1;
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  if(uart==port) rx_error=1;
}
uint8_t Motor_Read(uint8_t code)
{
  uint8_t data[3]={TURNTABLE_MOTOR_ID,0,0x6b};
  if(code!=0x35 && code!=0x36 && code!=0x3a) return 0;
  data[1]=code;
  return Send(data,3,code==0x36 ? 8 : (code==0x35 ? 6 : 4));
}
uint8_t Motor_Enable(void)
{
  uint8_t data[6]={TURNTABLE_MOTOR_ID,0xf3,0xab,1,0,0x6b};
  return Send(data,6,4);
}
uint8_t Motor_Move(int32_t pulses)
{
  uint32_t magnitude=(uint32_t)(pulses<0 ? -(int64_t)pulses : pulses);
  uint8_t data[13]={TURNTABLE_MOTOR_ID,0xfd,0,
    (uint8_t)(TURNTABLE_SPEED_RPM>>8),(uint8_t)TURNTABLE_SPEED_RPM,
    TURNTABLE_ACCELERATION,0,0,0,0,1,0,0x6b};
  data[2]=pulses<0;
  data[6]=(uint8_t)(magnitude>>24); data[7]=(uint8_t)(magnitude>>16);
  data[8]=(uint8_t)(magnitude>>8); data[9]=(uint8_t)magnitude;
  return Send(data,13,4);
}
uint8_t Motor_Stop(void)
{
  uint8_t data[5]={TURNTABLE_MOTOR_ID,0xfe,0x98,0,0x6b};
  /* RX error must not prevent a best-effort STOP transmission. */
  if(rx_error) {
    (void)HAL_UART_AbortReceive(port); rx_error=0;
    if(HAL_UART_Receive_IT(port,&rx_byte,1)!=HAL_OK) {
      (void)HAL_UART_Transmit(port,data,5,20); rx_error=1; return 0;
    }
  }
  return Send(data,5,4);
}
Motor_Result_t Motor_Poll(int64_t *value)
{
  uint8_t byte; uint32_t magnitude;
  if(rx_error) { active=0; return MOTOR_COMM_ERROR; }
  if(!active) return MOTOR_WAIT;
  while(tail!=head) {
    byte=rx[tail]; tail=(uint16_t)((tail+1U)%RX_SIZE);
    if(used==0) { if(byte==TURNTABLE_MOTOR_ID) frame[used++]=byte; continue; }
    if(used==1 && byte!=function) { used=byte==TURNTABLE_MOTOR_ID ? 1 : 0; continue; }
    frame[used++]=byte;
    if(used<expected) continue;
    used=0;
    if(frame[expected-1]!=0x6b) continue;
    if(function==0x36 || function==0x35) {
      if(frame[2]>1) continue;
      magnitude=function==0x36 ? ((uint32_t)frame[3]<<24)|((uint32_t)frame[4]<<16)|
        ((uint32_t)frame[5]<<8)|frame[6] : ((uint32_t)frame[3]<<8)|frame[4];
      *value=frame[2] ? -(int64_t)magnitude : (int64_t)magnitude;
    } else {
      *value=frame[2];
      if(function!=0x3a && frame[2]!=0x02) { active=0; return MOTOR_REJECTED; }
    }
    active=0; return MOTOR_OK;
  }
  if((uint32_t)(HAL_GetTick()-sent_tick)>=TURNTABLE_REPLY_TIMEOUT_MS) {
    active=0; return MOTOR_COMM_ERROR;
  }
  return MOTOR_WAIT;
}
