/* Real application/driver run against a simulated UART and motor, never real hardware. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "turntable.h"
#include "turntable_motor.h"
static UART_HandleTypeDef uart;
static uint32_t tick, moves, stops;
static uint8_t *rx_byte;
static uint8_t reply[16], reply_size;
static uint8_t pa, pb, settle, silent, stale, bad_ack, disabled, running;
static int64_t position;
static int32_t target, targets[1024];
static uint8_t last_move[13];
uint32_t HAL_GetTick(void) { return tick; }
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin)
{ (void)pin; return (port == GPIOA ? pa : pb) ? GPIO_PIN_RESET : GPIO_PIN_SET; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *p, uint8_t *b, uint16_t n)
{ (void)p; assert(n == 1); rx_byte=b; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *p)
{ (void)p; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *p,uint8_t *b,uint16_t n,uint32_t t)
{
  uint64_t mag; unsigned i; (void)p; (void)t;
  assert(b[0]==1 && b[n-1]==0x6b);
  reply[0]=1; reply[1]=b[1]; reply[2]=2; reply_size=4;
  if(b[1]==0xfd) {
    assert(n==13 && b[10]==1 && b[11]==0);
    assert(b[3]==0 && b[4]==8); /* 8 RPM, not X firmware 0.1 RPM */
    memcpy(last_move,b,13);
    mag=((uint32_t)b[6]<<24)|((uint32_t)b[7]<<16)|((uint32_t)b[8]<<8)|b[9];
    target=b[2] ? -(int32_t)mag : (int32_t)mag;
    assert(moves<1024); targets[moves++]=target; running=1;
    if(bad_ack) reply[2]=0xe2;
  } else if(b[1]==0xfe) { stops++; running=0; }
  else if(b[1]==0x36) {
    if(settle && running && !stale) position=(int64_t)target*65536/3200;
    reply_size=8; reply[2]=position<0;
    mag=(uint64_t)(position<0 ? -position : position);
    for(i=0;i<4;i++) reply[3+i]=(uint8_t)(mag>>(24-8*i));
  } else if(b[1]==0x3a) {
    reply[2]=disabled ? 0 : ((settle || !running) ? 3 : 1);
  } else if(b[1]==0x35) {
    reply_size=6; reply[2]=0; reply[3]=0;
    reply[4]=(!settle && running) ? 8 : 0;
  } else assert(b[1]==0xf3);
  reply[reply_size-1]=0x6b;
  if(silent) reply_size=0;
  return HAL_OK;
}
static void advance(unsigned ms)
{
  unsigned i; while(ms--) {
    tick++;
    if(reply_size) {
      uint8_t data[16], len=reply_size; memcpy(data,reply,len); reply_size=0;
      for(i=0;i<len;i++) { assert(rx_byte); *rx_byte=data[i]; HAL_UART_RxCpltCallback(&uart); }
    }
    Turntable_Process();
  }
}
static void reset(void)
{
  tick=moves=stops=0; rx_byte=0; reply_size=0; position=0; target=0;
  pa=pb=settle=silent=stale=bad_ack=disabled=running=0;
  Turntable_Init(&uart); advance(1200);
}
static void press_a(void) { pa=1; advance(30); pa=0; advance(30); }
static void press_b(void) { pb=1; advance(30); pb=0; advance(30); }
static void wait_state(Turntable_State_t state)
{
  unsigned n=2000;
  while(Turntable_Status.state!=state && n--) advance(1);
  assert(Turntable_Status.state==state);
}
int main(void)
{
  unsigned i; int64_t value; uint8_t packet[8];
  reset(); assert(moves==0); press_a(); advance(200);
  assert(moves==1 && target==533); /* nearest pulse to 60 degrees */
  advance(500); assert(Turntable_Status.state==TURNTABLE_MOVING);
  settle=1; wait_state(TURNTABLE_DWELL);
  advance(1999); assert(moves==1); advance(20); assert(moves==2 && target==1600);
  wait_state(TURNTABLE_DWELL); advance(2020); assert(target==2667);
  wait_state(TURNTABLE_DWELL); press_a(); advance(3000); assert(moves==3);

  reset(); press_b(); advance(200); assert(moves==1 && !Turntable_Status.automatic);
  pb=1; advance(2500); pb=0; advance(30); assert(moves==1); /* no hold repeat/homing */
  settle=1; wait_state(TURNTABLE_IDLE); advance(3000); assert(moves==1);
  press_b(); advance(200); assert(moves==2 && target==1600);

  reset(); press_a(); advance(200); position=4000;
  press_a(); wait_state(TURNTABLE_IDLE);
  assert(stops==1 && Turntable_Status.stop_confirmed);
  press_a(); advance(200); assert(moves==2 && targets[0]==targets[1]);

  reset(); stale=settle=1; press_a(); advance(500);
  assert(Turntable_Status.state==TURNTABLE_MOVING); /* old reached bit is insufficient */
  silent=1; advance(500); assert(Turntable_Status.state==TURNTABLE_FAULT && stops==1);
  assert(!Turntable_Status.stop_confirmed); press_a(); assert(moves==1);

  reset(); bad_ack=1; press_a(); advance(300);
  assert(Turntable_Status.state==TURNTABLE_FAULT && stops==1);
  reset(); press_a(); advance(200); disabled=1; advance(300);
  assert(Turntable_Status.state==TURNTABLE_FAULT);

  reset(); pa=1; advance(5); pa=0; advance(30); assert(moves==0);
  position=-65536; press_b(); advance(200); assert(target==-2667);
  settle=1; wait_state(TURNTABLE_IDLE);
  for(i=0;i<9;i++) { press_b(); wait_state(TURNTABLE_IDLE); }
  assert(target==6933); /* accumulated rounding, 3 turns beyond first target */

  reset(); press_a(); advance(200); silent=1; press_a(); advance(400);
  assert(Turntable_Status.state==TURNTABLE_FAULT && !Turntable_Status.stop_confirmed);
  reset(); press_a(); advance(11000);
  assert(Turntable_Status.state==TURNTABLE_FAULT && Turntable_Status.error==TURNTABLE_ERROR_MOTION_TIMEOUT);
  reset(); pa=pb=1; advance(30); pa=pb=0; advance(150);
  assert(moves==1 && Turntable_Status.automatic); /* stop/start key priority */
  reset(); pa=1; Turntable_Init(&uart); advance(1500); assert(moves==0);
  pa=0; advance(30); press_a(); advance(200); assert(moves==1);

  reset(); tick=0xfffffff0U; Turntable_Init(&uart); advance(1200);
  press_b(); advance(200); assert(moves==1); /* tick rollover */
  reset(); press_a(); advance(200); HAL_UART_ErrorCallback(&uart); advance(200);
  assert(Turntable_Status.state==TURNTABLE_FAULT && stops==1);

  /* Feed a split response containing 0x6b in its payload: trailer is not a delimiter. */
  reset(); position=0x6b00; assert(Motor_Read(0x36));
  memcpy(packet,reply,8); reply_size=0;
  for(i=0;i<4;i++) { *rx_byte=packet[i]; HAL_UART_RxCpltCallback(&uart); }
  assert(Motor_Poll(&value)==MOTOR_WAIT);
  for(i=4;i<8;i++) { *rx_byte=packet[i]; HAL_UART_RxCpltCallback(&uart); }
  assert(Motor_Poll(&value)==MOTOR_OK && value==0x6b00);
  assert(Motor_Read(0x36)); reply_size=0;
  packet[7]=0; /* reject invalid trailer */
  for(i=0;i<8;i++) { *rx_byte=packet[i]; HAL_UART_RxCpltCallback(&uart); }
  assert(Motor_Poll(&value)==MOTOR_WAIT);
  tick+=251; assert(Motor_Poll(&value)==MOTOR_COMM_ERROR);
  puts("turntable behavior: PASS"); return 0;
}
