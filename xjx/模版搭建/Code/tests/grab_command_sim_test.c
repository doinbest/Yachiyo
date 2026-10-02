/* Production grab command/state machine + production paced console TX.
 * Reuse mechanical/clock endpoints; the plant below is synthetic, not a calibration. */
#define GRAB_COMMAND_SIM
#define main grab_state_regression_main
#include "grab_task_test.c"
#undef main

static UART_HandleTypeDef sim_uart;
static char wire[65536];
static size_t wire_size;
static bool tx_blocked;
static unsigned speed_commands;

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart,uint8_t *data,uint16_t size)
{
  assert(uart==&sim_uart && size<=CONSOLE_TX_CHUNK_BYTES);
  if(tx_blocked) return HAL_BUSY;
  assert(wire_size+size<sizeof wire);
  memcpy(wire+wire_size,data,size); wire_size+=size; wire[wire_size]=0;
  ConsoleTx_TxCpltCallback(uart);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart)
{ assert(uart==&sim_uart); return HAL_OK; }

static void sim_reset(void)
{
  reset(); ConsoleTx_Init(&sim_uart); configure(true);
  wire[0]=0; wire_size=0; tx_blocked=false; speed_commands=0;
}
static void sim_command(const char *text)
{
  char line[64], *tokens[6], *p; unsigned count=0;
  assert(strlen(text)<sizeof line); strcpy(line,text);
  for(p=strtok(line," ");p && count<6;p=strtok(NULL," ")) tokens[count++]=p;
  assert(GrabTask_Command(count,tokens));
}
static void sim_step(int dx,int dy)
{
  step(true,1,dx,dy); ConsoleTx_Process(); speed_commands=velocity_requests;
}
static void drain(void)
{
  unsigned i;
  for(i=0;i<300;i++) { clock_ms+=50; ConsoleTx_Process(); }
  ConsoleTx_Stats_t stats; ConsoleTx_GetStats(&stats);
  if(stats.reply_pending || stats.reply_dropped || stats.urgent_dropped) fprintf(stderr,"pending=%u dropped=%lu urgent=%lu\n",stats.reply_pending,(unsigned long)stats.reply_dropped,(unsigned long)stats.urgent_dropped);
  assert(!stats.reply_pending && !stats.reply_dropped && !stats.urgent_dropped);
}
static void wait_align(void)
{
  unsigned i;
  for(i=0;i<180 && status().state!=GRAB_ALIGN;i++) sim_step(40,30);
  assert(status().state==GRAB_ALIGN && requested_color==CAMERA_COLOR_BLUE);
}
static void finish_stop(bool repeat)
{
  unsigned i, moves=speed_commands;
  sim_command("grab stop");
  for(i=0;i<100 && status().state!=GRAB_IDLE;i++) {
    if(repeat && i%10==0) sim_command("grab stop");
    sim_step(30,20);
  }
  assert(status().state==GRAB_IDLE && status().stop_confirmed);
  sim_command("grab status");
  assert(speed_commands==moves && !closures && !base_moves);
}
static void synthetic_convergence(void)
{
  float dx=40,dy=30,last_x=0; unsigned i; GrabTask_State_t previous;
  sim_reset(); sim_command("grab start align"); wait_align(); previous=status().state;
  printf("SIM start: B2 target=%u dx=40 dy=30; synthetic J=[1,0,0;0,1,1]\n",requested_color);
  for(i=0;i<1600 && status().state!=GRAB_HOLD;i++) {
    sim_step((int)lroundf(dx),(int)lroundf(dy));
    /* Pixels respond to body travel and actual simulated X displacement. */
    dx+=status().forward_mm_s*.05f;
    dy+=status().left_mm_s*.05f+physical[0]-last_x; last_x=physical[0];
    if(status().state!=previous) {
      printf("SIM state=%s elapsed=%lu dx=%.2f dy=%.2f\n",status().state_name,
        (unsigned long)status().elapsed_ms,dx,dy); previous=status().state;
    }
    assert(status().state!=GRAB_ERROR && status().state!=GRAB_STOPPING);
  }
  assert(status().state==GRAB_HOLD && status().stop_confirmed && !closures && !base_moves);
  assert(fabsf(dx)<=2.6f && fabsf(dy)<=2.6f && speed_commands>0);
  sim_command("grab status"); finish_stop(false); drain();
  assert(strstr(wire,"OK grab start align accepted color=3 protocol=B2\r\n"));
  assert(strstr(wire,"VISION RX fn=B2 color=3"));
  assert(strstr(wire,"state=hold mode=align") && strstr(wire,"state=idle mode=align"));
  puts("SIM nominal: converged, no grasp, confirmed stop, complete wire replies PASS");
}
static void blocked_transmit_then_repeated_stop(void)
{
  unsigned i;
  sim_reset(); sim_command("grab start align"); wait_align();
  tx_blocked=true;
  for(i=0;i<80;i++) sim_step(40,30);
  assert(speed_commands && status().state==GRAB_ALIGN);
  finish_stop(true); /* Physical confirmation must not depend on console delivery. */
  assert(status().stop_confirmed);
  tx_blocked=false; drain();
  assert(strstr(wire,"state=idle mode=align reason=cancelled") && strstr(wire,"stop_confirmed=1"));
  puts("SIM TX unavailable for 4s + repeated stops: stopped before link returns; reply drained PASS");
}
static void loss_and_recovery(void)
{
  unsigned i;
  sim_reset(); sim_command("grab start align"); wait_align();
  camera_valid=false; sim_step(40,30);
  assert(status().state==GRAB_VISION_PAUSE);
  for(i=0;i<100 && !status().stop_confirmed;i++) sim_step(40,30);
  assert(status().state==GRAB_VISION_PAUSE && status().stop_confirmed);
  /* Model the existing Pi recovering only after another ordinary B2 color request. */
  for(i=0;i<10 && status().state!=GRAB_ALIGN;i++) {
    camera_valid=camera_requests>=2;
    sim_step(30,20);
  }
  assert(status().state==GRAB_ALIGN && status().recovery_used==1 && camera_requests==2);
  finish_stop(false); drain();
  puts("SIM blue dropout until B2 restart: confirmed stop, one re-request, 3 new samples, resume PASS");
}
static void delayed_start_followed_by_stop(void)
{
  sim_reset(); clock_ms+=48000;
  /* Reproduce a delayed batch without running motion between received commands. */
  sim_command("grab start align"); sim_command("grab stop");
  for(unsigned i=0;i<100 && status().state!=GRAB_IDLE;i++) sim_step(40,30);
  assert(status().state==GRAB_IDLE && status().stop_confirmed);
  assert(!positions && !velocity_requests && !camera_requests && !closures);
  drain();
  puts("SIM delayed start/stop batch: cancelled before any target movement PASS");
}
static void blue_grace_command_and_logging(void)
{
  unsigned i;
  sim_reset(); sim_command("grab set loss_grace_ms 250");
  sim_command("grab start align"); wait_align();
  for(i=0;i<30;i++) sim_step(40,30);
  camera_valid=false; sim_step(40,30);
  assert(status().state==GRAB_VISION_GRACE && !status().stop_requested);
  sim_command("grab status");
  camera_valid=true; sim_step(39,29); sim_step(38,28); sim_step(37,27);
  assert(status().state==GRAB_ALIGN && !RecoveryUsed && status().loss_ms==200);
  sim_command("grab status"); finish_stop(false); drain();
  assert(strstr(wire,"state=vision_grace mode=align reason=vision_gap_decelerating"));
  assert(strstr(wire,"reason=vision_gap_recovered") && strstr(wire,"loss_ms=200 loss_max_ms=200 grace_count=0\r\n"));
  puts("SIM grab start align + short dropout: reduced motion, 3 observations, no quota used, complete logs PASS");
}
int main(void)
{
  setbuf(stdout,NULL);
  synthetic_convergence(); blocked_transmit_then_repeated_stop();
  loss_and_recovery(); delayed_start_followed_by_stop();
  blue_grace_command_and_logging();
  puts("grab_command_sim_test: PASS");
  return 0;
}
