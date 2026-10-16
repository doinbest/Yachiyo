/* Real bus tests can start with drivers already configured in Receive mode.
 * The dedicated motor_arrival_test covers read/preserve/write/verify itself. */
static void fixture_receive_modes(UART_HandleTypeDef *uart,uint32_t *clock)
{
 for(uint8_t id=5;id<=8;id++) {
  int ready=MotorBus_PositionReplyEnsure(id);
  if(ready==1)continue;
  assert(ready==0);*clock+=3;MotorBus_Process();
  uint8_t cfg[33]={0};cfg[0]=id;cfg[1]=0x42;cfg[2]=33;cfg[3]=21;cfg[22]=1;cfg[32]=0x6b;
  MotorBus_TxCpltCallback(uart);MotorBus_RxBytes(cfg,33);MotorBus_Process();
  assert(MotorBus_PositionReplyEnsure(id)==1);*clock+=3;
 }
}
