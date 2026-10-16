#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main circular_fixture_main
#include "motor_bus_circular_rx_test.c"
#undef main
#pragma GCC diagnostic pop
int main(void)
{
 MotorBus_RxErrorSnapshot_t s,first;
 const uint8_t query[]={7,0x35,0x6b},answer[]={7,0x35,0,0,0,0x6b};
 MotorBus_Event_t e;
 assert(MotorBus_Init(&uart)==HAL_OK && !MotorBus_RxErrorGet(&s,false));
 tick=1000;assert(MotorBus_Submit(MOTOR_BUS_ARM,query,3,6,123,false));
 MotorBus_Process();MotorBus_TxCpltCallback(&uart);MotorBus_Process();
 /* DMA contains the full frame, but no HT/IDLE callback delivered it yet. */
 receive_bytes(answer,6,false);assert(rx_position==6);
 MotorBus_UartErrorIRQ(&uart,4);
 /* DMA may advance before deferred HAL ErrorCallback; keep the IRQ image. */
 rx_buffer[0]=0xfa;rx_dma.counter=56;rx_active=false;
 uart.ErrorCode=HAL_UART_ERROR_NE;MotorBus_ErrorCallback(&uart);
 assert(MotorBus_RxErrorGet(&s,false) && s.sequence==1 && s.phase==1);
 assert(s.dma_remaining==58 && s.dma_remaining_end==58 && s.dma_cursor==64);
 assert(!s.queued && !s.partial && s.uart_sr==4 && s.uart_error==HAL_UART_ERROR_NE);
 assert(s.active && s.waiting && s.tx_complete && s.tx_ms==1000 && s.token==123);
 assert(s.tx_length==3 && !memcmp(s.tx,query,3) && !memcmp(s.dma,answer,6));
 MotorBus_Process();assert(!MotorBus_EventGet(MOTOR_BUS_ARM,&e));
 tick+=100;MotorBus_Process();assert(MotorBus_EventGet(MOTOR_BUS_ARM,&e) && e.result==MOTOR_BUS_TIMEOUT);
 assert(MotorBus_RxErrorGet(&first,true) && first.sequence==s.sequence);
 /* A later error updates latest, not the original failure snapshot. */
 uart.ErrorCode=HAL_UART_ERROR_DMA;MotorBus_ErrorCallback(&uart);MotorBus_Process();
 assert(MotorBus_RxErrorGet(&s,false) && s.sequence==2 && s.phase==0 && !s.active && !s.tx_length);
 assert(MotorBus_RxErrorGet(&first,true) && first.sequence==1 && first.dma[0]==7);
 assert(!MotorBus_RxErrorGet(NULL,false));
 puts("RX error snapshot: IRQ DMA image, exact TX, no data promotion, frozen first and callback fallback PASS");return 0;
}
