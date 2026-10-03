/* Real UART scheduler and DMA receive path; no serial hardware is opened. */
#include "radar_uart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t tick;
static DMA_HandleTypeDef dma;
static UART_HandleTypeDef uart={USART2,&dma,0};
static uint8_t *rx;
static uint16_t rx_at;
static HAL_UART_RxEventTypeTypeDef rx_type;
static bool busy,force_busy,complete=true;
static HAL_StatusTypeDef tx_result=HAL_OK;
static char sent[7];
static unsigned sends;
uint32_t HAL_GetTick(void){return tick;}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{assert(u==&uart&&n==4096);rx=p;rx_at=0;dma.counter=n;return HAL_OK;}
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *u){(void)u;return rx_type;}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u){(void)u;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u){(void)u;busy=false;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{assert(u==&uart&&n==6);if(force_busy||busy)return HAL_BUSY;if(tx_result!=HAL_OK)return tx_result;
 memcpy(sent,p,6);sent[6]=0;busy=true;sends++;return HAL_OK;}
static void process(void)
{LdsEvent_t event;RadarUart_Process();while(RadarUart_ReadEvent(&event)){}if(busy&&complete){busy=false;RadarUart_TxCpltCallback(&uart);}}
static void input(const void *data,unsigned n)
{const uint8_t *p=data;for(unsigned i=0;i<n;i++){
 rx[rx_at++]=p[i];if(rx_at==4096){rx_at=0;dma.counter=4096;rx_type=HAL_UART_RXEVENT_TC;RadarUart_RxEventCallback(&uart,4096);}
 else {dma.counter=4096-rx_at;if(rx_at==2048){rx_type=HAL_UART_RXEVENT_HT;RadarUart_RxEventCallback(&uart,2048);}}
 }rx_type=HAL_UART_RXEVENT_IDLE;RadarUart_RxEventCallback(&uart,rx_at);for(unsigned i=0;i<32;i++)process();}
static void init(void)
{busy=force_busy=false;complete=true;tx_result=HAL_OK;sends=0;assert(RadarUart_Init(&uart)==HAL_OK);}
static void start(void)
{assert(RadarUart_ProbeStart());process();process();assert(sends==1&&!strcmp(sent,"LUUIDH"));}
static RadarUart_Probe_t status(void){RadarUart_Probe_t s;RadarUart_ProbeGet(&s);return s;}
int main(void)
{
 RadarUart_Probe_t s;RadarUart_Status_t d;
 const uint8_t alarm[]={0xce,0xce,0xce,0xce,0,1};
 init();assert(sends==0&&status().state==RADAR_PROBE_IDLE);start();
 assert(status().tx_complete&&status().state==RADAR_PROBE_WAITING);
 assert(!RadarUart_ProbeStart()&&!RadarUart_StartScan());
 /* Alarm flood cannot hide a fragmented identity reply or impersonate it. */
 for(unsigned i=0;i<1200;i++)input(alarm,sizeof alarm);
 input("PRODU",5);input("CT SN: ",7);input("ABC123456",9);input("\r\n",2);
 s=status();assert(s.state==RADAR_PROBE_CONFIRMED&&s.tx_complete&&!strcmp(s.sn,"ABC123456"));
 assert(s.alarms==1200&&s.last_alarm==0x0100&&s.rx_bytes==7223);
 assert(s.raw_count==24&&!memcmp(s.raw,alarm,6));
 tick+=100;process();assert(status().rx_bytes==s.rx_bytes); /* Completed evidence is frozen. */
 init();start();input(alarm,6);tick+=2001;process();s=status();
 assert(s.state==RADAR_PROBE_UNCONFIRMED&&!strcmp(s.reason,"alarm_only")&&s.tx_complete);
 init();start();input("LUUIDH",6);tick+=2001;process();
 assert(status().state==RADAR_PROBE_UNCONFIRMED&&!strcmp(status().reason,"reply_unknown"));
 init();start();
 {const uint8_t packet[]={0xce,0xfa,1,0,0,0,17,100,0,118,0};input(packet,sizeof packet);}
 tick+=2001;process();RadarUart_StatusGet(&d);
 assert(d.packets==1&&status().state==RADAR_PROBE_UNCONFIRMED&&!strcmp(status().reason,"reply_unknown"));
 init();start();tick+=2001;process();assert(!strcmp(status().reason,"no_reply"));
 init();memcpy(rx,alarm,6);rx_at=6;dma.counter=4090;start();
 assert(status().rx_bytes==0&&status().alarms==0); /* A buffered alarm predates this query. */
 input(alarm,6);assert(status().alarms==1);tick+=2001;process();assert(!strcmp(status().reason,"alarm_only"));
 /* An old buffered reply must not be credited to a new transmission. */
 init();input("PRODUCT SN: OLD123456\r\n",22);start();tick+=2001;process();
 assert(status().state==RADAR_PROBE_UNCONFIRMED&&status().rx_bytes==0);
 init();input("PRODUCT SN: BUFFERED9\r\n",22); /* Put a second stale reply in DMA, without parsing. */
 memcpy(rx+rx_at,"PRODUCT SN: STALE1234\r\n",22);rx_at+=22;dma.counter=4096-rx_at;
 start();tick+=2001;process();assert(status().state==RADAR_PROBE_UNCONFIRMED&&status().sn[0]==0);
 /* No line terminator is required by the manufacturer's old fixed-length SN reply. */
 init();start();input("PRODUCT SN: 123456789",21);tick+=51;process();
 assert(status().state==RADAR_PROBE_CONFIRMED&&!strcmp(status().sn,"123456789"));
 init();start();input("PRODUCT SN:",11);tick+=2001;process();assert(status().state==RADAR_PROBE_UNCONFIRMED);
 init();start();input("PRODUCT SN: GOOD",16);RadarUart_ErrorCallback(&uart);process();
 tick+=2001;process();assert(status().state==RADAR_PROBE_UNCONFIRMED); /* Interrupted partial SN. */
 init();complete=false;assert(RadarUart_ProbeStart());process();tick+=501;process();
 assert(status().state==RADAR_PROBE_ERROR&&!status().tx_complete&&!strcmp(status().reason,"tx_timeout"));
 init();force_busy=true;assert(RadarUart_ProbeStart());process();tick+=501;process();
 assert(status().state==RADAR_PROBE_ERROR&&sends==0&&!strcmp(status().reason,"tx_busy"));
 init();tx_result=HAL_ERROR;assert(RadarUart_ProbeStart());process();
 assert(status().state==RADAR_PROBE_ERROR&&!strcmp(status().reason,"tx_error"));
 /* Stop cancels the query and retains evidence; its own command still gets sent. */
 init();start();input(alarm,6);RadarUart_StopScan();tick+=21;process();process();
 assert(status().state==RADAR_PROBE_CANCELLED&&!strcmp(sent,"LSTOPH")&&status().alarms==1);
 RadarUart_StatusGet(&d);assert(d.commands_sent==2&&!strcmp(d.last_tx,"LSTOPH"));
 init();assert(RadarUart_StartScan());assert(!RadarUart_ProbeStart());
 puts("radar_probe: SN response, alarm/echo/no-reply, stale DMA, TX errors, stop and command isolation PASS");
}
