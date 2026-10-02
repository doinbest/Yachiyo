#include "radar_scan.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t tick;
static DMA_HandleTypeDef dma;
static UART_HandleTypeDef uart = {USART2, &dma, 0};
static uint8_t *rx;
static uint16_t rx_size, rx_at;
static HAL_UART_RxEventTypeTypeDef rx_type;
static bool tx_busy, force_busy;
static unsigned tx_count;
static unsigned receive_failures;
static uint32_t tx_ticks[256];
static char last_tx[7];
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ assert(u==&uart && n==4096);if(receive_failures){receive_failures--;return HAL_ERROR;}rx=p;rx_size=n;rx_at=0;dma.counter=n;return HAL_OK; }
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *u)
{ (void)u;return rx_type; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u){(void)u;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u){(void)u;tx_busy=false;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{
  assert(u==&uart && n==6);
  if(force_busy||tx_busy)return HAL_BUSY;
  memcpy(last_tx,p,6);last_tx[6]=0;tx_busy=true;tx_ticks[tx_count++]=tick;return HAL_OK;
}
static void service(void)
{ RadarScan_Process();if(tx_busy){tx_busy=false;RadarUart_TxCpltCallback(&uart);} }
static void configured(void)
{ unsigned i;for(i=0;i<14;i++){service();tick+=20;}assert(!strcmp(last_tx,"LSTARH")); }
static void input(const uint8_t *p,unsigned n)
{
  unsigned i;for(i=0;i<n;i++){
    rx[rx_at++]=p[i];
    if(rx_at==rx_size){rx_at=0;dma.counter=rx_size;rx_type=HAL_UART_RXEVENT_TC;RadarUart_RxEventCallback(&uart,rx_size);}
    else {dma.counter=rx_size-rx_at;if(rx_at==rx_size/2){rx_type=HAL_UART_RXEVENT_HT;RadarUart_RxEventCallback(&uart,rx_at);}}
  }
  rx_type=HAL_UART_RXEVENT_IDLE;RadarUart_RxEventCallback(&uart,rx_at);
}
static void put16(uint8_t *p,unsigned v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static unsigned packet(uint8_t *p,bool fixed,unsigned start,unsigned count)
{
  unsigned i,at=fixed?8:6,sum=count+start+(fixed?900:0);
  p[0]=fixed?0xcf:0xce;p[1]=0xfa;put16(p+2,count);put16(p+4,start);if(fixed)put16(p+6,900);
  for(i=0;i<count;i++){p[at]=17;put16(p+at+1,100);sum+=117;at+=3;}
  put16(p+at,sum);return at+2;
}
static void measure(bool fixed,unsigned start,unsigned count,bool bad)
{
  uint8_t p[LDS_MAX_FRAME_BYTES];unsigned n=packet(p,fixed,start,count),i;
  if(bad)p[n-1]^=1;
  input(p,n);for(i=0;i<6;i++)service();tick++;
}
static void full_circle(bool fixed,unsigned count)
{ measure(fixed,2700,count,false);measure(fixed,0,count,false);measure(fixed,900,count,false);measure(fixed,1800,count,false);measure(fixed,2700,count,false);measure(fixed,0,count,false); }
int main(void)
{
  RadarScan_Status_t s;RadarUart_Status_t us;RadarMap_Params_t params;
  uint16_t count;const RadarSample_t *points;uint32_t id;unsigned i;
  uint8_t junk[4096];memset(junk,0,sizeof junk);
  assert(RadarScan_Init(&uart)==HAL_OK);RadarScan_StatusGet(&s);assert(s.state==RADAR_SCAN_IDLE&&tx_count==0);
  RadarMap_Defaults(&params);assert(RadarScan_Start(&params,7));force_busy=true;service();assert(tx_count==0);
  force_busy=false;configured();assert(tx_count==6);
  for(i=1;i<6;i++)assert(tx_ticks[i]-tx_ticks[i-1]>=20);
  full_circle(false,32);RadarScan_StatusGet(&s);
  assert(s.state==RADAR_SCAN_READY&&s.map_valid&&s.points_valid&&s.map_id==1&&s.scan_id==1&&s.origin_generation==7);
  assert(s.coverage_tenths==3600&&s.point_count==128&&s.unit==RADAR_UNIT_UNKNOWN);
  points=RadarScan_Points(&count);assert(points&&count==128&&points[0].angle_tenths==0&&points[31].angle_tenths==871);
  id=s.map_id;assert(RadarScan_Start(&params,8));configured();RadarScan_StatusGet(&s);
  assert(s.map_valid&&!s.points_valid&&RadarScan_Map()!=0&&RadarScan_Points(&count)==0);
  measure(true,2700,4,false);measure(true,0,4,false);measure(true,900,4,true);
  RadarScan_StatusGet(&s);assert(s.circle_restarts&&s.state==RADAR_SCAN_WAIT_WRAP&&s.map_id==id);
  full_circle(true,4);RadarScan_StatusGet(&s);assert(s.state==RADAR_SCAN_READY&&s.map_id==id+1);
  assert(RadarScan_Start(&params,9));configured();measure(false,2700,4,false);measure(false,0,4,false);
  {uint8_t st[]={0x53,0x54,0x0e,0,0,0,0x45,0x44};input(st,sizeof st);service();}
  RadarScan_StatusGet(&s);assert(s.unit==RADAR_UNIT_CM&&s.state==RADAR_SCAN_WAIT_WRAP&&s.unit_changes==1);
  full_circle(false,4);points=RadarScan_Points(&count);assert(points&&points[0].distance_mm==1000);
  assert(RadarScan_SetParam("threshold",5));RadarScan_StatusGet(&s);assert(s.map_valid&&!s.map_applicable);
  assert(!RadarScan_SetParam("threshold",0));assert(RadarScan_MapParams()->threshold==3);
  assert(RadarScan_Start(RadarScan_Params(),10));configured();full_circle(false,4);RadarScan_StatusGet(&s);assert(s.map_applicable);
  /* DMA overwrites must be counted by absolute TC epochs, even while main stalls. */
  assert(RadarScan_Start(&params,11));configured();measure(true,2700,4,false);measure(true,0,4,false);
  input(junk,sizeof junk);input(junk,sizeof junk);input(junk,sizeof junk);service();
  RadarScan_StatusGet(&s);RadarUart_StatusGet(&us);assert(us.overwritten_bytes>=8192&&s.circle_restarts&&s.map_valid);
  full_circle(false,4);RadarScan_StatusGet(&s);assert(s.state==RADAR_SCAN_READY);
  /* A complete map survives timeout and an explicit stop. */
  id=s.map_id;assert(RadarScan_Start(&params,12));configured();tick+=10001;service();RadarScan_StatusGet(&s);
  assert(s.state==RADAR_SCAN_ERROR&&s.map_valid&&!s.points_valid&&s.map_id==id);RadarScan_Stop();
  /* Point-cloud truncation affects display only: all 9216 samples still map.
   * A derived world pose passed to Start does not change the user revision. */
  params=*RadarScan_Params();params.lidar_x_mm=1200;params.lidar_y_mm=1200;params.zero_deg=0;
  assert(RadarScan_Start(&params,13));configured();measure(false,2700,256,false);measure(false,0,256,false);
  for(i=100;i<3600;i+=100)measure(false,i,256,false);
  measure(false,0,256,false);RadarScan_StatusGet(&s);
  assert(s.state==RADAR_SCAN_READY&&s.truncated&&s.point_count==4096&&s.raw_points==9216&&s.map_applicable);
  assert(RadarScan_Map()->counts[12]==9216);
  /* Parameters edited during a capture take effect on its successor. */
  assert(RadarScan_Start(&params,14));configured();assert(RadarScan_SetParam("threshold",6));
  full_circle(false,4);RadarScan_StatusGet(&s);assert(!s.map_applicable&&RadarScan_MapParams()->threshold==5);
  /* Late TC after an IDLE that already observed wrap cannot count it twice. */
  RadarUart_StatusGet(&us);i=us.received_bytes;rx_at=4000;dma.counter=96;rx_type=HAL_UART_RXEVENT_IDLE;RadarUart_RxEventCallback(&uart,4000);
  rx_at=100;dma.counter=3996;RadarUart_RxEventCallback(&uart,100);RadarUart_StatusGet(&us);id=us.received_bytes;
  rx_type=HAL_UART_RXEVENT_TC;RadarUart_RxEventCallback(&uart,4096);RadarUart_StatusGet(&us);assert(us.received_bytes==id&&id>=i);
  /* A late HT at the same live DMA position is diagnostic only, not new data. */
  rx_type=HAL_UART_RXEVENT_HT;RadarUart_RxEventCallback(&uart,2048);RadarUart_StatusGet(&us);assert(us.received_bytes==id);
  rx_type=HAL_UART_RXEVENT_IDLE;RadarUart_RxEventCallback(&uart,100);RadarUart_StatusGet(&us);assert(us.received_bytes==id);
  /* 512-byte slice and TC/HT duplicate suppression. */
  assert(RadarScan_Init(&uart)==HAL_OK);input(junk,1000);RadarUart_Process();RadarUart_StatusGet(&us);assert(us.parsed_bytes<=512);
  /* An initialization failure stays local and is retried on the next service. */
  receive_failures=1;assert(RadarScan_Init(&uart)==HAL_ERROR);service();RadarUart_StatusGet(&us);
  assert(us.rx_ready&&us.restarts==1);assert(RadarScan_Start(&params,15));configured();
  input(junk,37);service();RadarUart_ErrorCallback(&uart);service();
  full_circle(false,4);RadarScan_StatusGet(&s);assert(s.state==RADAR_SCAN_READY&&s.map_valid);
  RadarUart_StatusGet(&us);assert(us.uart_errors==1&&us.restarts==2);
  {UART_HandleTypeDef other=uart;RadarUart_ErrorCallback(&other);RadarUart_StatusGet(&us);assert(us.uart_errors==1);}
  /* ST can arrive during configuration; retaining its cm flag is essential. */
  assert(RadarScan_Start(&params,16));
  {uint8_t st[]={0x53,0x54,0x0e,0,0,0,0x45,0x44};input(st,sizeof st);service();}
  configured();full_circle(false,4);points=RadarScan_Points(&count);assert(points&&points[0].distance_mm==1000);
  /* Crossing zero alone is insufficient if the starting packet was at 30deg.
   * Complete the remaining angular coverage before publishing. */
  assert(RadarScan_Start(&params,17));configured();measure(false,2700,4,false);measure(false,300,4,false);
  measure(false,1200,4,false);measure(false,2100,4,false);measure(false,3000,4,false);measure(false,50,4,false);
  RadarScan_StatusGet(&s);assert(s.state==RADAR_SCAN_COLLECTING&&s.coverage_tenths==3350);
  measure(false,300,4,false);RadarScan_StatusGet(&s);assert(s.state==RADAR_SCAN_READY&&s.coverage_tenths==3600);
  /* A genuine zero crossing need not have its last start in the final 90deg. */
  assert(RadarScan_Start(&params,18));configured();measure(false,2650,4,false);measure(false,100,4,false);
  measure(false,1000,4,false);measure(false,1900,4,false);measure(false,2800,4,false);measure(false,100,4,false);
  RadarScan_StatusGet(&s);assert(s.state==RADAR_SCAN_READY);
  puts("radar_scan: commands, full circle, corrupt-frame/unit/DMA recovery, snapshot retention PASS");return 0;
}
