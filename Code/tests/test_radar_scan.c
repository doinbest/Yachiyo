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
/* Zero-span CF samples let independent board bearings stay exact while packet
 * starts still cover one rotation. Zero range fills carry no map evidence. */
static void point_packet(unsigned start,unsigned count,unsigned distance)
{
  uint8_t p[LDS_MAX_FRAME_BYTES];unsigned at=8,sum=count+start;
  p[0]=0xcf;p[1]=0xfa;put16(p+2,count);put16(p+4,start);put16(p+6,0);
  for(unsigned i=0;i<count;i++){p[at]=80;put16(p+at+1,distance);sum+=80+distance;at+=3;}
  put16(p+at,sum);input(p,at+2);for(unsigned i=0;i<6;i++)service();tick++;
}
static void board_circle(bool upper,bool lower,bool beyond_cloud)
{
  point_packet(2700,1,0);point_packet(0,256,0);
  if(beyond_cloud)for(unsigned angle=10;angle<=190;angle+=10)point_packet(angle,256,0);
  if(upper)point_packet(859,3,970); /* Upper physical board conflicts only under lower. */
  if(lower)point_packet(1033,3,1000); /* Lower physical board conflicts only under upper. */
  point_packet(2700,1,0);point_packet(0,1,0);
}

/* zero_deg=90 in the lower reference means 270 at the upper corner.
 * Independent 0.1deg/mm samples land in both protected front cells. */
static void manual_circle(RadarStart_Id_t start)
{
  point_packet(2700,1,0);point_packet(0,1,0);
  if(start==RADAR_START_UPPER){
    point_packet(133,3,1000);point_packet(864,3,1130);point_packet(2700,1,0);
  }else{
    point_packet(2700,1,0);point_packet(3326,3,2110);point_packet(3559,3,970);
  }
  point_packet(0,1,0);
}

static void test_nominal_manual_start(void)
{
  RadarMap_Params_t reference;RadarScan_Status_t s;RadarScan_StartStatus_t selected;
  RadarMap_Defaults(&reference);reference.zero_deg=90;
  assert(RadarScan_Init(&uart)==HAL_OK);
  assert(!RadarScan_StartAt(&reference,31,RADAR_START_UNKNOWN));
  assert(!RadarScan_StartAt(&reference,31,(RadarStart_Id_t)99));
  RadarScan_StartStatusGet(&selected);
  assert(!selected.scan_id && !strcmp(selected.result.reason,"not_scanned"));
  for(unsigned id=RADAR_START_UPPER;id<=RADAR_START_LOWER;id++){
    assert(RadarScan_StartAt(&reference,31+id,(RadarStart_Id_t)id));
    assert(!RadarScan_SelectStart(RADAR_START_LOWER));configured();
    if(id==RADAR_START_UPPER){
      /* Bad-frame recapture keeps the chosen corner, discards partial counts. */
      point_packet(2700,1,0);point_packet(0,1,0);point_packet(133,3,1000);
      measure(true,1800,4,true);
    }
    manual_circle((RadarStart_Id_t)id);
    RadarScan_StatusGet(&s);RadarScan_StartStatusGet(&selected);
    assert(s.state==RADAR_SCAN_READY && s.scan_id==id && s.map_id==id &&
      s.map_valid && s.map_applicable && s.points_valid && s.points_map_id==s.map_id);
    assert(s.origin_generation==31+id && s.coverage_tenths==3600 && s.raw_points==8);
    assert(selected.scan_id==id && !selected.automatic && selected.result.start_id==id &&
      !strcmp(selected.result.reason,"manual_reference"));
    unsigned index=id-1U;
    assert(selected.result.qr_counts[index]==3 && selected.result.raw_counts[index]==3 &&
      selected.result.conflict_mask[index]==RADAR_MAP_REQUIRED_CLEAR_MASK);
    assert(selected.result.qr_counts[1U-index]==0 && selected.result.raw_counts[1U-index]==0 &&
      selected.result.conflict_mask[1U-index]==0);
    assert(RadarScan_Map()->counts[14]==3 && RadarScan_Map()->counts[22]==3 &&
      RadarScan_Map()->mask==RADAR_MAP_FIXED_MASK);
    const RadarMap_Params_t *pose=RadarScan_MapParams();
    assert(pose->lidar_x_mm==(id==RADAR_START_UPPER?2330:2170) &&
      pose->lidar_y_mm==(id==RADAR_START_UPPER?2170:230) &&
      pose->zero_deg==(id==RADAR_START_UPPER?270:90));
    assert(RadarScan_Params()->zero_deg==180 && RadarScan_Params()->lidar_x_mm==2170);
    assert(!RadarScan_SelectStart(RADAR_START_UPPER));
    /* Extra packets after this one circle cannot publish another result. */
    full_circle(false,4);RadarScan_StatusGet(&s);assert(s.scan_id==id && s.map_id==id);
  }
  /* Invalid explicit input cannot destroy retained complete auto candidates. */
  assert(RadarScan_StartAuto(&reference,34));configured();board_circle(false,false,false);
  RadarScan_StartStatusGet(&selected);assert(selected.automatic && selected.result.start_id==RADAR_START_UNKNOWN);
  assert(!RadarScan_StartAt(&reference,34,RADAR_START_UNKNOWN));
  assert(RadarScan_SelectStart(RADAR_START_UPPER));assert(RadarScan_SelectStart(RADAR_START_LOWER));
  RadarScan_StatusGet(&s);RadarScan_StartStatusGet(&selected);
  RadarMap_t historical=*RadarScan_Map();RadarMap_Params_t historical_pose=*RadarScan_MapParams();
  uint32_t old_map=s.map_id,old_scan=s.scan_id;
  /* New explicit capture follows the same candidate/cloud cancellation rules. */
  assert(RadarScan_StartAt(&reference,35,RADAR_START_UPPER));
  RadarScan_StartStatus_t during;RadarScan_StartStatusGet(&during);
  assert(during.scan_id==selected.scan_id && during.automatic &&
    during.result.start_id==selected.result.start_id);
  assert(!RadarScan_SelectStart(RADAR_START_LOWER));RadarScan_Stop();
  RadarScan_StatusGet(&s);RadarScan_StartStatusGet(&during);
  assert(s.map_id==old_map && s.scan_id==old_scan && s.map_valid && !s.points_valid);
  assert(!memcmp(RadarScan_Map(),&historical,sizeof historical) &&
    !memcmp(RadarScan_MapParams(),&historical_pose,sizeof historical_pose));
  assert(during.scan_id==selected.scan_id && during.result.start_id==selected.result.start_id);
  assert(!RadarScan_SelectStart(RADAR_START_LOWER));
  assert(RadarScan_StartAt(&reference,36,RADAR_START_UPPER));configured();tick+=10001;service();
  RadarScan_StatusGet(&s);RadarScan_StartStatusGet(&during);
  assert(s.state==RADAR_SCAN_ERROR && s.map_id==old_map && s.scan_id==old_scan &&
    s.map_valid && during.scan_id==selected.scan_id && during.result.start_id==selected.result.start_id);
}
int main(void)
{
  RadarScan_Status_t s;RadarUart_Status_t us;RadarMap_Params_t params;
  RadarScan_StartStatus_t start_status;
  uint16_t count;const RadarSample_t *points;uint32_t id;unsigned i;
  uint8_t junk[4096];memset(junk,0,sizeof junk);
  assert(RadarScan_Init(&uart)==HAL_OK);RadarScan_StatusGet(&s);assert(s.state==RADAR_SCAN_IDLE&&tx_count==0);
  RadarScan_StartStatusGet(&start_status);assert(!start_status.scan_id && !strcmp(start_status.result.reason,"not_scanned"));
  assert(!RadarScan_SelectStart(RADAR_START_LOWER));
  RadarMap_Defaults(&params);assert(RadarScan_Start(&params,7));force_busy=true;service();assert(tx_count==0);
  force_busy=false;configured();assert(tx_count==6);
  for(i=1;i<6;i++)assert(tx_ticks[i]-tx_ticks[i-1]>=20);
  full_circle(false,32);RadarScan_StatusGet(&s);
  assert(s.state==RADAR_SCAN_READY&&s.map_valid&&s.points_valid&&s.map_id==1&&s.scan_id==1&&s.origin_generation==7);
  assert(s.coverage_tenths==3600&&s.point_count==128&&s.unit==RADAR_UNIT_UNKNOWN);
  points=RadarScan_Points(&count);assert(points&&count==128&&points[0].angle_tenths==0&&points[31].angle_tenths==871);
  RadarScan_StartStatusGet(&start_status);assert(start_status.scan_id==1&&!start_status.automatic&&start_status.result.start_id==RADAR_START_LOWER&&!strcmp(start_status.result.reason,"manual_reference"));
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
  /* Real fault signature: every received byte belongs to a six-byte alarm.
   * Expose its code; later valid measurements can still complete a new scan. */
  assert(RadarScan_Init(&uart)==HAL_OK);assert(RadarScan_Start(&params,19));configured();
  {uint8_t alarm[]={0xce,0xce,0xce,0xce,1,0};input(alarm,sizeof alarm);service();input(alarm,sizeof alarm);service();}
  RadarScan_StatusGet(&s);assert(s.bytes==12&&s.alarm==2&&s.last_alarm==1&&s.packets==0);
  assert(s.state==RADAR_SCAN_WAIT_WRAP&&s.unit==RADAR_UNIT_UNKNOWN&&!s.points_valid);
  tick+=10001;service();RadarScan_StatusGet(&s);assert(s.state==RADAR_SCAN_ERROR&&!strcmp(s.reason,"scan_timeout"));
  assert(RadarScan_Start(&params,20));configured();full_circle(false,4);RadarScan_StatusGet(&s);
  assert(s.state==RADAR_SCAN_READY&&s.points_valid&&s.alarm==2&&s.last_alarm==1);
  /* Read-only identity verification retains the complete map, cloud and RAM. */
  {RadarUart_Probe_t probe;uint32_t mask=RadarScan_Map()->mask;uint16_t threshold=RadarScan_Params()->threshold;
   id=s.map_id;for(i=0;i<4;i++){tick+=20;service();}
   assert(RadarUart_ProbeStart());service();service();assert(!strcmp(last_tx,"LUUIDH"));
   input((const uint8_t *)"PRODUCT SN: ABC123456\r\n",22);for(i=0;i<6;i++)service();
   RadarUart_ProbeGet(&probe);RadarScan_StatusGet(&s);
   assert(probe.state==RADAR_PROBE_CONFIRMED&&s.state==RADAR_SCAN_READY&&s.map_id==id&&s.points_valid);
   assert(RadarScan_Map()->mask==mask&&RadarScan_Params()->threshold==threshold&&RadarScan_Points(&count));}
  /* Ambiguous auto circles retain the last map/pose, but cannot execute it as
   * the newly captured result; their complete raw cloud remains available. */
  RadarMap_t historical=*RadarScan_Map();RadarMap_Params_t historical_params=*RadarScan_MapParams();
  uint32_t historical_map=s.map_id,previous_scan=s.scan_id,historical_tick=s.completed_tick;
  RadarMap_Defaults(&params);assert(RadarScan_StartAuto(&params,21));configured();board_circle(false,false,false);
  RadarScan_StatusGet(&s);RadarScan_StartStatusGet(&start_status);
  assert(s.state==RADAR_SCAN_READY && s.scan_id==previous_scan+1 && s.map_id==historical_map);
  assert(s.completed_tick==historical_tick);
  assert(s.map_valid && !s.map_applicable && s.points_valid && !s.points_map_id && !strcmp(s.reason,"start_ambiguous"));
  assert(!memcmp(RadarScan_Map(),&historical,sizeof historical) && !memcmp(RadarScan_MapParams(),&historical_params,sizeof historical_params));
  assert(start_status.automatic && start_status.scan_id==s.scan_id && start_status.result.start_id==RADAR_START_UNKNOWN);
  assert(RadarScan_Points(&count) && count==257);
  uint32_t circle_completed_tick=tick-1U;tick+=100;
  assert(!RadarScan_SelectStart(RADAR_START_UNKNOWN));assert(RadarScan_SelectStart(RADAR_START_LOWER));
  RadarScan_StatusGet(&s);RadarScan_StartStatusGet(&start_status);
  assert(s.scan_id==previous_scan+1 && s.map_id==historical_map+1 && s.points_map_id==s.map_id && s.map_applicable);
  assert(s.completed_tick==circle_completed_tick);
  assert(start_status.result.start_id==RADAR_START_LOWER && !strcmp(start_status.result.reason,"manual_selection"));
  RadarScan_Stop();RadarScan_StartStatusGet(&start_status);assert(start_status.result.start_id==RADAR_START_LOWER);
  assert(RadarScan_StartAuto(&params,22));assert(!RadarScan_SelectStart(RADAR_START_UPPER));RadarScan_Stop();
  RadarScan_StartStatusGet(&start_status);assert(start_status.scan_id==previous_scan+1);
  /* All raw samples feed classification, including evidence beyond 4096. */
  assert(RadarScan_StartAuto(&params,23));configured();board_circle(false,true,true);
  RadarScan_StatusGet(&s);RadarScan_StartStatusGet(&start_status);
  assert(s.state==RADAR_SCAN_READY && s.truncated && s.point_count==4096 && s.raw_points==5124);
  assert(start_status.result.start_id==RADAR_START_LOWER && start_status.result.qr_counts[0]==3 && start_status.result.qr_counts[1]==0);
  assert(s.map_applicable && s.points_map_id==s.map_id && RadarScan_MapParams()->lidar_y_mm==230);
  /* Contradictory frames never silently publish the lower assumption. */
  historical_map=s.map_id;assert(RadarScan_StartAuto(&params,24));configured();board_circle(true,true,false);
  RadarScan_StatusGet(&s);RadarScan_StartStatusGet(&start_status);
  assert(s.map_id==historical_map && !strcmp(s.reason,"start_both_conflict") && !s.points_map_id);
  assert(start_status.result.qr_counts[0]==3 && start_status.result.qr_counts[1]==3 && start_status.result.start_id==RADAR_START_UNKNOWN);
  previous_scan=s.scan_id;assert(RadarScan_SelectStart(RADAR_START_UPPER));RadarScan_StatusGet(&s);
  assert(s.scan_id==previous_scan && s.map_id==historical_map+1 && RadarScan_MapParams()->lidar_x_mm==2330 && RadarScan_MapParams()->lidar_y_mm==2170);
  assert(RadarScan_Map()->counts[14]==3 && !(RadarScan_Map()->mask&RADAR_MAP_REQUIRED_CLEAR_MASK));
  /* A damaged partial circle must discard its hypothesis counts as well. */
  assert(RadarScan_StartAuto(&params,25));configured();point_packet(2700,1,0);point_packet(0,1,0);point_packet(859,3,970);
  measure(true,1800,4,true);board_circle(false,false,false);RadarScan_StatusGet(&s);RadarScan_StartStatusGet(&start_status);
  assert(s.state==RADAR_SCAN_READY && s.circle_restarts && start_status.result.start_id==RADAR_START_UNKNOWN && !start_status.result.qr_counts[1]);
  test_nominal_manual_start();
  puts("radar_scan: commands, full circle, manual corners, corrupt-frame/unit/DMA recovery, alarms, snapshot retention PASS");return 0;
}
