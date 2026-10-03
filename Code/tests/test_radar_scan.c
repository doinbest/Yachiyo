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
static uint32_t tx_ticks[1024];
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
static void board_circle(bool upper,bool lower,bool beyond_cloud);
static uint32_t crc32_bytes(const uint8_t *bytes,unsigned count)
{
  uint32_t crc=0xffffffffU;
  for(unsigned i=0;i<count;i++){
    crc^=bytes[i];for(unsigned bit=0;bit<8;bit++)crc=(crc>>1)^((crc&1U)?0xedb88320U:0U);
  }
  return crc^0xffffffffU;
}
static void debug_input(uint8_t *expected,unsigned *length,const uint8_t *bytes,unsigned count)
{
  memcpy(expected+*length,bytes,count);*length+=count;input(bytes,count);
  for(unsigned i=0;i<8;i++)service();
}
/* Dropping unparsed status/alarm/bad bytes, using current parameters, or marking
 * a cropped circle complete must fail these tests. Only the HAL is simulated. */
static void test_debug_raw(void)
{
  RadarMap_Params_t p;RadarScan_Raw_t raw;RadarScan_Status_t s;
  uint8_t expected[2048],downloaded[RADAR_UART_RAW_BYTES],frame[LDS_MAX_FRAME_BYTES];
  RadarSample_t saved[128];RadarMap_t saved_map;uint16_t count;
  static const uint8_t prefix[]={0x53,0x54,0x0f,0,0,0,0x45,0x44,
    0xce,0xce,0xce,0xce,7,0,'P','R','O','D','U','C','T',' ','S','N',':',' ','T','1','\r','\n'};
  assert(crc32_bytes((const uint8_t *)"123456789",9)==0xcbf43926U);
  /* A recovered inner event ends before the last RX byte that exposed it.
   * Its location must come from buffered frame consumption, not Poll time. */
  {
    static const uint8_t nested[]={0xce,0xfa,4,0,100,0,0xce,0xfa,1,0,200,0,0,0,0,201,0,0,0,0};
    LdsEvent_t event;uint32_t end;RadarUart_Raw_t uart_raw;
    assert(RadarScan_Init(&uart)==HAL_OK);RadarUart_RawStart();RadarUart_RawGet(&uart_raw);
    input(nested,sizeof nested);RadarUart_Process();
    assert(RadarUart_ReadEventAt(&event,&end) && event.kind==LDS_EVENT_MEASUREMENT && event.data.packet.start_angle_tenths==200);
    assert(end-uart_raw.first_position==17);
    RadarUart_RawStop();RadarUart_RawGet(&uart_raw);
    assert(uart_raw.total==sizeof nested && RadarUart_RawRead(0,downloaded,sizeof downloaded)==sizeof nested && !memcmp(downloaded,nested,sizeof nested));
  }
  for(unsigned fixed=0;fixed<2;fixed++){
    RadarMap_Defaults(&p);assert(RadarScan_Init(&uart)==HAL_OK);
    RadarScan_RawStatusGet(&raw);assert(!raw.id && !raw.complete && !RadarScan_RawRead(0,downloaded,1));
    assert(RadarScan_DebugStart(&p,71,RADAR_START_LOWER));configured();
    unsigned length=0,n=packet(frame,fixed,1800,4);frame[n-1]^=1;
    debug_input(expected,&length,prefix,sizeof prefix);debug_input(expected,&length,frame,n);
    unsigned prelude=length;
    static const unsigned angles[]={2700,0,900,1800,2700,0};
    for(unsigned i=0;i<6;i++){n=packet(frame,fixed,angles[i],4);debug_input(expected,&length,frame,n);}
    RadarScan_StatusGet(&s);RadarScan_RawStatusGet(&raw);
    assert(s.state==RADAR_SCAN_READY && raw.complete && raw.id==1 && raw.scan_id==s.scan_id && raw.map_id==s.map_id);
    assert(raw.origin_generation==71 && raw.unit==RADAR_UNIT_MM && raw.start_id==RADAR_START_LOWER && !raw.automatic);
    assert(raw.raw.total==length && raw.raw.received==length && !raw.raw.truncated && !raw.raw.losses);
    assert(raw.circle_start==prelude+n && raw.circle_end==length && !raw.raw.window_start);
    assert(raw.raw.crc32==crc32_bytes(expected,length) && raw.bad==1 && raw.cloud_valid);
    assert(RadarScan_RawRead(0,downloaded,sizeof downloaded)==length && !memcmp(expected,downloaded,length));
    assert(!RadarScan_RawRead(length,downloaded,1));
    assert(RadarScan_RawRead(3,downloaded,7)==7 && !memcmp(expected+3,downloaded,7));
    unsigned expected_count=fixed?15U:16U; /* CF's final 360deg endpoint is excluded. */
    const RadarSample_t *points=RadarScan_Points(&count);assert(points && count==expected_count);
    memcpy(saved,points,count*sizeof(*points));saved_map=*RadarScan_Map();
    assert(RadarScan_SetParam("zero_deg",90));RadarScan_RawStatusGet(&raw);assert(raw.parameters.zero_deg==180);
    /* Replay the actual captured bytes through the real parser and assembler. */
    assert(RadarScan_Init(&uart)==HAL_OK);assert(RadarScan_Start(&p,71));configured();
    input(expected,length);for(unsigned i=0;i<100;i++)service();
    points=RadarScan_Points(&count);assert(points && count==expected_count);
    for(unsigned i=0;i<count;i++)assert(points[i].angle_tenths==saved[i].angle_tenths &&
      points[i].distance_mm==saved[i].distance_mm && points[i].energy==saved[i].energy);
    assert(!memcmp(RadarScan_Map(),&saved_map,sizeof saved_map));
    RadarScan_RawStatusGet(&raw);assert(!raw.id);
  }
  /* A long prelude rolls away, but the final circle remains downloadable. */
  assert(RadarScan_Init(&uart)==HAL_OK);RadarMap_Defaults(&p);
  assert(RadarScan_DebugStart(&p,72,RADAR_START_UPPER));configured();
  memset(downloaded,0x21,1000);
  for(unsigned i=0;i<20;i++){input(downloaded,1000);for(unsigned j=0;j<4;j++)service();}
  full_circle(true,4);RadarScan_RawStatusGet(&raw);RadarScan_StatusGet(&s);
  assert(raw.complete && raw.raw.truncated && raw.raw.total==RADAR_UART_RAW_BYTES && raw.raw.window_start>0);
  assert(raw.circle_start>=raw.raw.window_start && raw.circle_end<=raw.raw.window_start+raw.raw.total);
  assert(raw.parameters.lidar_x_mm==2330 && raw.parameters.lidar_y_mm==2170 && raw.parameters.zero_deg==0);
  assert(RadarScan_RawRead(0,downloaded,sizeof downloaded)==sizeof downloaded && raw.raw.crc32==crc32_bytes(downloaded,sizeof downloaded));
  uint32_t raw_id=raw.id;assert(RadarScan_Start(&p,73));RadarScan_RawStatusGet(&raw);
  assert(raw.id==raw_id && raw.complete && !raw.cloud_valid);configured();full_circle(false,4);
  RadarScan_RawStatusGet(&raw);assert(raw.id==raw_id && !raw.cloud_valid);
  /* Partial stop and timeout publish diagnostic bytes, never a complete circle. */
  assert(RadarScan_DebugStart(&p,74,RADAR_START_LOWER));configured();input(prefix,sizeof prefix);service();RadarScan_Stop();
  RadarScan_RawStatusGet(&raw);assert(raw.id==raw_id+1 && raw.state==RADAR_SCAN_STOPPED && !raw.complete && !raw.scan_id && !raw.map_id);
  assert(raw.raw.total==sizeof prefix && RadarScan_RawRead(0,downloaded,sizeof downloaded)==sizeof prefix);
  assert(RadarScan_DebugStart(&p,75,RADAR_START_UNKNOWN));configured();input(prefix,sizeof prefix);service();tick+=10001;service();
  RadarScan_RawStatusGet(&raw);assert(raw.state==RADAR_SCAN_ERROR && !raw.complete && raw.automatic);
  /* More than 16KB in the final circle is explicit truncation, not replayable. */
  assert(RadarScan_DebugStart(&p,76,RADAR_START_LOWER));configured();measure(false,2700,256,false);measure(false,0,256,false);
  for(unsigned angle=100;angle<3600;angle+=100)measure(false,angle,256,false);
  measure(false,0,256,false);RadarScan_RawStatusGet(&raw);RadarScan_StatusGet(&s);
  assert(s.state==RADAR_SCAN_READY && raw.scan_id==s.scan_id && raw.raw.truncated && !raw.complete);
  assert(raw.circle_start<raw.raw.window_start && raw.point_total==9216 && raw.cloud_truncated);
  /* DMA loss is recorded independently of parsing; a later intact circle can recover. */
  assert(RadarScan_DebugStart(&p,77,RADAR_START_LOWER));configured();
  for(unsigned i=0;i<3;i++)input(downloaded,4096);
  service();full_circle(false,4);
  RadarScan_RawStatusGet(&raw);assert(raw.raw.losses && raw.complete && raw.circle_start>=raw.raw.window_start);
  /* An unresolved auto circle binds map=0; selecting it later does not rewrite
   * the already frozen diagnostic model or attach the older published map. */
  RadarScan_StatusGet(&s);uint32_t historical_map=s.map_id;
  assert(RadarScan_DebugStart(&p,78,RADAR_START_UNKNOWN));configured();board_circle(false,false,false);
  RadarScan_RawStatusGet(&raw);RadarScan_StatusGet(&s);
  assert(raw.complete && raw.cloud_valid && raw.scan_id==s.scan_id && !raw.map_id && s.map_id==historical_map);
  assert(raw.points==257 && !raw.start_id && raw.automatic && raw.parameters.lidar_x_mm==2170);
  assert(RadarScan_SelectStart(RADAR_START_UPPER));RadarScan_RawStatusGet(&raw);
  assert(!raw.map_id && !raw.start_id && raw.parameters.lidar_x_mm==2170 && raw.cloud_valid);
  assert(RadarScan_DebugStart(&p,79,RADAR_START_LOWER));configured();
  {static const uint8_t cm[]={0x53,0x54,0x0e,0,0,0,0x45,0x44};input(cm,sizeof cm);service();}
  full_circle(false,4);RadarScan_RawStatusGet(&raw);
  const RadarSample_t *points=RadarScan_Points(&count);
  assert(raw.complete && raw.unit==RADAR_UNIT_CM && points && points[0].distance_mm==1000);
}
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
static void test_live_cloud(void)
{
  RadarMap_Params_t p;RadarScan_LiveStatus_t live;RadarLive_Point_t points[RADAR_LIVE_BINS];
  RadarScan_Status_t saved_status,after;RadarScan_Raw_t saved_raw,raw_after;
  RadarScan_StartStatus_t saved_start,start_after;RadarMap_t saved_map;RadarSample_t saved_cloud[32];uint16_t count;
  /* The preceding debug scan supplies a complete static map/cloud/raw history. */
  RadarScan_StatusGet(&saved_status);RadarScan_RawStatusGet(&saved_raw);RadarScan_StartStatusGet(&saved_start);
  saved_map=*RadarScan_Map();const RadarSample_t *cloud=RadarScan_Points(&count);assert(count<=32);
  memcpy(saved_cloud,cloud,count*sizeof(*cloud));RadarMap_Defaults(&p);p.zero_deg=90;
  p.angle_min_tenths=p.angle_max_tenths=900; /* Live diagnostics cover all 360deg. */
  assert(!RadarScan_LiveStart(&p,81,RADAR_START_UNKNOWN));
  for(unsigned fixed=0;fixed<2;fixed++){
    unsigned sent=tx_count;
    assert(RadarScan_LiveStart(&p,81,RADAR_START_UPPER));assert(!RadarScan_LiveStart(&p,81,RADAR_START_LOWER));
    assert(!RadarScan_Start(&p,81) && !RadarScan_DebugStart(&p,81,RADAR_START_LOWER));
    RadarScan_LiveStatusGet(&live);assert(live.active && live.state==RADAR_LIVE_PREPARING && !live.sequence);
    assert(live.parameters.lidar_x_mm==2330 && live.parameters.lidar_y_mm==2170 && live.parameters.zero_deg==270);
    assert(!RadarScan_LiveCopy(points));configured();full_circle(fixed,4);
    RadarScan_LiveStatusGet(&live);assert(live.active && live.state==RADAR_LIVE_STREAMING && live.sequence==1);
    assert(live.points==(fixed?12:16) && tx_count==sent+6 && RadarScan_LiveCopy(points));
    assert(points[0].angle_tenths==0 && points[0].distance_mm==100);
    if(fixed)assert(points[10].angle_tenths==300 && points[10].distance_mm==100);
    else assert(points[7].angle_tenths==225 && points[7].distance_mm==100);
    /* The ending boundary packet starts the next revolution immediately. */
    measure(fixed,900,4,false);measure(fixed,1800,4,false);measure(fixed,2700,4,false);measure(fixed,0,4,false);
    RadarScan_LiveStatusGet(&live);assert(live.sequence==2 && tx_count==sent+6);
    measure(fixed,900,4,true);RadarScan_LiveStatusGet(&live);
    assert(live.active && live.sequence==2 && live.circle_restarts && live.state==RADAR_LIVE_WAITING);
    full_circle(fixed,4);RadarScan_LiveStatusGet(&live);assert(live.sequence==3 && tx_count==sent+6);
    tick+=10001;service();RadarScan_LiveStatusGet(&live);assert(live.active && live.age_ms>=10000);
    RadarScan_LiveStop();RadarScan_LiveStatusGet(&live);assert(!live.active && live.state==RADAR_LIVE_STOPPED);
    RadarScan_StatusGet(&after);RadarScan_RawStatusGet(&raw_after);RadarScan_StartStatusGet(&start_after);
    assert(after.scan_id==saved_status.scan_id && after.map_id==saved_status.map_id && after.points_valid &&
      after.state==saved_status.state && after.completed_tick==saved_status.completed_tick);
    assert(raw_after.id==saved_raw.id && raw_after.complete==saved_raw.complete && raw_after.raw.crc32==saved_raw.raw.crc32 && raw_after.cloud_valid);
    assert(start_after.scan_id==saved_start.scan_id && start_after.result.start_id==saved_start.result.start_id);
    assert(!memcmp(RadarScan_Map(),&saved_map,sizeof saved_map));
    cloud=RadarScan_Points(&count);for(unsigned i=0;i<count;i++)assert(cloud[i].angle_tenths==saved_cloud[i].angle_tenths &&
      cloud[i].distance_mm==saved_cloud[i].distance_mm && cloud[i].energy==saved_cloud[i].energy);
  }
  /* Same-bin returns keep the nearest accepted sample's actual raw angle. */
  assert(RadarScan_LiveStart(&p,82,RADAR_START_LOWER));configured();
  point_packet(2700,1,0);point_packet(0,1,1000);point_packet(10,1,500);point_packet(20,1,700);
  point_packet(30,1,0);point_packet(40,1,99);point_packet(50,1,50000);point_packet(60,1,200);
  point_packet(2700,1,0);point_packet(0,1,0);RadarScan_LiveStatusGet(&live);
  assert(live.sequence==1 && live.points==2 && RadarScan_LiveCopy(points));
  assert(points[0].angle_tenths==10 && points[0].distance_mm==500 && points[2].angle_tenths==60 && points[2].distance_mm==200);
  /* Status unit changes and DMA loss discard just the partial revolution. */
  {static const uint8_t cm[]={0x53,0x54,0x0e,0,0,0,0x45,0x44};input(cm,sizeof cm);service();}
  full_circle(false,4);RadarScan_LiveStatusGet(&live);assert(live.unit==RADAR_UNIT_CM && live.sequence==2);
  assert(RadarScan_LiveCopy(points) && points[0].distance_mm==1000);
  uint8_t junk[4096];memset(junk,0x21,sizeof junk);for(unsigned i=0;i<3;i++)input(junk,sizeof junk);
  service();RadarScan_LiveStatusGet(&live);assert(live.active && live.sequence==2 && live.state==RADAR_LIVE_WAITING);
  full_circle(false,4);RadarScan_LiveStatusGet(&live);assert(live.sequence==3 && live.state==RADAR_LIVE_STREAMING);
  RadarScan_Stop();RadarScan_LiveStatusGet(&live);RadarScan_StatusGet(&after);
  assert(!live.active && live.state==RADAR_LIVE_STOPPED && after.state==saved_status.state && after.map_id==saved_status.map_id);
  p.energy_min=100;assert(RadarScan_LiveStart(&p,83,RADAR_START_LOWER));configured();full_circle(true,4);
  RadarScan_LiveStatusGet(&live);assert(live.active && live.sequence==1 && !live.points);
  assert(RadarScan_LiveCopy(points));for(unsigned i=0;i<120;i++)assert(!points[i].distance_mm);
  RadarScan_LiveStop();
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
  test_nominal_manual_start();test_debug_raw();test_live_cloud();
  puts("radar_scan: static/raw CE/CF replay, continuous live bins/unit/filter/recovery/history, commands and snapshot retention PASS");return 0;
}
