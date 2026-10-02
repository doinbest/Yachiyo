#include "lds_parser.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static LdsParser_t parser;
static void put16(uint8_t *p, unsigned v) { p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8); }
static unsigned packet(uint8_t *p,bool fixed,unsigned start,unsigned sector,unsigned count)
{
  unsigned i,at=fixed?8:6,sum=count+start+(fixed?sector:0);
  p[0]=fixed?0xcf:0xce;p[1]=0xfa;put16(p+2,count);put16(p+4,start);
  if(fixed)put16(p+6,sector);
  for(i=0;i<count;i++){p[at]=17;put16(p+at+1,100+i);sum+=117+i;at+=3;}
  put16(p+at,sum);return at+2;
}
static unsigned feed(const uint8_t *p,unsigned n)
{unsigned i,events=0;for(i=0;i<n;i++)if(LdsParser_FeedByte(&parser,p[i]))events++;return events;}
int main(void)
{
  uint8_t frame[LDS_MAX_FRAME_BYTES],status[]={0x53,0x54,0x0f,0,0,0,0x45,0x44};
  uint8_t alarm[]={0xce,0xce,0xce,0xce,0x34,0x12};unsigned n,i;
  LdsParser_Init(&parser);n=packet(frame,false,3590,0,256);
  assert(feed(frame,n)==1);assert(LdsParser_Event(&parser)->data.packet.count==256);
  assert(LdsParser_Event(&parser)->data.packet.points[255].distance==355);
  n=packet(frame,true,10,150,3);
  for(i=0;i<n;i++)assert(LdsParser_FeedByte(&parser,frame[i])==(i==n-1));
  assert(LdsParser_Event(&parser)->data.packet.fixed);
  assert(LdsParser_Event(&parser)->data.packet.sector_angle_tenths==150);
  assert(feed(status,sizeof status)==1 && LdsParser_Event(&parser)->data.status_flags==15);
  assert(feed(alarm,sizeof alarm)==1 && LdsParser_Event(&parser)->data.alarm_code==0x1234);
  assert(parser.stats.alarms==1);
  n=packet(frame,false,80,0,1);frame[n-1]^=1;
  assert(feed(frame,n)==0 && parser.stats.bad_checksum==1);
  /* A missing checksum byte consumes the next CE; resync must retain that CE. */
  n=packet(frame,false,100,0,2);assert(feed(frame,n-1)==0);
  n=packet(frame,false,200,0,1);assert(feed(frame,n)==1);
  assert(LdsParser_Event(&parser)->data.packet.start_angle_tenths==200);
  frame[0]=0xcf;frame[1]=0xfa;put16(frame+2,257);
  assert(feed(frame,4)==0 && parser.stats.bad_length);
  n=packet(frame,false,300,0,1);assert(feed(frame,n)==1);
  status[7]=0;assert(feed(status,sizeof status)==0);
  assert(parser.stats.bad_length>=2);
  LdsParser_Reset(&parser);assert(parser.used==0 && parser.stats.packets==4);
  puts("radar_parser: fragmented CEFA/CFFA/ST/alarm, checksum and lost-byte resync PASS");return 0;
}
