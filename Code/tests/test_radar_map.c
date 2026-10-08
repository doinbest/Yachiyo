#include "radar_map.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks, failures;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; \
  printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)
static int near(float a, float b) { return fabsf(a-b) < 0.02f; }

static void test_generic_grid_core(void)
{
  RadarGrid_t grid;
  uint8_t path[RADAR_MAP_CELL_COUNT],count=0;
  int16_t x=0,y=0;

  RadarGrid_Default(&grid);
  CHECK(RadarGrid_Valid(&grid));
  CHECK(grid.x_mm[0]==150 && grid.y_mm[0]==150 && grid.x_mm[5]==2250 && grid.y_mm[5]==2250);
  CHECK(RadarGrid_FindCell(&grid,150.0f,150.0f,false)==0);
  CHECK(RadarGrid_FindCell(&grid,75.0f,1200.0f,false)<0);
  CHECK(RadarGrid_FindCell(&grid,2325.0f,1200.0f,false)<0);
  CHECK(RadarGrid_FindCell(&grid,2249.0f,2249.0f,false)==24);
  CHECK(RadarGrid_FindCell(&grid,2250.0f,2250.0f,false)<0);
  CHECK(RadarGrid_FindCell(&grid,2250.0f,2250.0f,true)==24);
  CHECK(RadarGrid_CellCenter(&grid,0,&x,&y) && x==350 && y==350);
  CHECK(RadarGrid_CellCenter(&grid,24,&x,&y) && x==2050 && y==2050);
  CHECK(RadarGrid_Path(&grid,RADAR_MAP_FIXED_MASK,0,4,path,&count));
  CHECK(count==5 && path[0]==0 && path[1]==1 && path[2]==2 && path[3]==3 && path[4]==4);

  grid.x_mm[1]=650;grid.y_mm[2]=1100;
  CHECK(RadarGrid_Valid(&grid));
  CHECK(RadarGrid_FindCell(&grid,650.0f,1100.0f,false)==11);
  CHECK(RadarGrid_CellCenter(&grid,1,&x,&y) && x==825 && y==350);
  grid.x_mm[2]=699;
  CHECK(!RadarGrid_Valid(&grid));
}

/* Catch rotation/reflection mistakes and double application of the map yaw. */
static void test_direction_and_history(void)
{
  RadarMap_Params_t p;
  RadarSample_t s = {0,1000,80};
  float x=0,y=0;
  const float expected[4][2]={{1250,150},{2250,1150},{3250,150},{2250,-850}};
  RadarMap_Defaults(&p);
  CHECK(p.angle_min_tenths==0 && p.angle_max_tenths==3600 && p.threshold==1);
  for(unsigned i=0;i<4;i++) {
    s.angle_tenths=(uint16_t)(i*900);
    CHECK(RadarMap_Project(&p,&s,&x,&y));
    CHECK(near(x,expected[i][0]) && near(y,expected[i][1]));
  }
  p.lidar_x_mm=p.lidar_y_mm=1200;p.zero_deg=90;s.angle_tenths=0;s.distance_mm=500;
  CHECK(RadarMap_Project(&p,&s,&x,&y) && near(x,1200) && near(y,1700));
  s.angle_tenths=900;
  CHECK(RadarMap_Project(&p,&s,&x,&y) && near(x,1700) && near(y,1200));
  RadarMap_LegacyPoint(230,230,&x,&y);CHECK(near(x,2170) && near(y,230));
  RadarMap_LegacyPoint(1200,350,&x,&y);CHECK(near(x,2050) && near(y,1200));
  CHECK(RadarMap_LegacyMask(1U)==16U);
  CHECK(RadarMap_LegacyMask(4U)==16384U);
  CHECK(RadarMap_LegacyMask(16384U)==4194304U);
  CHECK(RadarMap_LegacyMask(0x50140U)==0x50140U);
  CHECK(RadarMap_LegacyMask(0x404405U)==0x404414U);
}

/* The same world obstacles must be counted identically after sensor rotation. */
static void test_rotated_world_obstacles(void)
{
  const int yaw[]={0,90,180,270,450,-90};
  const uint16_t world_angles[]={0,900,1800,2700};
  RadarMap_Params_t p;RadarMap_t m,reference;RadarSample_t s={0,700,80};
  RadarMap_Defaults(&p);p.lidar_x_mm=p.lidar_y_mm=1200;p.threshold=1;p.angle_max_tenths=3600;
  for(unsigned rotation=0;rotation<sizeof(yaw)/sizeof(yaw[0]);rotation++) {
    p.zero_deg=(float)yaw[rotation];RadarMap_Reset(&m);
    for(unsigned obstacle=0;obstacle<4;obstacle++) {
      int local=(yaw[rotation]*10-(int)world_angles[obstacle])%3600;
      if(local<0)local+=3600;
      s.angle_tenths=(uint16_t)local;
      CHECK(RadarMap_Add(&m,&p,&s));
    }
    RadarMap_Finish(&m,&p);
    CHECK(m.counts[14]==1 && m.counts[22]==1 && m.counts[10]==1 && m.counts[2]==1);
    if(!rotation)reference=m;
    else CHECK(m.mask==reference.mask && !memcmp(m.counts,reference.counts,sizeof(m.counts)));
  }
}

/* The F1 protected start rotates to R1C5; do not erase another corner or station. */
static void test_scanned_start_corner_protection(void)
{
  RadarMap_Params_t p;RadarMap_t m;RadarPlan_t plan;
  RadarMap_Defaults(&p);p.lidar_x_mm=2170;p.lidar_y_mm=230;
  RadarMap_Reset(&m);m.counts[4]=200;m.counts[5]=1;m.counts[20]=1;
  RadarMap_FinishRaw(&m,&p);CHECK(m.mask&(1UL<<4));
  RadarMap_Finish(&m,&p);
  CHECK(m.mask==(RADAR_MAP_FIXED_MASK|(1UL<<5)|(1UL<<20)));
  CHECK(m.counts[4]==200 && m.counts[5]==1 && m.counts[20]==1);
  CHECK(RadarPlan_Build(&m,&plan) && plan.valid && plan.count>0);
  m.cloud_built=true;m.evidence_mask=(1UL<<4)|(1UL<<5)|(1UL<<20);
  RadarMap_Finish(&m,&p);CHECK(!(m.mask&(1UL<<4)) && (m.mask&(1UL<<5)));

  p.lidar_x_mm=2250;p.lidar_y_mm=150;
  RadarMap_Reset(&m);m.counts[4]=200;m.counts[24]=1;
  RadarMap_Finish(&m,&p);
  CHECK(!(m.mask&(1UL<<4)) && (m.mask&(1UL<<24)));

  p.lidar_x_mm=2250;p.lidar_y_mm=2250;
  RadarMap_Reset(&m);m.counts[24]=6;m.counts[4]=2;m.counts[2]=3;
  RadarMap_Finish(&m,&p);
  CHECK(!(m.mask&(1UL<<24)) && (m.mask&(1UL<<4)) && !(m.mask&(1UL<<2)));
  CHECK(m.counts[24]==6 && m.counts[4]==2 && m.counts[2]==3);

  p.lidar_x_mm=1200;p.lidar_y_mm=1200;
  RadarMap_Finish(&m,&p);CHECK((m.mask&(1UL<<24)) && (m.mask&(1UL<<4)));
}

/* Catch ignored filters, threshold off-by-one and blanket mission-cell exemption. */
static void test_map_filters_and_station_obstacle(void)
{
  RadarMap_Params_t p;
  RadarMap_t m;
  RadarSample_t s={0,500,80};
  RadarMap_Defaults(&p);p.lidar_x_mm=p.lidar_y_mm=350;p.zero_deg=0;p.threshold=3;
  RadarMap_Reset(&m);
  CHECK(RadarMap_ParamsValid(&p));
  CHECK(RadarMap_Add(&m,&p,&s));CHECK(RadarMap_Add(&m,&p,&s));
  RadarMap_Finish(&m,&p);CHECK(m.mask==0x50140U && m.counts[1]==2);
  CHECK(RadarMap_Add(&m,&p,&s));RadarMap_Finish(&m,&p);
  CHECK(m.mask==0x50142U && m.counts[1]==3);
  p.energy_min=81;CHECK(!RadarMap_Add(&m,&p,&s));p.energy_min=0;
  p.energy_max=79;CHECK(!RadarMap_Add(&m,&p,&s));p.energy_max=255;
  p.distance_min_mm=501;CHECK(!RadarMap_Add(&m,&p,&s));p.distance_min_mm=100;
  p.distance_max_mm=499;CHECK(!RadarMap_Add(&m,&p,&s));p.distance_max_mm=40000;
  p.angle_min_tenths=3500;p.angle_max_tenths=100;
  s.angle_tenths=3500;CHECK(RadarMap_Add(&m,&p,&s));
  s.angle_tenths=100;CHECK(RadarMap_Add(&m,&p,&s));
  s.angle_tenths=900;CHECK(!RadarMap_Add(&m,&p,&s));
  p.angle_min_tenths=0;p.angle_max_tenths=3600;s.angle_tenths=0;
  p.lidar_x_mm=1200;p.lidar_y_mm=350;s.distance_mm=100;
  RadarMap_Reset(&m);
  for(unsigned i=0;i<3;i++)CHECK(RadarMap_Add(&m,&p,&s));
  RadarMap_Finish(&m,&p);CHECK(m.mask==0x50140U && m.counts[2]==3 && (m.evidence_mask&(1UL<<2)));
  RadarPlan_t plan;CHECK(RadarPlan_Build(&m,&plan));
  CHECK(plan.valid && plan.count>0);
  m.mask|=1UL<<2;CHECK(!RadarPlan_Build(&m,&plan) && plan.failed_leg==3);
  p.lidar_x_mm=2150;p.lidar_y_mm=150;
  CHECK(RadarMap_Add(&m,&p,&s)); /* Reference rotation includes x=2250. */
  p.lidar_x_mm=50;
  CHECK(!RadarMap_Add(&m,&p,&s)); /* Reference rotation excludes x=150. */
  p.threshold=0;CHECK(!RadarMap_ParamsValid(&p));
  p.threshold=3;p.zero_deg=NAN;CHECK(!RadarMap_ParamsValid(&p));
}

static int cell_at(int x,int y)
{
  const int lines[]={150,550,1000,1400,1850,2250};
  int r=-1,c=-1;
  for(int i=0;i<5;i++){if(x>=lines[i]&&x<lines[i+1])c=i;if(y>=lines[i]&&y<lines[i+1])r=i;}
  return r<0||c<0?-1:r*5+c;
}

static void build_cloud(RadarMap_t *m,const RadarMap_Params_t *p,const RadarSample_t *s,unsigned n)
{
  RadarMap_BuildCloud(m,p,s,(uint16_t)n);
}

static RadarSample_t world_sample(float x,float y,const RadarMap_Params_t *p)
{
  float dx=x-p->lidar_x_mm,dy=y-p->lidar_y_mm;
  float angle=fmodf(p->zero_deg-atan2f(dy,dx)*57.2957795131f+360.0f,360.0f);
  RadarSample_t s={(uint16_t)lroundf(angle*10.0f),(uint16_t)lroundf(hypotf(dx,dy)),30};
  return s;
}

/* A thin foreground with one valid return must remain blocked by default. */
static void test_default_keeps_single_return_obstacle(void)
{
  RadarMap_Params_t p;RadarMap_t m;
  const RadarSample_t one[]={{400,1500,30}};
  RadarMap_Defaults(&p);
  build_cloud(&m,&p,one,1);
  CHECK(m.counts[12]==1 && m.mask==(RADAR_MAP_FIXED_MASK|(1UL<<12)));
  RadarMap_Reset(&m);
  CHECK(RadarMap_Add(&m,&p,&one[0]));RadarMap_Finish(&m,&p);
  CHECK(m.mask==(RADAR_MAP_FIXED_MASK|(1UL<<12)));
}

/* Exterior neighbours never erase interior hits; cache order is irrelevant. */
static void test_cloud_exterior_contour_and_occlusions(void)
{
  RadarMap_Params_t p;RadarMap_t m;RadarSample_t samples[326];unsigned n=0;
  RadarMap_Defaults(&p);p.lidar_x_mm=2250;p.lidar_y_mm=150;p.threshold=2;
  for(unsigned angle=250;angle<=900;angle+=2) {
    uint16_t distance=(uint16_t)lroundf(1600.0f/sinf((float)angle*0.001745329252f));
    if(angle>=370 && angle<=376)distance=2180;
    if(angle>=542 && angle<=550)distance=1040;
    samples[n]=(RadarSample_t){(uint16_t)angle,distance,30};
    n++;
  }
  build_cloud(&m,&p,samples,n);
  RadarMap_t streamed;RadarMap_Reset(&streamed);
  for(unsigned i=0;i<n;i++)RadarMap_Add(&streamed,&p,&samples[i]);
  RadarMap_FinishRaw(&streamed,&p);
  CHECK(m.mask==streamed.mask && !memcmp(m.counts,streamed.counts,sizeof m.counts));
  CHECK(m.counts[13]>0 && m.counts[13]<p.threshold);
  CHECK(!(m.mask&(1UL<<13))); /* Each cell must reach the threshold itself. */
  CHECK(m.counts[17]>0 && m.counts[19]>0 && m.background_points==0);
  static RadarSample_t rotated[326];
  RadarMap_t reference=m;
  for(unsigned start=1;start<n;start++) {
    for(unsigned i=0;i<n;i++)rotated[i]=samples[(start+i)%n];
    build_cloud(&m,&p,rotated,n);
    CHECK(!memcmp(&m,&reference,sizeof(m)));
  }
}

/* Long interior objects, diagonals and additional objects cannot be erased. */
static void test_cloud_interior_objects_and_real_distance(void)
{
  RadarMap_Params_t p;RadarMap_t m;RadarSample_t line[27];
  RadarMap_Defaults(&p);p.lidar_x_mm=2250;p.lidar_y_mm=150;p.threshold=2;
  for(unsigned i=0;i<27;i++)line[i]=world_sample(600.0f+25.0f*(float)i,1100,&p);
  build_cloud(&m,&p,line,27);
  CHECK(m.mask==(RADAR_MAP_FIXED_MASK|(1UL<<11)|(1UL<<12)));
  CHECK(m.counts[11]+m.counts[12]==27);
  const RadarSample_t diagonal[]={{445,2500,30},{450,2500,30}};
  build_cloud(&m,&p,diagonal,2);
  CHECK(m.mask==(RADAR_MAP_FIXED_MASK|(1UL<<20)) && m.counts[20]==2);
  const RadarSample_t three[]={{150,1800,30},{152,1800,30},
    {400,1500,30},{402,1500,30},{650,1900,30},{652,1900,30}};
  build_cloud(&m,&p,three,6);
  CHECK(m.mask==(RADAR_MAP_FIXED_MASK|(1UL<<5)|(1UL<<12)|(1UL<<23)));
}

/* Evidence threshold is per cell, irrespective of continuous-object grouping. */
static void test_cloud_object_threshold_and_filters(void)
{
  RadarMap_Params_t p;RadarMap_t m;
  RadarMap_Defaults(&p);p.lidar_x_mm=2250;p.lidar_y_mm=150;p.threshold=3;
  const RadarSample_t cross[]={{543,1041,25},{545,1048,27},{548,1039,25},{550,1050,18},{552,1023,8}};
  build_cloud(&m,&p,cross,5);
  CHECK(m.counts[8]==3 && m.counts[13]==2);
  CHECK(m.mask==RADAR_MAP_FIXED_MASK);
  const RadarSample_t one[]={{400,1500,30}};
  build_cloud(&m,&p,one,1);CHECK(m.mask==RADAR_MAP_FIXED_MASK && m.counts[12]==1);
  p.threshold=2;
  const RadarSample_t separate[]={{370,1500,30},{420,1500,30}};
  build_cloud(&m,&p,separate,2);
  CHECK(m.mask==(RADAR_MAP_FIXED_MASK|(1UL<<12))); /* Two independent hits in cell12. */
  p.angle_max_tenths=900;
  const RadarSample_t rejected[]={{901,1000,30},{902,1000,30},{0,2300,30},{1,2300,30}};
  build_cloud(&m,&p,rejected,4);CHECK(m.mask==RADAR_MAP_FIXED_MASK);
  p.energy_min=31;build_cloud(&m,&p,one,1);
  CHECK(m.mask==RADAR_MAP_FIXED_MASK && m.counts[12]==0);
  p.energy_min=0;
  const RadarSample_t required[]={{818,1060,30},{820,1060,30}};
  build_cloud(&m,&p,required,2);CHECK(m.mask&(1UL<<14));
  RadarMap_Finish(&m,&p);CHECK(!(m.mask&(1UL<<14)) && m.counts[14]==2);
}

/* Cache index0 is arbitrary: a complete circle must retain the same objects. */
static void test_cloud_cyclic_start_and_sector_boundaries(void)
{
  static RadarSample_t ordered[360],rotated[360];
  RadarMap_Params_t p;RadarMap_t reference,m;
  RadarMap_Defaults(&p);
  p.threshold=2;p.angle_max_tenths=900; /* Explicit selected-sector endpoint test. */
  for(unsigned i=0;i<360;i++) {
    unsigned angle=i*10;
    uint16_t distance=angle==90 || angle==100?800:angle==400?1500:0;
    ordered[i]=(RadarSample_t){(uint16_t)angle,distance,30};
  }
  build_cloud(&reference,&p,ordered,360);
  CHECK(reference.mask==(RADAR_MAP_FIXED_MASK|(1UL<<3)) && reference.counts[3]==2);
  for(unsigned start=0;start<360;start++) {
    for(unsigned i=0;i<360;i++)rotated[i]=ordered[(start+i)%360];
    build_cloud(&m,&p,rotated,360);
    CHECK(!memcmp(&m,&reference,sizeof(m)));
  }
  /* One valid endpoint cannot borrow support from an excluded raw angle. */
  p.lidar_y_mm=350; /* Keep wrapped rays inside the restored inset grid. */
  for(unsigned i=0;i<360;i++) {
    unsigned angle=i*10;
    uint16_t distance=angle==10 || angle==3590?800:angle==3550?1500:0;
    ordered[i]=(RadarSample_t){(uint16_t)angle,distance,30};
  }
  build_cloud(&m,&p,ordered,360);
  CHECK(m.mask==RADAR_MAP_FIXED_MASK && m.counts[3]==1);
  /* A deliberately wrapped selected sector is continuous across raw0. */
  p.angle_min_tenths=3500;p.angle_max_tenths=100;
  build_cloud(&reference,&p,ordered,360);
  CHECK(reference.mask==(RADAR_MAP_FIXED_MASK|(1UL<<3)) && reference.counts[3]==2);
  for(unsigned start=0;start<360;start+=7) {
    for(unsigned i=0;i<360;i++)rotated[i]=ordered[(start+i)%360];
    build_cloud(&m,&p,rotated,360);
    CHECK(!memcmp(&m,&reference,sizeof(m)));
  }
}

/* Only the mission's short entry and final parking approach may be diagonal. */
static void check_clear_path(const RadarPlan_t *p,uint32_t mask,bool mission)
{
  for(unsigned i=1;i<p->count;i++) {
    int x=p->points[i-1].x_mm,y=p->points[i-1].y_mm;
    int dx=p->points[i].x_mm-x,dy=p->points[i].y_mm-y;
    int start_y=p->start_id==RADAR_START_UPPER?2250:150;
    int staging_y=p->start_id==RADAR_START_UPPER?2100:300;
    bool shortcut=mission && i==1 && x==2250 && y==start_y &&
      p->points[i].x_mm==2100 && p->points[i].y_mm==staging_y && p->points[i].station==0;
    bool final=mission && i+1==p->count && x==2100 && y==staging_y &&
      p->points[i].x_mm==2250 && p->points[i].y_mm==start_y && p->points[i].station==1;
    CHECK((dx==0)!=(dy==0) || shortcut || final);
    int steps=abs(dx)+abs(dy);
    for(int n=0;n<=steps;n++) {
      int cell=cell_at(x+dx*n/steps,y+dy*n/steps);
      if(cell>=0)CHECK(!(mask&(1U<<cell)));
    }
  }
  for(unsigned i=1;i+1<p->count;i++)if(!p->points[i].station) {
    const RadarPlan_Point_t *a=&p->points[i-1],*b=&p->points[i],*c=&p->points[i+1];
    int dx1=b->x_mm-a->x_mm,dy1=b->y_mm-a->y_mm;
    int dx2=c->x_mm-b->x_mm,dy2=c->y_mm-b->y_mm;
    CHECK(!(((dx1==0&&dx2==0)||(dy1==0&&dy2==0)) && dx1*dx2+dy1*dy2>0));
  }
}

/* Catch changed mission order, lost visits, sensor-origin-as-car-origin and no reroute. */
static void test_mission_and_detour(void)
{
  const int expected[9][3]={{2250,150,1},{2100,1200,5},{1200,2100,4},
    {1200,350,2},{350,1200,3},{1200,2100,4},{1200,350,2},{350,1200,3},{2250,150,1}};
  RadarMap_t m;RadarPlan_t p,q;
  RadarMap_Reset(&m);CHECK(RadarPlan_Build(&m,&p));CHECK(p.valid && p.count<193);
  unsigned visit=0;
  for(unsigned i=0;i<p.count;i++)if(p.points[i].station) {
    CHECK(visit<9);
    if(visit<9)CHECK(p.points[i].x_mm==expected[visit][0] && p.points[i].y_mm==expected[visit][1] &&
      p.points[i].station==expected[visit][2] && p.points[i].visit==visit);
    visit++;
  }
  CHECK(visit==9);check_clear_path(&p,m.mask,true);
  CHECK(RadarPlan_Build(&m,&q));CHECK(p.count==q.count && !memcmp(p.points,q.points,p.count*sizeof(p.points[0])));
  m.mask|=1U<<12;CHECK(RadarPlan_Build(&m,&q));CHECK(q.valid);
  check_clear_path(&q,m.mask,true);
  CHECK(p.count!=q.count || memcmp(p.points,q.points,p.count*sizeof(p.points[0])));
  m.mask=0x50140U|(1U<<14);CHECK(!RadarPlan_Build(&m,&q));CHECK(q.failed_leg==1 && !q.valid && !q.count);
  m.mask=0x50140U|0x3E0U;CHECK(!RadarPlan_Build(&m,&q));CHECK(q.failed_leg==1);
  RadarMap_Reset(&m);
  CHECK(RadarPlan_BuildLeg(&m,2250,150,2100,1200,&q));check_clear_path(&q,m.mask,false);
  if(q.count)CHECK(q.points[0].x_mm==2250 && q.points[0].y_mm==150 && q.points[q.count-1].x_mm==2100 && q.points[q.count-1].y_mm==1200);
  CHECK(!RadarPlan_BuildLeg(&m,100,100,2100,1200,&q));
  CHECK(!RadarPlan_BuildLeg(&m,-1,100,2100,1200,&q));
}

/* Follow the shared F1 grid chain, not a separate clear cross-grid shortcut. */
static void test_station_connectors_without_jogs(void)
{
  RadarMap_t m;RadarPlan_t p;
  RadarMap_Reset(&m);m.mask|=(1UL<<5)|(1UL<<20);
  CHECK(RadarPlan_BuildLeg(&m,2100,1200,1200,2100,&p));
  CHECK(p.count==3);
  if(p.count==3)CHECK(p.points[1].x_mm==1200 && p.points[1].y_mm==1200);
  check_clear_path(&p,m.mask,false);
  CHECK(RadarPlan_BuildLeg(&m,350,1200,2100,300,&p));
  CHECK(p.count==4);
  if(p.count==4)CHECK(p.points[1].x_mm==2050 && p.points[1].y_mm==1200 &&
                     p.points[2].x_mm==2050 && p.points[2].y_mm==300);
  check_clear_path(&p,m.mask,false);
  CHECK(RadarPlan_BuildLeg(&m,1200,2100,1200,350,&p) && p.count==2);
  CHECK(RadarPlan_BuildLeg(&m,2010,2010,2100,2100,&p) && p.count==3);
  check_clear_path(&p,m.mask,false);
  /* Block both one-bend alternatives: the original grid detour must survive. */
  RadarMap_Reset(&m);m.mask|=(1UL<<3)|(1UL<<15);
  CHECK(RadarPlan_BuildLeg(&m,350,350,2050,2050,&p));
  CHECK(p.count>3);check_clear_path(&p,m.mask,false);
  /* A shortcut grazing a blocked cell's edge is not a clear shortcut. */
  RadarMap_Reset(&m);
  CHECK(RadarPlan_BuildLeg(&m,550,350,550,1200,&p));
  CHECK(p.count>2);check_clear_path(&p,m.mask,false);
  RadarMap_Reset(&m);m.mask|=(1UL<<5)|(1UL<<20);
  CHECK(RadarPlan_Build(&m,&p) && p.count<193);
  check_clear_path(&p,m.mask,true);
}

/* Publish the execution staging point; obstruction must still trigger a detour. */
static void test_first_shortcut_and_final_staging(void)
{
  RadarMap_t m;RadarPlan_t p,q;
  RadarMap_Reset(&m);CHECK(RadarPlan_Build(&m,&p));
  CHECK(p.points[1].x_mm==2100 && p.points[1].y_mm==300 &&
        p.points[1].station==0 && p.points[1].visit==0);
  CHECK(p.points[2].x_mm==2100 && p.points[2].y_mm==1200 &&
        p.points[2].station==5 && p.points[2].visit==1);
  CHECK(p.count>=3 && p.points[p.count-2].x_mm==2100 && p.points[p.count-2].y_mm==300 &&
        p.points[p.count-2].station==0 && p.points[p.count-2].visit==0);
  CHECK(p.points[p.count-1].x_mm==2250 && p.points[p.count-1].y_mm==150 &&
        p.points[p.count-1].station==1 && p.points[p.count-1].visit==8);
  m.mask|=1U<<9;CHECK(RadarPlan_Build(&m,&q));
  CHECK(q.points[1].x_mm!=2100 || q.points[1].y_mm!=300);
  check_clear_path(&q,m.mask,true);
  RadarMap_Reset(&m);m.mask|=1U<<4;
  CHECK(!RadarPlan_Build(&m,&q) && q.failed_leg==1 && !q.valid && q.count==0);
  RadarMap_Reset(&m);m.mask|=1U<<14;
  CHECK(!RadarPlan_Build(&m,&q) && q.failed_leg==1 && !q.valid && q.count==0);
}

/* The four task cells share F1 protection; manual obstacles remain authoritative. */
static void test_scanned_required_clear_cells(void)
{
  RadarMap_Params_t params;
  RadarMap_t map;
  RadarPlan_t plan;
  RadarMap_Defaults(&params);RadarMap_Reset(&map);
  map.counts[14]=3;map.counts[22]=3;map.counts[2]=3;map.counts[10]=3;
  RadarMap_Finish(&map,&params);
  CHECK(map.mask==RADAR_MAP_FIXED_MASK);
  CHECK(map.counts[14]==3 && map.counts[22]==3);
  CHECK(RadarPlan_Build(&map,&plan));
  map.mask|=1UL<<2;CHECK(!RadarPlan_Build(&map,&plan) && plan.failed_leg==3);
  RadarMap_Reset(&map);map.mask|=1UL<<14;
  CHECK(!RadarPlan_Build(&map,&plan) && plan.failed_leg==1);
  RadarMap_Reset(&map);map.counts[14]=3;map.counts[22]=3;
  RadarMap_FinishRaw(&map,&params);
  CHECK(map.mask==(RADAR_MAP_FIXED_MASK|(1UL<<14)|(1UL<<22)));
  RadarMap_Finish(&map,&params);
  CHECK(map.mask==RADAR_MAP_FIXED_MASK && map.counts[14]==3 && map.counts[22]==3);
}

/* The selected corner is a vehicle target; all detours retain station/visit order. */
static void test_dual_start_missions(void)
{
  const uint8_t stations[9]={1,5,4,2,3,4,2,3,1};
  const uint32_t obstacles[]={0U,1UL<<12,1UL<<9,1UL<<19,1UL<<7,1UL<<3,1UL<<23};
  RadarMap_t map;RadarPlan_t plan,legacy;
  for(unsigned id=RADAR_START_UPPER;id<=RADAR_START_LOWER;id++) {
    int start_y=id==RADAR_START_UPPER?2250:150;
    int staging_y=id==RADAR_START_UPPER?2100:300;
    for(unsigned mask=0;mask<sizeof(obstacles)/sizeof(obstacles[0]);mask++) {
      RadarMap_Reset(&map);map.mask|=obstacles[mask];
      CHECK(RadarPlan_BuildFromStart(&map,(RadarStart_Id_t)id,&plan));
      CHECK(plan.valid && plan.start_id==id && plan.count>2 && plan.count<193);
      if(plan.count<3)continue;
      CHECK(plan.points[0].x_mm==2250 && plan.points[0].y_mm==start_y);
      CHECK(plan.points[plan.count-1].x_mm==2250 && plan.points[plan.count-1].y_mm==start_y);
      CHECK(plan.points[plan.count-2].x_mm==2100 && plan.points[plan.count-2].y_mm==staging_y);
      unsigned visit=0;
      for(unsigned point=0;point<plan.count;point++)if(plan.points[point].station) {
        CHECK(visit<9);
        if(visit<9)CHECK(plan.points[point].station==stations[visit] && plan.points[point].visit==visit);
        visit++;
      }
      CHECK(visit==9);check_clear_path(&plan,map.mask,true);
      if(mask==0) {
        CHECK(plan.points[1].station==0 && plan.points[1].visit==0 &&
          plan.points[1].x_mm==2100 && plan.points[1].y_mm==staging_y);
        CHECK(plan.points[2].station==5 && plan.points[2].visit==1 &&
          plan.points[2].x_mm==2100 && plan.points[2].y_mm==1200);
      }
    }
    RadarMap_Reset(&map);map.mask|=1UL<<14;
    CHECK(!RadarPlan_BuildFromStart(&map,(RadarStart_Id_t)id,&plan));
    CHECK(!plan.valid && plan.count==0 && plan.failed_leg==1 && plan.start_id==id);
    RadarMap_Reset(&map);map.mask|=1UL<<22;
    CHECK(!RadarPlan_BuildFromStart(&map,(RadarStart_Id_t)id,&plan) && plan.failed_leg==2);
    RadarMap_Reset(&map);map.mask|=1UL<<(id==RADAR_START_UPPER?24:4);
    CHECK(!RadarPlan_BuildFromStart(&map,(RadarStart_Id_t)id,&plan) && plan.failed_leg==1);
  }
  RadarMap_Reset(&map);
  CHECK(!RadarPlan_BuildFromStart(&map,RADAR_START_UNKNOWN,&plan) && !plan.valid && !plan.count);
  CHECK(!RadarPlan_BuildFromStart(&map,(RadarStart_Id_t)99,&plan));
  CHECK(RadarPlan_Build(&map,&legacy));
  CHECK(RadarPlan_BuildFromStart(&map,RADAR_START_LOWER,&plan));
  CHECK(plan.start_id==RADAR_START_LOWER && !memcmp(&legacy,&plan,sizeof(plan)));
}

int main(void)
{
  test_generic_grid_core();
  test_direction_and_history();test_rotated_world_obstacles();
  test_scanned_start_corner_protection();
  test_map_filters_and_station_obstacle();test_mission_and_detour();test_first_shortcut_and_final_staging();
  test_scanned_required_clear_cells();
  test_station_connectors_without_jogs();
  test_dual_start_missions();
  test_cloud_exterior_contour_and_occlusions();
  test_cloud_interior_objects_and_real_distance();
  test_cloud_object_threshold_and_filters();
  test_default_keeps_single_return_obstacle();
  test_cloud_cyclic_start_and_sector_boundaries();
  printf("Radar map/planner: %u checks, %u failures\n",checks,failures);
  return failures?1:0;
}
