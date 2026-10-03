#include "radar_map.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks, failures;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; \
  printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)
static int near(float a, float b) { return fabsf(a-b) < 0.02f; }

/* Catch rotation/reflection mistakes and double application of the map yaw. */
static void test_direction_and_history(void)
{
  RadarMap_Params_t p;
  RadarSample_t s = {0,1000,80};
  float x=0,y=0;
  const float expected[4][2]={{1170,230},{2170,1230},{3170,230},{2170,-770}};
  RadarMap_Defaults(&p);
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
  RadarMap_Defaults(&p);p.lidar_x_mm=p.lidar_y_mm=1200;p.threshold=1;
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

/* Catch ignored filters, threshold off-by-one and blanket mission-cell exemption. */
static void test_map_filters_and_station_obstacle(void)
{
  RadarMap_Params_t p;
  RadarMap_t m;
  RadarSample_t s={0,500,80};
  RadarMap_Defaults(&p);p.lidar_x_mm=p.lidar_y_mm=350;p.zero_deg=0;
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
  RadarMap_Finish(&m,&p);CHECK(m.mask==0x50144U && m.counts[2]==3);
  RadarPlan_t plan;CHECK(!RadarPlan_Build(&m,&plan));
  CHECK(!plan.valid && plan.failed_leg==3 && plan.count==0);
  p.lidar_x_mm=2150;p.lidar_y_mm=150;
  CHECK(!RadarMap_Add(&m,&p,&s)); /* x=2250 excluded for measurements. */
  p.lidar_x_mm=50;
  CHECK(RadarMap_Add(&m,&p,&s)); /* x=150 included. */
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

/* Only the mission's explicit entry and final parking approach may be diagonal. */
static void check_clear_path(const RadarPlan_t *p,uint32_t mask,bool mission)
{
  for(unsigned i=1;i<p->count;i++) {
    int x=p->points[i-1].x_mm,y=p->points[i-1].y_mm;
    int dx=p->points[i].x_mm-x,dy=p->points[i].y_mm-y;
    bool shortcut=mission && i==1 && x==2250 && y==150 &&
      p->points[i].x_mm==2100 && p->points[i].y_mm==1200 && p->points[i].station==5;
    bool final=mission && i+1==p->count && x==2100 && y==300 &&
      p->points[i].x_mm==2250 && p->points[i].y_mm==150 && p->points[i].station==1;
    CHECK((dx==0)!=(dy==0) || shortcut || final);
    int steps=abs(dx)+abs(dy);
    for(int n=0;n<=steps;n++) {
      int cell=cell_at(x+dx*n/steps,y+dy*n/steps);
      if(cell>=0)CHECK(!(mask&(1U<<cell)));
    }
  }
  for(unsigned i=1;i+1<p->count;i++)if(!p->points[i].station) {
    const RadarPlan_Point_t *a=&p->points[i-1],*b=&p->points[i],*c=&p->points[i+1];
    CHECK(!((a->x_mm==b->x_mm&&b->x_mm==c->x_mm) ||
            (a->y_mm==b->y_mm&&b->y_mm==c->y_mm)));
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
}

/* A middle blocked cell must stop the shortcut even when both endpoints are free. */
static void test_first_shortcut_and_final_staging(void)
{
  RadarMap_t m;RadarPlan_t p,q;
  RadarMap_Reset(&m);CHECK(RadarPlan_Build(&m,&p));
  CHECK(p.points[1].x_mm==2100 && p.points[1].y_mm==1200 &&
        p.points[1].station==5 && p.points[1].visit==1);
  CHECK(p.count>=3 && p.points[p.count-2].x_mm==2100 && p.points[p.count-2].y_mm==300 &&
        p.points[p.count-2].station==0 && p.points[p.count-2].visit==0);
  CHECK(p.points[p.count-1].x_mm==2250 && p.points[p.count-1].y_mm==150 &&
        p.points[p.count-1].station==1 && p.points[p.count-1].visit==8);
  m.mask|=1U<<9;CHECK(RadarPlan_Build(&m,&q));
  CHECK(q.points[1].x_mm!=2100 || q.points[1].y_mm!=1200 || !q.points[1].station);
  check_clear_path(&q,m.mask,true);
  RadarMap_Reset(&m);m.mask|=1U<<4;
  CHECK(!RadarPlan_Build(&m,&q) && q.failed_leg==1 && !q.valid && q.count==0);
  RadarMap_Reset(&m);m.mask|=1U<<14;
  CHECK(!RadarPlan_Build(&m,&q) && q.failed_leg==1 && !q.valid && q.count==0);
}

int main(void)
{
  test_direction_and_history();test_rotated_world_obstacles();
  test_map_filters_and_station_obstacle();test_mission_and_detour();test_first_shortcut_and_final_staging();
  printf("Radar map/planner: %u checks, %u failures\n",checks,failures);
  return failures?1:0;
}
