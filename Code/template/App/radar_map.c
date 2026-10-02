/**
 * @file radar_map.c
 * @brief 当前地图中的静止雷达建图及四邻居工位路线。
 * 算法依据：Code/reference/Radar_2026-10-01/Radar 的格点计数和毫米A*；
 * 本实现采用当前坐标、运行参数和精确车体工位锚点，不复制F1 HAL。
 */
#include "radar_map.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define RADAR_DEG_TO_RAD 0.01745329251994329577f
#define RADAR_CELL_NONE  0xFFU
#define RADAR_COST_MAX   0xFFFFFFFFUL

static const int16_t GridLines[6] = {150,550,1000,1400,1850,2250};
static const uint8_t MissionOrder[RADAR_PLAN_MISSION_VISITS] = {1,5,4,2,3,4,2,3,1};
/* Indexed by station-1: START, ROUGH, STORE, RAW, QR. Vehicle-center targets. */
static const int16_t Stations[5][2] = {
  {2250,150},{1200,350},{350,1200},{1200,2100},{2100,1200}
};

void RadarMap_Defaults(RadarMap_Params_t *p)
{
  if(p==NULL) return;
  memset(p,0,sizeof(*p));
  p->lidar_x_mm=2170.0f;p->lidar_y_mm=230.0f;p->zero_deg=180.0f;
  p->distance_min_mm=100U;p->distance_max_mm=40000U;
  p->angle_min_tenths=0U;p->angle_max_tenths=3600U;
  p->threshold=3U;p->energy_min=0U;p->energy_max=255U;
}

bool RadarMap_ParamsValid(const RadarMap_Params_t *p)
{
  return p!=NULL && isfinite(p->lidar_x_mm) && isfinite(p->lidar_y_mm) &&
    isfinite(p->zero_deg) && p->distance_min_mm<=p->distance_max_mm &&
    p->angle_min_tenths<=3600U && p->angle_max_tenths<=3600U &&
    p->energy_min<=p->energy_max && p->threshold>0U;
}

void RadarMap_Reset(RadarMap_t *m)
{
  if(m==NULL) return;
  memset(m,0,sizeof(*m));m->mask=RADAR_MAP_FIXED_MASK;
}

bool RadarMap_Project(const RadarMap_Params_t *p,const RadarSample_t *s,float *x,float *y)
{
  float angle,px,py;
  if(p==NULL || s==NULL || x==NULL || y==NULL ||
     !isfinite(p->lidar_x_mm) || !isfinite(p->lidar_y_mm) || !isfinite(p->zero_deg)) return false;
  angle=(fmodf(p->zero_deg,360.0f)-(float)(s->angle_tenths%3600U)*0.1f)*RADAR_DEG_TO_RAD;
  px=p->lidar_x_mm+(float)s->distance_mm*cosf(angle);
  py=p->lidar_y_mm+(float)s->distance_mm*sinf(angle);
  if(!isfinite(px) || !isfinite(py)) return false;
  *x=px;*y=py;return true;
}

static int axis_cell(float value,bool endpoint)
{
  unsigned i;
  if(!isfinite(value) || value<150.0f || value>2250.0f) return -1;
  if(value==2250.0f) return endpoint?4:-1;
  for(i=0U;i<5U;i++) if(value<(float)GridLines[i+1U]) return (int)i;
  return -1;
}

static int find_cell(float x,float y,bool endpoint)
{
  int c=axis_cell(x,endpoint),r=axis_cell(y,endpoint);
  return c<0 || r<0?-1:r*5+c;
}

bool RadarMap_Add(RadarMap_t *m,const RadarMap_Params_t *p,const RadarSample_t *s)
{
  uint16_t angle;
  float x,y;
  int cell;
  bool allowed;
  if(m==NULL || s==NULL || !RadarMap_ParamsValid(p)) return false;
  angle=(uint16_t)(s->angle_tenths%3600U);
  allowed=p->angle_min_tenths<=p->angle_max_tenths?
    angle>=p->angle_min_tenths && angle<=p->angle_max_tenths:
    angle>=p->angle_min_tenths || angle<=p->angle_max_tenths;
  if(!allowed || s->distance_mm<p->distance_min_mm || s->distance_mm>p->distance_max_mm ||
     s->energy<p->energy_min || s->energy>p->energy_max || !RadarMap_Project(p,s,&x,&y)) return false;
  cell=find_cell(x,y,false);
  if(cell<0) return false;
  if(m->counts[cell]<0xFFFFU) ++m->counts[cell];
  return true;
}

void RadarMap_Finish(RadarMap_t *m,const RadarMap_Params_t *p)
{
  unsigned i;
  if(m==NULL || !RadarMap_ParamsValid(p)) return;
  m->mask=RADAR_MAP_FIXED_MASK;
  for(i=0U;i<RADAR_MAP_CELL_COUNT;i++) if(m->counts[i]>=p->threshold) m->mask|=1UL<<i;
}

void RadarMap_LegacyPoint(float old_x,float old_y,float *x,float *y)
{
  if(x!=NULL) *x=2400.0f-old_y;
  if(y!=NULL) *y=old_x;
}

uint32_t RadarMap_LegacyMask(uint32_t old_mask)
{
  uint32_t mask=0U;
  unsigned r,c;
  for(r=0U;r<5U;r++) for(c=0U;c<5U;c++)
    if(old_mask&(1UL<<(r*5U+c))) mask|=1UL<<(c*5U+4U-r);
  return mask;
}

static void cell_center(uint8_t cell,int16_t *x,int16_t *y)
{
  unsigned r=cell/5U,c=cell%5U;
  *x=(int16_t)((GridLines[c]+GridLines[c+1U])/2);
  *y=(int16_t)((GridLines[r]+GridLines[r+1U])/2);
}

static uint32_t cell_distance(uint8_t a,uint8_t b)
{
  int16_t ax,ay,bx,by;
  int dx,dy;
  cell_center(a,&ax,&ay);cell_center(b,&bx,&by);
  dx=ax-bx;dy=ay-by;
  return (uint32_t)((dx<0?-dx:dx)+(dy<0?-dy:dy));
}

static bool cell_neighbor(uint8_t cell,unsigned dir,uint8_t *next)
{
  static const int8_t dr[4]={1,0,-1,0},dc[4]={0,1,0,-1};
  int r=(int)(cell/5U)+dr[dir],c=(int)(cell%5U)+dc[dir];
  if(r<0 || r>=5 || c<0 || c>=5) return false;
  *next=(uint8_t)(r*5+c);return true;
}

/* 25 nodes: linear open-set selection is small, bounded, and deterministic. */
static bool cell_path(uint32_t mask,uint8_t start,uint8_t goal,uint8_t path[25],uint8_t *count)
{
  uint32_t cost[25];
  uint8_t parent[25],open[25]={0},closed[25]={0},order[25],next_order=0U;
  uint8_t current,best,i,n;
  unsigned dir;
  *count=0U;
  if(mask&((1UL<<start)|(1UL<<goal))) return false;
  for(i=0U;i<25U;i++){cost[i]=RADAR_COST_MAX;parent[i]=RADAR_CELL_NONE;order[i]=RADAR_CELL_NONE;}
  cost[start]=0U;open[start]=1U;order[start]=next_order++;
  for(;;) {
    uint32_t best_f=RADAR_COST_MAX,best_h=RADAR_COST_MAX;
    best=RADAR_CELL_NONE;
    for(i=0U;i<25U;i++) if(open[i]) {
      uint32_t h=cell_distance(i,goal),f=cost[i]+h;
      if(best==RADAR_CELL_NONE || f<best_f || (f==best_f && h<best_h) ||
         (f==best_f && h==best_h && order[i]<order[best])) {best=i;best_f=f;best_h=h;}
    }
    if(best==RADAR_CELL_NONE) return false;
    current=best;
    if(current==goal) break;
    open[current]=0U;closed[current]=1U;
    for(dir=0U;dir<4U;dir++) if(cell_neighbor(current,dir,&n) && !closed[n] && !(mask&(1UL<<n))) {
      uint32_t tentative=cost[current]+cell_distance(current,n);
      if(tentative<cost[n]) {
        cost[n]=tentative;parent[n]=current;
        if(!open[n]){open[n]=1U;order[n]=next_order++;}
      }
    }
  }
  current=goal;
  do {
    if(*count>=25U) return false;
    path[(*count)++]=current;
    if(current==start) break;
    current=parent[current];
  } while(current!=RADAR_CELL_NONE);
  if(path[*count-1U]!=start) return false;
  for(i=0U;i<*count/2U;i++){n=path[i];path[i]=path[*count-1U-i];path[*count-1U-i]=n;}
  return true;
}

static bool append_point(RadarPlan_t *p,int16_t x,int16_t y,uint8_t station,uint8_t visit)
{
  RadarPlan_Point_t *last;
  if(p->count>0U) {
    last=&p->points[p->count-1U];
    if(last->x_mm==x && last->y_mm==y) {
      if(station!=0U){last->station=station;last->visit=visit;}
      return true;
    }
    if(p->count>1U && last->station==0U) {
      const RadarPlan_Point_t *before=&p->points[p->count-2U];
      int dx1=last->x_mm-before->x_mm,dy1=last->y_mm-before->y_mm;
      int dx2=x-last->x_mm,dy2=y-last->y_mm;
      if(((dx1==0 && dx2==0) || (dy1==0 && dy2==0)) && dx1*dx2+dy1*dy2>0) {
        last->x_mm=x;last->y_mm=y;last->station=station;last->visit=visit;return true;
      }
    }
  }
  if(p->count>=RADAR_PLAN_MAX_POINTS) return false;
  last=&p->points[p->count++];last->x_mm=x;last->y_mm=y;last->station=station;last->visit=visit;
  return true;
}

static bool append_leg(const RadarMap_t *m,int16_t sx,int16_t sy,int16_t ex,int16_t ey,
                       uint8_t station,uint8_t visit,RadarPlan_t *p)
{
  int start=find_cell((float)sx,(float)sy,true),goal=find_cell((float)ex,(float)ey,true);
  uint32_t mask=(m->mask|RADAR_MAP_FIXED_MASK)&RADAR_MAP_ALL_CELLS;
  uint8_t path[25],count,i;
  int16_t x,y;
  if(start<0 || goal<0 || !cell_path(mask,(uint8_t)start,(uint8_t)goal,path,&count)) return false;
  if(!append_point(p,sx,sy,0U,0U)) return false;
  if(start==goal) return append_point(p,ex,sy,0U,0U) && append_point(p,ex,ey,station,visit);
  cell_center(path[0],&x,&y);
  /* Axis-aligned connectors stay inside their unblocked endpoint cells. */
  if(!append_point(p,x,sy,0U,0U)) return false;
  for(i=0U;i<count;i++) {
    cell_center(path[i],&x,&y);
    if(!append_point(p,x,y,0U,0U)) return false;
  }
  return append_point(p,ex,y,0U,0U) && append_point(p,ex,ey,station,visit);
}

bool RadarPlan_Build(const RadarMap_t *m,RadarPlan_t *p)
{
  unsigned leg;
  if(p==NULL) return false;
  memset(p,0,sizeof(*p));
  if(m==NULL) return false;
  if(!append_point(p,Stations[0][0],Stations[0][1],1U,0U)) return false;
  for(leg=0U;leg+1U<RADAR_PLAN_MISSION_VISITS;leg++) {
    uint8_t from=MissionOrder[leg],to=MissionOrder[leg+1U];
    if(!append_leg(m,Stations[from-1U][0],Stations[from-1U][1],
                   Stations[to-1U][0],Stations[to-1U][1],to,(uint8_t)(leg+1U),p)) {
      p->count=0U;p->failed_leg=(uint8_t)(leg+1U);return false;
    }
  }
  p->valid=true;return true;
}

bool RadarPlan_BuildLeg(const RadarMap_t *m,float sx,float sy,float ex,float ey,RadarPlan_t *p)
{
  if(p==NULL) return false;
  memset(p,0,sizeof(*p));
  if(m==NULL || find_cell(sx,sy,true)<0 || find_cell(ex,ey,true)<0) return false;
  if(!append_leg(m,(int16_t)lroundf(sx),(int16_t)lroundf(sy),
                 (int16_t)lroundf(ex),(int16_t)lroundf(ey),0U,0U,p)) {
    p->count=0U;p->failed_leg=1U;return false;
  }
  p->valid=true;return true;
}
