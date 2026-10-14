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
#define RADAR_BOUNDARY_TOLERANCE_MM 40.0f
#define RADAR_BOUNDARY_GAP_MM 150.0f
#define RADAR_BOUNDARY_SPAN_MM 400.0f
#define RADAR_BOUNDARY_MIN_POINTS 8U

static const RadarGrid_t DefaultGrid = {
  {150,550,1000,1400,1850,2250},
  {150,550,1000,1400,1850,2250}
};
/* The physical field remains 0..2400; occupancy excludes its 150 mm edge belt.
 * Keep the proven vehicle-center corridors and station connectors unchanged. */
static const RadarGrid_t WaypointGrid = {
  {150,550,1000,1400,1850,2250},{150,550,1000,1400,1850,2250}
};
static const uint8_t MissionOrder[RADAR_PLAN_MISSION_VISITS] = {1,5,4,2,3,4,2,3,1};
/* Indexed by station-1: START, ROUGH, STORE, RAW, QR. Vehicle-center targets. */
static const int16_t Stations[5][2] = {
  {2250,150},{1200,380},{350,1200},{1200,2100},{2100,1200}
};

void RadarMap_Defaults(RadarMap_Params_t *p)
{
  if(p==NULL) return;
  memset(p,0,sizeof(*p));
  p->lidar_x_mm=2280.0f;p->lidar_y_mm=314.0f;p->zero_deg=180.0f;
  p->upper_lidar_x_mm=2050.0f;p->upper_lidar_y_mm=2250.0f;
  RadarGeometry_Default(&p->geometry,2U);p->situation=2U;p->grid_version=4U;
  p->start_id=RADAR_START_LOWER;
  p->distance_min_mm=100U;p->distance_max_mm=40000U;
  p->angle_min_tenths=0U;p->angle_max_tenths=3600U;
  p->threshold=1U;p->energy_min=0U;p->energy_max=255U;
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
  memset(m,0,sizeof(*m));m->mask=RADAR_MAP_FIXED_MASK;m->grid_version=4U;
}

void RadarMap_FreezeParams(const RadarMap_Params_t *p,RadarMap_Params_t *frozen)
{
  RadarMap_Params_t next=*p;
  if(next.situation){
    bool upper=next.grid_version>=4U && next.start_id==RADAR_START_UPPER;
    float nx=(float)next.geometry.origin_q16[0]/65536.0f;
    float ny=(float)next.geometry.origin_q16[1]/65536.0f;
    if(!upper){nx=roundf(nx);ny=roundf(ny);}
    float angle=fmodf(next.zero_deg,360.0f);if(angle<0.0f)angle+=360.0f;
    next.geometry.origin_q16[0]+=(int32_t)roundf((next.lidar_x_mm-(float)nx)*65536.0f);
    next.geometry.origin_q16[1]+=(int32_t)roundf((next.lidar_y_mm-(float)ny)*65536.0f);
    next.geometry.zero_tenths=(uint16_t)roundf(angle*10.0f);
    next.lidar_x_mm=(float)next.geometry.origin_q16[0]/65536.0f;
    next.lidar_y_mm=(float)next.geometry.origin_q16[1]/65536.0f;
    if(!upper){next.lidar_x_mm=roundf(next.lidar_x_mm);next.lidar_y_mm=roundf(next.lidar_y_mm);}
    next.zero_deg=(float)next.geometry.zero_tenths*0.1f;
  }
  *frozen=next;
}

bool RadarMap_Project(const RadarMap_Params_t *p,const RadarSample_t *s,float *x,float *y)
{
  float angle,px,py;
  if(p==NULL || s==NULL || x==NULL || y==NULL ||
     !isfinite(p->lidar_x_mm) || !isfinite(p->lidar_y_mm) || !isfinite(p->zero_deg)) return false;
  if(p->situation){
    RadarMap_Params_t frozen;RadarGeometry g;int32_t ix,iy;
    /* Keep the source's fractional translation; nominal edits move the origin
     * relative to its rounded display position, without applying it twice. */
    RadarMap_FreezeParams(p,&frozen);g=frozen.geometry;
    RadarGeometry_Project(&g,s->angle_tenths,s->distance_mm,&ix,&iy);
    *x=(float)ix;*y=(float)iy;return true;
  }
  angle=(fmodf(p->zero_deg,360.0f)-(float)(s->angle_tenths%3600U)*0.1f)*RADAR_DEG_TO_RAD;
  px=p->lidar_x_mm+(float)s->distance_mm*cosf(angle);
  py=p->lidar_y_mm+(float)s->distance_mm*sinf(angle);
  if(!isfinite(px) || !isfinite(py)) return false;
  *x=px;*y=py;return true;
}

void RadarGrid_Default(RadarGrid_t *grid)
{
  if(grid!=NULL) *grid=DefaultGrid;
}

bool RadarGrid_Valid(const RadarGrid_t *grid)
{
  unsigned i;
  if(grid==NULL) return false;
  /* Current inset grid and the previous full-field grid used by old snapshots. */
  if(!((grid->x_mm[0]==0 && grid->x_mm[5]==2400 &&
        grid->y_mm[0]==0 && grid->y_mm[5]==2400) ||
       (grid->x_mm[0]==150 && grid->x_mm[5]==2250 &&
        grid->y_mm[0]==150 && grid->y_mm[5]==2250))) return false;
  for(i=0U;i<5U;i++)
    if(grid->x_mm[i+1U]-grid->x_mm[i]<50 || grid->y_mm[i+1U]-grid->y_mm[i]<50) return false;
  return true;
}

static int axis_cell(const int16_t lines[RADAR_GRID_BOUNDARY_COUNT],float value,bool endpoint)
{
  unsigned i;
  if(!isfinite(value) || value<(float)lines[0] || value>(float)lines[5]) return -1;
  if(value==(float)lines[5]) return endpoint?4:-1;
  for(i=0U;i<5U;i++) if(value<(float)lines[i+1U]) return (int)i;
  return -1;
}

int8_t RadarGrid_FindCell(const RadarGrid_t *grid,float x,float y,bool endpoint)
{
  int c,r;
  if(!RadarGrid_Valid(grid)) return -1;
  c=axis_cell(grid->x_mm,x,endpoint);r=axis_cell(grid->y_mm,y,endpoint);
  return (int8_t)(c<0 || r<0?-1:r*5+c);
}

static int find_cell(float x,float y,bool endpoint)
{
  return (int)RadarGrid_FindCell(&DefaultGrid,x,y,endpoint);
}

bool RadarMap_SampleAllowed(const RadarMap_Params_t *p,const RadarSample_t *s)
{
  uint16_t angle,minimum,maximum;
  bool allowed;
  if(!p || !s || !s->distance_mm)return false;
  angle=(uint16_t)(s->angle_tenths%3600U);
  minimum=(uint16_t)(p->angle_min_tenths%3600U);
  maximum=(uint16_t)(p->angle_max_tenths%3600U);
  if(p->angle_min_tenths==0U && p->angle_max_tenths==3600U)allowed=true;
  else allowed=p->angle_min_tenths<=p->angle_max_tenths?
    angle>=minimum && angle<=maximum:angle>=minimum || angle<=maximum;
  return allowed && s->distance_mm>=p->distance_min_mm && s->distance_mm<=p->distance_max_mm &&
    s->energy>=p->energy_min && s->energy<=p->energy_max;
}

static uint16_t saturated_add(uint16_t a,uint16_t b)
{
  uint32_t sum=(uint32_t)a+b;
  return (uint16_t)(sum>0xFFFFU?0xFFFFU:sum);
}

static bool boundary_endpoints_join(float x,float y,const RadarMap_BoundaryRun_t *run)
{
  float dx=x-run->last_x_mm,dy=y-run->last_y_mm;
  return dx*dx+dy*dy<=RADAR_BOUNDARY_GAP_MM*RADAR_BOUNDARY_GAP_MM;
}

static void confirm_boundary_run(RadarMap_t *m,const RadarMap_BoundaryRun_t *run)
{
  if(run->hits<RADAR_BOUNDARY_MIN_POINTS || !run->outside_hits ||
     run->tangent_max_mm-run->tangent_min_mm<RADAR_BOUNDARY_SPAN_MM)return;
  for(unsigned i=0U;i<RADAR_MAP_CELL_COUNT;i++)
    m->boundary_counts[i]=saturated_add(m->boundary_counts[i],run->counts[i]);
}

static void close_boundary_run(RadarMap_t *m)
{
  if(!m->boundary_pending.hits)return;
  if(m->boundary_pending_is_first)m->boundary_first=m->boundary_pending;
  else confirm_boundary_run(m,&m->boundary_pending);
  memset(&m->boundary_pending,0,sizeof m->boundary_pending);
  m->boundary_pending_is_first=false;
}

/* Only a contiguous long return with outward evidence is a field edge.
 * The measured wall varies by up to 28 mm in actual circles. Its 40 mm
 * tolerance is not a blind perimeter exclusion: detached hits remain obstacles. */
static void add_boundary_evidence(RadarMap_t *m,float x,float y,int cell)
{
  const float normal[4]={x,2400.0f-x,y,2400.0f-y};
  RadarMap_BoundaryRun_t *run=&m->boundary_pending;
  unsigned edge=0U,i;
  float distance=fabsf(normal[0]),tangent;
  for(i=1U;i<4U;i++)if(fabsf(normal[i])<distance) {
    edge=i;distance=fabsf(normal[i]);
  }
  if(distance>RADAR_BOUNDARY_TOLERANCE_MM){close_boundary_run(m);return;}
  if(run->hits && (run->edge!=edge || !boundary_endpoints_join(x,y,run)))close_boundary_run(m);
  tangent=edge<2U?y:x;
  if(!run->hits) {
    run->edge=(uint8_t)edge;
    run->first_x_mm=x;run->first_y_mm=y;
    run->tangent_min_mm=run->tangent_max_mm=tangent;
    m->boundary_pending_is_first=!m->accepted_points && !m->outside_points;
  } else {
    if(tangent<run->tangent_min_mm)run->tangent_min_mm=tangent;
    if(tangent>run->tangent_max_mm)run->tangent_max_mm=tangent;
  }
  run->last_x_mm=x;run->last_y_mm=y;
  if(run->hits<0xFFFFU)++run->hits;
  if(normal[edge]<0.0f && run->outside_hits<0xFFFFU)++run->outside_hits;
  if(cell>=0 && run->counts[cell]<0xFFFFU)++run->counts[cell];
}

static void finish_boundary_evidence(RadarMap_t *m)
{
  RadarMap_BoundaryRun_t *first=&m->boundary_first,*last=&m->boundary_pending;
  if(first->hits && last->hits && first->edge==last->edge &&
     boundary_endpoints_join(first->first_x_mm,first->first_y_mm,last)) {
    for(unsigned i=0U;i<RADAR_MAP_CELL_COUNT;i++)first->counts[i]=saturated_add(first->counts[i],last->counts[i]);
    first->hits=saturated_add(first->hits,last->hits);
    first->outside_hits=saturated_add(first->outside_hits,last->outside_hits);
    if(last->tangent_min_mm<first->tangent_min_mm)first->tangent_min_mm=last->tangent_min_mm;
    if(last->tangent_max_mm>first->tangent_max_mm)first->tangent_max_mm=last->tangent_max_mm;
  } else confirm_boundary_run(m,last);
  confirm_boundary_run(m,first);
  memset(first,0,sizeof *first);memset(last,0,sizeof *last);
  m->boundary_pending_is_first=false;
}

bool RadarMap_Add(RadarMap_t *m,const RadarMap_Params_t *p,const RadarSample_t *s)
{
  float x,y;
  int cell;
  if(m==NULL || s==NULL || !RadarMap_ParamsValid(p) || !RadarMap_SampleAllowed(p,s) ||
     !RadarMap_Project(p,s,&x,&y)) return false;
  /* Half-open reference boundaries must rotate with the points, too.
   * F1 (row,column) becomes vehicle (column,4-row), not a mirrored grid. */
  m->grid_version=p->grid_version;
  if(p->grid_version>=4U)cell=RadarGrid_FindCell(&DefaultGrid,x,y,false);
  else {cell=RadarGrid_FindCell(&DefaultGrid,y,2400.0f-x,false);
    if(cell>=0)cell=(cell%5)*5+4-cell/5;}
  m->cloud_built=false;m->evidence_mask=0U;
  add_boundary_evidence(m,x,y,cell);
  if(cell<0) {
    if(m->outside_points<0xFFFFU)++m->outside_points;
    return false;
  }
  if(m->counts[cell]<0xFFFFU) ++m->counts[cell];
  if(m->accepted_points<0xFFFFU) ++m->accepted_points;
  return true;
}

void RadarMap_BuildCloud(RadarMap_t *m,const RadarMap_Params_t *p,
                         const RadarSample_t *samples,uint16_t count)
{
  if(m==NULL)return;
  RadarMap_Reset(m);
  if(!RadarMap_ParamsValid(p) || (count && samples==NULL))return;
  /* Use the same per-cell point threshold as the verified F1 host profile.
   * Preserve all raw counts, including the known-field-edge evidence. */
  for(uint16_t i=0U;i<count;i++)RadarMap_Add(m,p,&samples[i]);
  /* The circle is complete: resolve its cache seam before publishing the raw
   * map. Keep pending run state out of the frozen result; Finish remains
   * idempotent and raw counts/evidence are not changed here. */
  finish_boundary_evidence(m);
  m->cloud_built=true;
  RadarMap_FinishRaw(m,p);
}

void RadarMap_BuildStable(RadarMap_t *m,const RadarMap_Params_t *p,
                          const RadarStablePoint *points,uint16_t count)
{
  if(!m || !RadarMap_ParamsValid(p) || (count && !points))return;
  RadarMap_Reset(m);m->grid_version=p->grid_version;
  for(uint16_t i=0;i<count;i++){
    int cell=RadarGrid_FindCell(&DefaultGrid,points[i].x_mm,points[i].y_mm,false);
    if(cell<0){m->outside_points++;continue;}
    m->counts[cell]++;m->accepted_points++;
  }
  m->cloud_built=true;RadarMap_FinishRaw(m,p);
}

void RadarMap_FinishRaw(RadarMap_t *m,const RadarMap_Params_t *p)
{
  unsigned i;
  if(m==NULL || !RadarMap_ParamsValid(p)) return;
  m->grid_version=p->grid_version;m->mask=RADAR_MAP_FIXED_MASK;
  m->evidence_mask=0U;
  m->background_points=0U;
  for(i=0U;i<RADAR_MAP_CELL_COUNT;i++)
    if(m->counts[i]>=p->threshold)m->evidence_mask|=1UL<<i;
  m->mask|=m->evidence_mask;
}

void RadarMap_Finish(RadarMap_t *m,const RadarMap_Params_t *p)
{
  uint32_t clear_mask=RADAR_MAP_TASK_CLEAR_MASK;
  uint32_t background=0U;
  unsigned i;
  int sensor_cell;
  if(m==NULL || !RadarMap_ParamsValid(p)) return;
  RadarMap_FinishRaw(m,p);
  finish_boundary_evidence(m);
  m->mask=RADAR_MAP_FIXED_MASK;
  for(i=0U;i<RADAR_MAP_CELL_COUNT;i++) {
    uint32_t boundary_hits=m->boundary_counts[i];
    if(boundary_hits>m->counts[i])boundary_hits=m->counts[i];
    background+=boundary_hits;
    if((uint32_t)m->counts[i]-boundary_hits>=p->threshold)m->mask|=1UL<<i;
  }
  m->background_points=(uint16_t)(background>0xFFFFU?0xFFFFU:background);
  /* Rotate F1 task protection into the vehicle frame, retaining counts and
   * raw evidence. Only the installed corner is protected, not both starts. */
  sensor_cell=p->grid_version>=4U?(p->start_id==RADAR_START_UPPER?24:4):
    RadarGrid_FindCell(&DefaultGrid,p->lidar_x_mm,p->lidar_y_mm,true);
  if(sensor_cell==4 || sensor_cell==24)clear_mask|=1UL<<(unsigned)sensor_cell;
  m->mask=(m->mask&~clear_mask)|RADAR_MAP_FIXED_MASK;
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

bool RadarGrid_CellCenter(const RadarGrid_t *grid,uint8_t cell,int16_t *x,int16_t *y)
{
  unsigned r=cell/5U,c=cell%5U;
  if(!RadarGrid_Valid(grid) || cell>=RADAR_MAP_CELL_COUNT || x==NULL || y==NULL) return false;
  *x=(int16_t)((grid->x_mm[c]+grid->x_mm[c+1U])/2);
  *y=(int16_t)((grid->y_mm[r]+grid->y_mm[r+1U])/2);
  return true;
}

static void cell_center(uint8_t cell,int16_t *x,int16_t *y)
{
  (void)RadarGrid_CellCenter(&WaypointGrid,cell,x,y);
}

static uint32_t cell_distance(const RadarGrid_t *grid,uint8_t a,uint8_t b)
{
  int16_t ax,ay,bx,by;
  int dx,dy;
  (void)RadarGrid_CellCenter(grid,a,&ax,&ay);(void)RadarGrid_CellCenter(grid,b,&bx,&by);
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
bool RadarGrid_Path(const RadarGrid_t *grid,uint32_t mask,uint8_t start,uint8_t goal,
                    uint8_t path[RADAR_MAP_CELL_COUNT],uint8_t *count)
{
  uint32_t cost[25];
  uint8_t parent[25],open[25]={0},closed[25]={0},order[25],next_order=0U;
  uint8_t current,best,i,n;
  unsigned dir;
  if(count==NULL || path==NULL) return false;
  *count=0U;
  if(!RadarGrid_Valid(grid) || start>=RADAR_MAP_CELL_COUNT || goal>=RADAR_MAP_CELL_COUNT) return false;
  mask&=RADAR_MAP_ALL_CELLS;
  if(mask&((1UL<<start)|(1UL<<goal))) return false;
  for(i=0U;i<25U;i++){cost[i]=RADAR_COST_MAX;parent[i]=RADAR_CELL_NONE;order[i]=RADAR_CELL_NONE;}
  cost[start]=0U;open[start]=1U;order[start]=next_order++;
  for(;;) {
    uint32_t best_f=RADAR_COST_MAX,best_h=RADAR_COST_MAX;
    best=RADAR_CELL_NONE;
    for(i=0U;i<25U;i++) if(open[i]) {
      uint32_t h=cell_distance(grid,i,goal),f=cost[i]+h;
      if(best==RADAR_CELL_NONE || f<best_f || (f==best_f && h<best_h) ||
         (f==best_f && h==best_h && order[i]<order[best])) {best=i;best_f=f;best_h=h;}
    }
    if(best==RADAR_CELL_NONE) return false;
    current=best;
    if(current==goal) break;
    open[current]=0U;closed[current]=1U;
    for(dir=0U;dir<4U;dir++) if(cell_neighbor(current,dir,&n) && !closed[n] && !(mask&(1UL<<n))) {
      uint32_t tentative=cost[current]+cell_distance(grid,current,n);
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

static bool cell_path(uint8_t version,uint32_t mask,uint8_t start,uint8_t goal,uint8_t path[25],uint8_t *count)
{
  uint32_t reference_mask=0U;
  unsigned cell;
  if(version>=4U)return RadarGrid_Path(&WaypointGrid,mask,start,goal,path,count);
  /* Inverse T: formal (row,column) -> reference (4-column,row).
   * Search in F1's frame so equal-cost ties rotate with N/E/S/W, too. */
  for(cell=0U;cell<25U;cell++)if(mask&(1UL<<cell))
    reference_mask|=1UL<<((4U-cell%5U)*5U+cell/5U);
  start=(uint8_t)((4U-start%5U)*5U+start/5U);
  goal=(uint8_t)((4U-goal%5U)*5U+goal/5U);
  if(!RadarGrid_Path(&WaypointGrid,reference_mask,start,goal,path,count))return false;
  for(cell=0U;cell<*count;cell++)path[cell]=(uint8_t)((path[cell]%5U)*5U+4U-path[cell]/5U);
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

static bool segment_clear(uint32_t mask,int16_t sx,int16_t sy,int16_t ex,int16_t ey);

/** @brief 将实际线段判障后加入路线；同点由append_point合并。
 * @param p 当前已包含至少一个点的路线。
 * @param mask 正式判障网格低25位占用掩码，包含人工障碍。
 * @param x,y 下一个实际世界坐标，mm。
 * @param station,visit 业务工位及访问编号；中转为0。
 * @return 线段不碰占用矩形且路点容量足够时为true。
 */
static bool append_checked_point(RadarPlan_t *p,uint32_t mask,int16_t x,int16_t y,
                                 uint8_t station,uint8_t visit)
{
  const RadarPlan_Point_t *last=&p->points[p->count-1U];
  return segment_clear(mask,last->x_mm,last->y_mm,x,y) && append_point(p,x,y,station,visit);
}

static bool append_leg(const RadarMap_t *m,int16_t sx,int16_t sy,int16_t ex,int16_t ey,
                       uint8_t station,uint8_t visit,RadarPlan_t *p)
{
  int start=find_cell((float)sx,(float)sy,true),goal=find_cell((float)ex,(float)ey,true);
  uint32_t mask=(m->mask|RADAR_MAP_FIXED_MASK)&RADAR_MAP_ALL_CELLS;
  uint8_t path[25],count,i;
  int16_t x,y;
  if(start<0 || goal<0 || !cell_path(m->grid_version,mask,(uint8_t)start,(uint8_t)goal,path,&count)) return false;
  if(!append_point(p,sx,sy,0U,0U)) return false;
  if(start==goal)
    return append_checked_point(p,mask,ex,sy,0U,0U) &&
           append_checked_point(p,mask,ex,ey,station,visit);
  cell_center(path[0],&x,&y);
  /* Only endpoint-cell connectors adapt the F1 grid chain to real stations.
   * Do not select another cross-grid shortcut merely because it is clear. */
  if(path[1]/5U==path[0]/5U) {
    if(!append_checked_point(p,mask,sx,y,0U,0U))return false;
  } else if(!append_checked_point(p,mask,x,sy,0U,0U))return false;
  for(i=0U;i<count;i++) {
    cell_center(path[i],&x,&y);
    if(!append_checked_point(p,mask,x,y,0U,0U)) return false;
  }
  if(path[count-1U]/5U==path[count-2U]/5U)
    return append_checked_point(p,mask,ex,y,0U,0U) &&
           append_checked_point(p,mask,ex,ey,station,visit);
  return append_checked_point(p,mask,x,ey,0U,0U) &&
         append_checked_point(p,mask,ex,ey,station,visit);
}

/* Closed cell rectangles include their edges and corners. The shortcut is only
 * accepted when the complete segment stays outside every occupied rectangle. */
static bool segment_clear(uint32_t mask,int16_t sx,int16_t sy,int16_t ex,int16_t ey)
{
  const float origin[2]={(float)sx,(float)sy},delta[2]={(float)(ex-sx),(float)(ey-sy)};
  unsigned cell,axis;
  for(cell=0U;cell<RADAR_MAP_CELL_COUNT;cell++) if(mask&(1UL<<cell)) {
    const unsigned index[2]={cell%5U,cell/5U};
    float enter=0.0f,leave=1.0f;
    bool intersects=true;
    for(axis=0U;axis<2U;axis++) {
      const int16_t *lines=axis==0U?DefaultGrid.x_mm:DefaultGrid.y_mm;
      float low=(float)lines[index[axis]],high=(float)lines[index[axis]+1U];
      if(delta[axis]==0.0f) {
        if(origin[axis]<low || origin[axis]>high){intersects=false;break;}
      } else {
        float a=(low-origin[axis])/delta[axis],b=(high-origin[axis])/delta[axis],swap;
        if(a>b){swap=a;a=b;b=swap;}
        if(a>enter)enter=a;
        if(b<leave)leave=b;
        if(enter>leave){intersects=false;break;}
      }
    }
    if(intersects)return false;
  }
  return true;
}

bool RadarPlan_BuildFromStart(const RadarMap_t *m,RadarStart_Id_t start_id,RadarPlan_t *p)
{
  unsigned leg;
  int16_t start_y=start_id==RADAR_START_UPPER?2250:150;
  int16_t staging_y=start_id==RADAR_START_UPPER?2100:300;
  if(p==NULL) return false;
  memset(p,0,sizeof(*p));
  if(m==NULL || (start_id!=RADAR_START_UPPER && start_id!=RADAR_START_LOWER)) return false;
  p->start_id=(uint8_t)start_id;
  if(!append_point(p,2250,start_y,1U,0U)) return false;
  for(leg=0U;leg+1U<RADAR_PLAN_MISSION_VISITS;leg++) {
    uint8_t from=MissionOrder[leg],to=MissionOrder[leg+1U];
    int16_t from_x=from==1U?2250:Stations[from-1U][0];
    int16_t from_y=from==1U?start_y:Stations[from-1U][1];
    bool ok;
    if(leg==0U && segment_clear((m->mask|RADAR_MAP_FIXED_MASK)&RADAR_MAP_ALL_CELLS,
                               2250,start_y,2100,staging_y) &&
       segment_clear((m->mask|RADAR_MAP_FIXED_MASK)&RADAR_MAP_ALL_CELLS,
                     2100,staging_y,2100,1200)) {
      /* Publish the same short lane-entry move used by the motion profile;
       * do not display or collision-check an abandoned long diagonal. */
      ok=append_point(p,2100,staging_y,0U,0U) && append_point(p,2100,1200,5U,1U);
    } else if(leg+2U==RADAR_PLAN_MISSION_VISITS) {
      ok=append_leg(m,from_x,from_y,2100,staging_y,0U,0U,p) &&
         append_checked_point(p,(m->mask|RADAR_MAP_FIXED_MASK)&RADAR_MAP_ALL_CELLS,
                              2250,start_y,1U,8U);
    } else {
      ok=append_leg(m,from_x,from_y,
                    Stations[to-1U][0],Stations[to-1U][1],to,(uint8_t)(leg+1U),p);
    }
    if(!ok) {
      p->count=0U;p->failed_leg=(uint8_t)(leg+1U);return false;
    }
  }
  p->valid=true;return true;
}

bool RadarPlan_Build(const RadarMap_t *m,RadarPlan_t *p)
{
  return RadarPlan_BuildFromStart(m,RADAR_START_LOWER,p);
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
