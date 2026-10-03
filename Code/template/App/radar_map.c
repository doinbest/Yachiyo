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
  p->lidar_x_mm=2250.0f;p->lidar_y_mm=150.0f;p->zero_deg=180.0f;
  p->distance_min_mm=100U;p->distance_max_mm=40000U;
  p->angle_min_tenths=0U;p->angle_max_tenths=900U;
  p->threshold=2U;p->energy_min=0U;p->energy_max=255U;
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

static bool sample_allowed(const RadarMap_Params_t *p,const RadarSample_t *s)
{
  uint16_t angle;
  bool allowed;
  angle=(uint16_t)(s->angle_tenths%3600U);
  allowed=p->angle_min_tenths<=p->angle_max_tenths?
    angle>=p->angle_min_tenths && angle<=p->angle_max_tenths:
    angle>=p->angle_min_tenths || angle<=p->angle_max_tenths;
  return allowed && s->distance_mm>=p->distance_min_mm && s->distance_mm<=p->distance_max_mm &&
    s->energy>=p->energy_min && s->energy<=p->energy_max;
}

bool RadarMap_Add(RadarMap_t *m,const RadarMap_Params_t *p,const RadarSample_t *s)
{
  float x,y;
  int cell;
  if(m==NULL || s==NULL || !RadarMap_ParamsValid(p) || !sample_allowed(p,s) ||
     !RadarMap_Project(p,s,&x,&y)) return false;
  cell=find_cell(x,y,false);
  if(cell<0) return false;
  m->cloud_built=false;m->evidence_mask=0U;
  if(m->counts[cell]<0xFFFFU) ++m->counts[cell];
  if(m->accepted_points<0xFFFFU) ++m->accepted_points;
  return true;
}

/* 1534点实测：短物直径12/28mm，邻段间隔323..923mm；长背景端部
 * 各10点共线RMS为5.7/3.0mm。80..300mm分段均同结果，采用100mm。
 * 只在真实场外延伸证据下去掉长轮廓，不把所有长物或拟合线当墙。
 */
#define RADAR_CONTOUR_GAP_MM       100.0f
#define RADAR_CONTOUR_SHORT_MM     200.0f
#define RADAR_CONTOUR_LONG_MM      600.0f
#define RADAR_CONTOUR_SIDE_MM      400.0f
#define RADAR_CONTOUR_DROP_MM      150.0f
#define RADAR_CONTOUR_BRIDGE_MM    300.0f
#define RADAR_CONTOUR_LINE_RMS_MM  20.0f
#define RADAR_CONTOUR_LINE_MAX_MM  50.0f
#define RADAR_CONTOUR_LINE_POINTS  10U

typedef struct {
  uint16_t first,last,points,outside,inside;
  uint16_t counts[RADAR_MAP_CELL_COUNT];
  uint32_t cells;
  float first_x,first_y,last_x,last_y;
  float min_x,max_x,min_y,max_y,max_distance;
} RadarContour_t;

typedef struct {
  const RadarSample_t *samples;
  uint16_t count,start;
} RadarCloudView_t;

/* 只映射虚拟顺序，不移动/复制原始缓存；邻域拟合也使用这一顺序。 */
static const RadarSample_t *cloud_sample(const RadarCloudView_t *cloud,uint16_t index)
{
  uint32_t actual=(uint32_t)cloud->start+index;
  if(actual>=cloud->count)actual-=cloud->count;
  return &cloud->samples[actual];
}

static float contour_span(const RadarContour_t *s)
{
  float dx=s->max_x-s->min_x,dy=s->max_y-s->min_y;
  return sqrtf(dx*dx+dy*dy);
}

static void contour_add(RadarContour_t *s,uint16_t index,const RadarSample_t *sample,float x,float y)
{
  int cell=find_cell(x,y,false);
  if(s->points==0U) {
    s->first=index;s->first_x=x;s->first_y=y;
    s->min_x=s->max_x=x;s->min_y=s->max_y=y;
  }
  s->last=index;s->last_x=x;s->last_y=y;++s->points;
  if(x<s->min_x)s->min_x=x;
  if(x>s->max_x)s->max_x=x;
  if(y<s->min_y)s->min_y=y;
  if(y>s->max_y)s->max_y=y;
  if((float)sample->distance_mm>s->max_distance)s->max_distance=(float)sample->distance_mm;
  if(x<0.0f || x>2400.0f || y<0.0f || y>2400.0f)++s->outside;
  if(cell>=0) {++s->inside;++s->counts[cell];s->cells|=1UL<<cell;}
}

/* 游标指向下一段首点；跳过筛选无效点，仍保存场外轮廓连接证据。 */
static bool contour_next(RadarContour_t *s,const RadarMap_Params_t *p,
                         const RadarCloudView_t *cloud,uint16_t *cursor)
{
  uint16_t i=*cursor;
  memset(s,0,sizeof(*s));
  while(i<cloud->count) {
    float x,y,dx,dy;
    const RadarSample_t *point=cloud_sample(cloud,i);
    if(!sample_allowed(p,point) || !RadarMap_Project(p,point,&x,&y)) {++i;continue;}
    dx=x-s->last_x;dy=y-s->last_y;
    if(s->points && dx*dx+dy*dy>RADAR_CONTOUR_GAP_MM*RADAR_CONTOUR_GAP_MM)break;
    contour_add(s,i,point,x,y);++i;
  }
  *cursor=i;return s->points!=0U;
}

static void contour_merge(RadarContour_t *group,const RadarContour_t *s)
{
  group->points+=s->points;group->inside+=s->inside;group->outside+=s->outside;
  group->cells|=s->cells;
  for(unsigned i=0U;i<RADAR_MAP_CELL_COUNT;i++)group->counts[i]+=s->counts[i];
  if(s->min_x<group->min_x)group->min_x=s->min_x;
  if(s->max_x>group->max_x)group->max_x=s->max_x;
  if(s->min_y<group->min_y)group->min_y=s->min_y;
  if(s->max_y>group->max_y)group->max_y=s->max_y;
}

static void contour_publish(RadarMap_t *m,const RadarMap_Params_t *p,const RadarContour_t *s)
{
  m->outside_points+=s->outside;
  if(s->outside>=3U && contour_span(s)>=RADAR_CONTOUR_LONG_MM) {
    m->background_points+=s->points;return;
  }
  m->accepted_points+=s->inside;
  for(unsigned i=0U;i<RADAR_MAP_CELL_COUNT;i++)m->counts[i]+=s->counts[i];
  if(s->points>=p->threshold)m->evidence_mask|=s->cells;
}

typedef struct {
  float x,y,xx,yy,xy;
  uint16_t count;
} RadarLineMoments_t;

/* 局部坐标平移仅用于拟合数值稳定；地图和测距始终保持原值。 */
static bool contour_line(const RadarMap_Params_t *p,const RadarCloudView_t *cloud,
                         const RadarContour_t *a,const RadarContour_t *c)
{
  RadarLineMoments_t moments={0};
  float angle,nx,ny,mx,my,variance;
  for(unsigned pass=0U;pass<2U;pass++) {
    if(pass) {
      mx=moments.x/(float)moments.count;my=moments.y/(float)moments.count;
      float xx=moments.xx-moments.x*mx,yy=moments.yy-moments.y*my;
      float xy=moments.xy-moments.x*my;
      angle=0.5f*atan2f(2.0f*xy,xx-yy);nx=-sinf(angle);ny=cosf(angle);
      variance=0.5f*(xx+yy-sqrtf((xx-yy)*(xx-yy)+4.0f*xy*xy))/(float)moments.count;
      if(variance>RADAR_CONTOUR_LINE_RMS_MM*RADAR_CONTOUR_LINE_RMS_MM)return false;
    }
    for(unsigned side=0U;side<2U;side++) {
      unsigned kept=0U;
      int index=side?(int)c->first:(int)a->last;
      int end=side?(int)c->last:(int)a->first;
      int step=side?1:-1;
      while((side?index<=end:index>=end) && kept<RADAR_CONTOUR_LINE_POINTS) {
        float x,y;
        const RadarSample_t *sample=cloud_sample(cloud,(uint16_t)index);
        if(sample_allowed(p,sample) && RadarMap_Project(p,sample,&x,&y)) {
          x-=a->last_x;y-=a->last_y;++kept;
          if(!pass) {
            moments.x+=x;moments.y+=y;moments.xx+=x*x;moments.yy+=y*y;moments.xy+=x*y;++moments.count;
          } else if(fabsf((x-mx)*nx+(y-my)*ny)>RADAR_CONTOUR_LINE_MAX_MM)return false;
        }
        index+=step;
      }
    }
  }
  return true;
}

static bool contour_bridge(const RadarMap_Params_t *p,const RadarCloudView_t *cloud,
                           const RadarContour_t *a,const RadarContour_t *b,const RadarContour_t *c)
{
  float dx=c->first_x-a->last_x,dy=c->first_y-a->last_y;
  if(contour_span(b)>RADAR_CONTOUR_SHORT_MM || contour_span(a)<RADAR_CONTOUR_SIDE_MM ||
     contour_span(c)<RADAR_CONTOUR_SIDE_MM ||
     (float)cloud_sample(cloud,a->last)->distance_mm-b->max_distance<RADAR_CONTOUR_DROP_MM ||
     (float)cloud_sample(cloud,c->first)->distance_mm-b->max_distance<RADAR_CONTOUR_DROP_MM ||
     dx*dx+dy*dy>RADAR_CONTOUR_BRIDGE_MM*RADAR_CONTOUR_BRIDGE_MM)return false;
  return contour_line(p,cloud,a,c);
}

void RadarMap_BuildCloud(RadarMap_t *m,const RadarMap_Params_t *p,
                         const RadarSample_t *samples,uint16_t count)
{
  RadarContour_t a,b,c,group;
  RadarCloudView_t cloud={samples,count,0U};
  uint16_t cursor=0U;
  bool have_b,have_c;
  if(m==NULL)return;
  RadarMap_Reset(m);
  if(!RadarMap_ParamsValid(p) || (count && samples==NULL))return;
  m->cloud_built=true;
  /* 默认0..90从最小raw角开始；跨0的选域从其min开始，不能在原缓存
   * index0处拆开物体，也不借用选域外点盲拼首尾以满足阈值。
   */
  uint16_t minimum=3600U;
  for(uint16_t i=0U;i<count;i++) {
    uint16_t angle=(uint16_t)(((uint32_t)samples[i].angle_tenths%3600U+3600U-
                              (uint32_t)p->angle_min_tenths%3600U)%3600U);
    if(angle<minimum) {minimum=angle;cloud.start=i;}
  }
  if(!contour_next(&a,p,&cloud,&cursor)) {RadarMap_FinishRaw(m,p);return;}
  group=a;
  have_b=contour_next(&b,p,&cloud,&cursor);
  have_c=have_b && contour_next(&c,p,&cloud,&cursor);
  while(have_b) {
    if(have_c && contour_bridge(p,&cloud,&a,&b,&c)) {
      contour_publish(m,p,&b);contour_merge(&group,&c);a=c;
      have_b=contour_next(&b,p,&cloud,&cursor);
      have_c=have_b && contour_next(&c,p,&cloud,&cursor);
    } else {
      contour_publish(m,p,&group);group=b;a=b;
      have_b=have_c;
      if(have_b) {b=c;have_c=contour_next(&c,p,&cloud,&cursor);}
    }
  }
  contour_publish(m,p,&group);RadarMap_FinishRaw(m,p);
}

void RadarMap_FinishRaw(RadarMap_t *m,const RadarMap_Params_t *p)
{
  unsigned i;
  if(m==NULL || !RadarMap_ParamsValid(p)) return;
  m->mask=RADAR_MAP_FIXED_MASK;
  if(m->cloud_built)m->mask|=m->evidence_mask;
  else for(i=0U;i<RADAR_MAP_CELL_COUNT;i++) if(m->counts[i]>=p->threshold) m->mask|=1UL<<i;
}

void RadarMap_Finish(RadarMap_t *m,const RadarMap_Params_t *p)
{
  if(m==NULL || !RadarMap_ParamsValid(p)) return;
  RadarMap_FinishRaw(m,p);
  m->mask=(m->mask&~RADAR_MAP_REQUIRED_CLEAR_MASK)|RADAR_MAP_FIXED_MASK;
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
      float low=(float)GridLines[index[axis]],high=(float)GridLines[index[axis]+1U];
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
                               2250,start_y,2100,1200)) {
      ok=append_point(p,2100,1200,5U,1U);
    } else if(leg+2U==RADAR_PLAN_MISSION_VISITS) {
      ok=append_leg(m,from_x,from_y,2100,staging_y,0U,0U,p) &&
         append_point(p,2250,start_y,1U,8U);
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
