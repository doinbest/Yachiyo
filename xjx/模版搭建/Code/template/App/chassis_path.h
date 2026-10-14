/* Fixed continuous junction paths, shared with the host preview. Units mm/deg.
 * No HAL or task state: firmware and preview must use identical geometry. */
#ifndef CHASSIS_PATH_H
#define CHASSIS_PATH_H
#include <math.h>
#include <stdbool.h>
#define CHASSIS_PATH_CAPACITY 26U
#define CHASSIS_PATH_ENTRY_MAX_MM_S 200.0f
typedef struct { float x,y,yaw,s,limit; } ChassisPath_Point;
typedef struct {
  ChassisPath_Point point[CHASSIS_PATH_CAPACITY];
  unsigned count;
  float length,turn_end;
} ChassisPath;
static inline void ChassisPath_Add(ChassisPath *p,float x,float y,float yaw,float limit)
{
  float s=0;
  if(p->count){ChassisPath_Point a=p->point[p->count-1];s=a.s+hypotf(x-a.x,y-a.y);}
  p->point[p->count++]=(ChassisPath_Point){x,y,yaw,s,limit};p->length=s;
}
/* kind 1: lower entrance; 2: raw station from east; 3: raw from south. */
static inline bool ChassisPath_Build(ChassisPath *p,unsigned kind,float sx,float sy,
                                     float tx,float ty,float speed)
{
  if(!p || kind<1 || kind>3 || !isfinite(speed) || speed<=0)return false;
  p->count=0;p->length=p->turn_end=0;
  ChassisPath_Add(p,sx,sy,kind==1?180:90,speed);
  if(kind==1){
    float entry_speed=fminf(speed,CHASSIS_PATH_ENTRY_MAX_MM_S);
    /* Leave the start square without rotating, then join the corridor with
     * a vertical tangent. The old inward bow left only ~7 mm at the block. */
    for(unsigned i=1;i<=8;i++){
      float t=i/8.0f,u=1-t;
      float x=u*u*u*sx+3*u*u*t*2200+3*u*t*t*2125+t*t*t*2125;
      float y=u*u*u*sy+3*u*u*t*200+3*u*t*t*240+t*t*t*350;
      ChassisPath_Add(p,x,y,180,entry_speed);
    }
    /* Spread yaw over 600 mm, staying close to the corridor centre. Ease
     * lateral motion and yaw at both ends; exit parallel to the QR lane. */
    for(unsigned i=1;i<=16;i++){
      float t=i/16.0f,e=t*t*(3-2*t);
      ChassisPath_Add(p,2125-25*e,350+600*t,180-90*e,entry_speed);
    }
    p->turn_end=p->length;
    ChassisPath_Add(p,tx,ty,90,0);
    return true;
  }
  if(kind==2)ChassisPath_Add(p,1300,2050,90,60);
  else ChassisPath_Add(p,1200,1950,90,60);
  for(unsigned i=1;i<=20;i++){
    float t=i/20.0f,b=30*sinf(3.14159265358979323846f*t);
    if(kind==2)ChassisPath_Add(p,1300-100*t,2050-b,90-90*t,60);
    else ChassisPath_Add(p,1200+b,1950+100*t,90-90*t,60);
  }
  p->turn_end=p->length;
  ChassisPath_Add(p,tx,ty,kind==1?90:0,0);
  return true;
}
static inline ChassisPath_Point ChassisPath_Sample(const ChassisPath *p,float s)
{
  if(s<=0)return p->point[0];
  for(unsigned i=1;i<p->count;i++)if(s<=p->point[i].s){
    ChassisPath_Point a=p->point[i-1],b=p->point[i];
    float t=(s-a.s)/(b.s-a.s);
    return (ChassisPath_Point){a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,
      a.yaw+(b.yaw-a.yaw)*t,s,b.limit};
  }
  return p->point[p->count-1];
}
/* Projection only advances through nearby remaining path. Resume retains s,
 * so a stop cannot select a passed branch or replay the entrance. */
static inline float ChassisPath_Project(const ChassisPath *p,float s,float x,float y)
{
  float best=1e30f,result=s;
  for(unsigned i=1;i<p->count;i++){
    ChassisPath_Point a=p->point[i-1],b=p->point[i];
    if(b.s<s || a.s>s+200)continue;
    float dx=b.x-a.x,dy=b.y-a.y,l=b.s-a.s;
    float t=fmaxf(0,fminf(1,((x-a.x)*dx+(y-a.y)*dy)/(l*l)));
    float q=fmaxf(s,a.s+t*l);
    ChassisPath_Point at=ChassisPath_Sample(p,q);
    float e=hypotf(at.x-x,at.y-y);
    if(e<best){best=e;result=q;}
  }
  return result;
}
#endif
