#include "chassis_observer.h"
#include <string.h>
#include <math.h>
void ChassisObserver_Init(ChassisObserver_t *o, float x, float y, float yaw)
{
  memset(o, 0, sizeof(*o));
  o->command.x_mm = o->feedback.x_mm = x;
  o->command.y_mm = o->feedback.y_mm = y;
  o->command.yaw_rad = o->feedback.yaw_rad = yaw;
}
void ChassisObserver_Command(ChassisObserver_t *o, const ChassisModel_Geometry_t *g,
                             const float rpm[4], uint32_t now, uint32_t applied_ms, bool valid)
{
  ChassisModel_Velocity_t v;
  uint32_t dt = now - o->command_ms, after = 0;
  if (!valid || !ChassisModel_Forward(g, rpm, &v))
  {
    o->command_valid = false;
    o->command_baseline = false;
    return;
  }
  if (o->command_baseline && dt <= 100U)
  {
    if (applied_ms != o->applied_ms)
    {
      after = now - applied_ms;
      if (after > dt)
      {
        o->command_valid = false;
        goto baseline;
      }
    }
    (void)ChassisModel_Forward(g, o->previous_rpm, &v);
    o->command_valid = ChassisModel_IntegrateMidpoint(&o->command, &v, (dt - after) / 1000.0f);
    if (after)
    {
      (void)ChassisModel_Forward(g, rpm, &v);
      o->command_valid = ChassisModel_IntegrateMidpoint(&o->command, &v, after / 1000.0f);
    }
  }
  else
    o->command_valid = false;
baseline:
  memcpy(o->previous_rpm, rpm, sizeof(o->previous_rpm));
  o->applied_ms = applied_ms;
  o->command_ms = now;
  o->command_baseline = true;
}
void ChassisObserver_FeedbackReset(ChassisObserver_t *o)
{
  o->feedback_valid = o->feedback_baseline = o->aligned_baseline = false;
}
static void yaw_sample(ChassisObserver_History_t *h, uint32_t time, float yaw)
{
  unsigned last=(h->yaw_next+CHASSIS_OBSERVER_YAW_HISTORY-1U)%CHASSIS_OBSERVER_YAW_HISTORY;
  if(h->yaw_count && h->yaw_history[last].time_ms==time) {
    h->yaw_history[last].rad=yaw;
    return;
  }
  h->yaw_history[h->yaw_next].time_ms=time;
  h->yaw_history[h->yaw_next].rad=yaw;
  h->yaw_next=(h->yaw_next+1U)%CHASSIS_OBSERVER_YAW_HISTORY;
  if(h->yaw_count<CHASSIS_OBSERVER_YAW_HISTORY) h->yaw_count++;
}
static bool yaw_at(const ChassisObserver_History_t *h, uint32_t time, float *yaw)
{
  unsigned first=(h->yaw_next+CHASSIS_OBSERVER_YAW_HISTORY-h->yaw_count)%CHASSIS_OBSERVER_YAW_HISTORY;
  for(unsigned n=0;n<h->yaw_count;n++) {
    unsigned i=(first+n)%CHASSIS_OBSERVER_YAW_HISTORY;
    if(time==h->yaw_history[i].time_ms) { *yaw=h->yaw_history[i].rad;return true; }
    if(n && (int32_t)(h->yaw_history[i].time_ms-time)>0) {
      unsigned before=(i+CHASSIS_OBSERVER_YAW_HISTORY-1U)%CHASSIS_OBSERVER_YAW_HISTORY;
      uint32_t dt=h->yaw_history[i].time_ms-h->yaw_history[before].time_ms;
      uint32_t offset=time-h->yaw_history[before].time_ms;
      if(!dt || dt>600U || offset>dt) return false;
      float delta=remainderf(h->yaw_history[i].rad-h->yaw_history[before].rad,2*CHASSIS_MODEL_PI);
      *yaw=remainderf(h->yaw_history[before].rad+delta*(float)offset/dt,2*CHASSIS_MODEL_PI);
      return true;
    }
  }
  return false; /* No extrapolation beyond measured heading history. */
}
static void position_sample(ChassisObserver_History_t *h, unsigned wheel, uint32_t time, int64_t position)
{
  unsigned next=h->position_next[wheel];
  unsigned last=(next+CHASSIS_OBSERVER_POSITION_HISTORY-1U)%CHASSIS_OBSERVER_POSITION_HISTORY;
  if(h->position_count[wheel] && h->position_history[wheel][last].time_ms==time) return;
  h->position_history[wheel][next].time_ms=time;
  h->position_history[wheel][next].position=position;
  h->position_next[wheel]=(next+1U)%CHASSIS_OBSERVER_POSITION_HISTORY;
  if(h->position_count[wheel]<CHASSIS_OBSERVER_POSITION_HISTORY) h->position_count[wheel]++;
}
static bool position_at(const ChassisObserver_History_t *h, unsigned wheel, uint32_t time, double *position)
{
  unsigned first=(h->position_next[wheel]+CHASSIS_OBSERVER_POSITION_HISTORY-h->position_count[wheel])%
      CHASSIS_OBSERVER_POSITION_HISTORY;
  for(unsigned n=0;n<h->position_count[wheel];n++) {
    unsigned i=(first+n)%CHASSIS_OBSERVER_POSITION_HISTORY;
    if(time==h->position_history[wheel][i].time_ms) {
      *position=(double)h->position_history[wheel][i].position;return true;
    }
    if(n && (int32_t)(h->position_history[wheel][i].time_ms-time)>0) {
      unsigned before=(i+CHASSIS_OBSERVER_POSITION_HISTORY-1U)%CHASSIS_OBSERVER_POSITION_HISTORY;
      uint32_t dt=h->position_history[wheel][i].time_ms-h->position_history[wheel][before].time_ms;
      uint32_t offset=time-h->position_history[wheel][before].time_ms;
      if(!dt || dt>600U || offset>dt) return false;
      *position=(double)h->position_history[wheel][before].position+
          (double)(h->position_history[wheel][i].position-h->position_history[wheel][before].position)*offset/dt;
      return true;
    }
  }
  return false;
}
void ChassisObserver_Feedback(ChassisObserver_t *o, const ChassisModel_Geometry_t *g,
                              const ChassisObserver_Feedback_t *f, ChassisObserver_History_t *h)
{
  uint32_t oldest=0,newest=UINT32_MAX,common;
  uint32_t yaw_ms=f->yaw_time_valid?f->yaw_ms:f->now_ms;
  double aligned[4];
  float yaw, rpm[4];
  bool new_group=true, bracket=true;
  ChassisModel_Velocity_t velocity;
  if(!o->feedback_baseline) memset(h,0,sizeof(*h));
  if(!f->units_per_rev || !f->yaw_valid || !isfinite(f->yaw_rad) || f->now_ms-yaw_ms>600U) goto invalid;
  for(unsigned i=0;i<4;i++) {
    uint32_t age=f->now_ms-f->time_ms[i];
    if(!f->valid[i] || age>600U) goto invalid;
    if(age>oldest) oldest=age;
    if(age<newest) newest=age;
    if(f->time_ms[i]==o->previous_ms[i]) new_group=false;
    position_sample(h,i,f->time_ms[i],f->position[i]);
  }
  yaw_sample(h,yaw_ms,f->yaw_rad);
  if(o->feedback_baseline && !new_group) return;
  o->feedback_span_ms=oldest-newest;
  if(o->feedback_span_ms>(f->max_span_ms?f->max_span_ms:250U)) goto invalid;
  common=f->now_ms-oldest;
  if(f->now_ms-yaw_ms>oldest) common=yaw_ms;
  if(o->aligned_baseline && (int32_t)(common-o->feedback_ms)<=0) return;
  for(unsigned i=0;i<4;i++) {
    if(!position_at(h,i,common,&aligned[i])) { bracket=false;break; }
  }
  bracket=bracket && yaw_at(h,common,&yaw);
  if(bracket) {
    if(o->aligned_baseline) {
      uint32_t dt=common-o->feedback_ms;
      if(!dt || dt>600U) goto invalid;
      for(unsigned i=0;i<4;i++)
        rpm[i]=(float)((aligned[i]-o->aligned_position[i])*60000.0/((double)f->units_per_rev*dt));
      if(!ChassisModel_Forward(g,rpm,&velocity)) goto invalid;
      float seconds=dt/1000.0f;
      velocity.omega_rad_s=remainderf(yaw-o->previous_yaw,2*CHASSIS_MODEL_PI)/seconds;
      o->feedback.yaw_rad=o->previous_yaw;
      o->feedback_valid=ChassisModel_IntegrateMidpoint(&o->feedback,&velocity,seconds);
      o->feedback.yaw_rad=yaw;
    } else o->feedback_valid=false;
    memcpy(o->aligned_position,aligned,sizeof(aligned));
    o->feedback_ms=common;o->previous_yaw=yaw;o->aligned_baseline=true;
  } else { o->aligned_baseline=false;o->feedback_valid=false; }
  for(unsigned i=0;i<4;i++) {
    o->previous_position[i]=f->position[i];o->previous_ms[i]=f->time_ms[i];
  }
  o->feedback_baseline=true;
  return;
invalid:
  ChassisObserver_FeedbackReset(o);
}
