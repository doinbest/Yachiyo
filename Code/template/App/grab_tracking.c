#include "grab_tracking.h"
#include <math.h>
#include <string.h>

static float bounded(float x,float low,float high)
{ return x<low?low:(x>high?high:x); }

static void timing_add(GrabTracking_Timing_t *t,uint32_t ms)
{ t->count++;if(ms>t->maximum_ms)t->maximum_ms=ms;t->bins[ms<200?ms:200]++; }
unsigned GrabTracking_Percentile(const GrabTracking_Timing_t *t,unsigned percent)
{
  uint32_t sum=0,goal=(uint32_t)(((uint64_t)t->count*percent+99)/100);
  if(!t->count)return 0;
  for(unsigned i=0;i<=200;i++){sum+=t->bins[i];if(sum>=goal)return i;}return 200;
}
void GrabTracking_Defaults(GrabTracking_Config_t *c)
{
  c->body_speed_mm_s=160; c->x_speed_mm_s=60; c->accel_mm_s2=300;
  c->lambda_s=2.5f; c->x_weight=2.333333f; c->lead_mm=12; c->tolerance_px=5;
  c->control_ms=20; c->batch_ms=0; c->latency_ms=50; c->fit_ms=240;
  c->horizon_ms=120; c->grace_ms=250;
}
void GrabTracking_Init(GrabTracking_t *t) { memset(t,0,sizeof(*t)); }
void GrabTracking_Reference(GrabTracking_t *t,float left)
{ t->reference_left_mm=left; t->reference_valid=true; }
void GrabTracking_Begin(GrabTracking_t *t)
{
  t->frozen=false; t->record_head=t->record_count=0;
  t->error_count=t->latency_count=0; t->stable_pending=false;
  memset(t->error_square,0,sizeof(t->error_square));
  memset(t->error_bins,0,sizeof(t->error_bins));
  memset(t->latency_square,0,sizeof(t->latency_square));
  memset(t->previous_mm_s,0,sizeof(t->previous_mm_s));
  t->visual_updates=t->timer_updates=t->control_sequence=0;
  memset(&t->control_timing,0,sizeof t->control_timing);
  memset(&t->prediction_timing,0,sizeof t->prediction_timing);
  memset(&t->visual_delay,0,sizeof t->visual_delay);
  memset(&t->computation_timing,0,sizeof t->computation_timing);
  t->have_update=false; /* First segment update excludes idle time. */
}

static void push(GrabTracking_Sample_t *s,unsigned *count,uint32_t now,const float value[3])
{
  unsigned index=*count;
  if(index && s[index-1].time_ms==now) index--;
  else if(index==GRAB_TRACKING_SAMPLES)
  { memmove(s,s+1,(GRAB_TRACKING_SAMPLES-1)*sizeof(*s)); index--; }
  s[index].time_ms=now; memcpy(s[index].value,value,3*sizeof(float));
  *count=index+1;
}

/* Interpolate measured camera motion at exposure estimate, not at receive time. */
static void motion_at(const GrabTracking_t *t,uint32_t now,float result[3])
{
  unsigned i,axis;
  memcpy(result,t->motion[0].value,3*sizeof(float));
  if((int32_t)(now-t->motion[0].time_ms)<=0)return;
  for(i=1;i<t->motions;i++)
  {
    if((int32_t)(now-t->motion[i].time_ms)<=0)
    {
      float fraction=(float)(uint32_t)(now-t->motion[i-1].time_ms)/
        (float)(t->motion[i].time_ms-t->motion[i-1].time_ms);
      fraction=bounded(fraction,0,1);
      for(axis=0;axis<3;axis++) result[axis]=t->motion[i-1].value[axis]+
        fraction*(t->motion[i].value[axis]-t->motion[i-1].value[axis]);
      return;
    }
  }
  memcpy(result,t->motion[t->motions-1].value,3*sizeof(float));
}

static bool solve_fit(double m[3][4],unsigned degree,float *speed,float *accel)
{
  unsigned i,r,c,pivot;
  for(c=0;c<degree;c++)
  {
    double scale; pivot=c;
    for(r=c+1;r<degree;r++)if(fabs(m[r][c])>fabs(m[pivot][c]))pivot=r;
    if(fabs(m[pivot][c])<1e-10)return false;
    for(i=c;i<4;i++) { double swap=m[c][i]; m[c][i]=m[pivot][i]; m[pivot][i]=swap; }
    scale=m[c][c]; for(i=c;i<4;i++)m[c][i]/=scale;
    for(r=0;r<degree;r++)if(r!=c)
    { scale=m[r][c]; for(i=c;i<4;i++)m[r][i]-=scale*m[c][i]; }
  }
  *speed=(float)m[1][3]; *accel=degree==3?(float)(2*m[2][3]):0;
  return isfinite(*speed)&&isfinite(*accel);
}

/* One pixel of quantization cannot justify a new large curvature at every
 * frame. Fit the same 240 ms observations, with continuous velocity/curvature
 * priors. Units remain px/s and px/s², separate from motor acceleration.
 * These are soft estimation priors, not target speed/acceleration caps. */
static bool fit(GrabTracking_t *t,unsigned axis,uint32_t window,float delta,float noise_variance,
                float *speed,float *accel)
{
  double m[3][4]={{0}},raw[3][4],moments[5]={0}; unsigned i,r,c,n=0,degree;
  float minimum=1e30f,maximum=-1e30f,curvature_support;
  uint32_t latest=t->samples_data[t->samples-1].time_ms;
  const double velocity_variance=144,acceleration_variance=6400;
  for(i=0;i<t->samples;i++)
  {
    uint32_t age=latest-t->samples_data[i].time_ms; double x,power=1;
    if(age>window)continue;
    minimum=fminf(minimum,t->samples_data[i].value[axis]);
    maximum=fmaxf(maximum,t->samples_data[i].value[axis]);
    x=-(double)age/1000.; n++;
    for(r=0;r<5;r++) { moments[r]+=power; power*=x; }
    power=1;
    for(r=0;r<3;r++) { m[r][3]+=power*t->samples_data[i].value[axis]; power*=x; }
  }
  if(n<2)return false;
  {
    float span=maximum-minimum;
    /* A trend inside one observed quantization step supports a velocity,
     * but cannot justify persistent acceleration. Slow motion is NOT gated. */
    curvature_support=fmaxf(0,(span*span-4*noise_variance)/(span*span+4*noise_variance));
  }
  {
    double spread=moments[2]-moments[1]*moments[1]/n;
    double slope=spread>1e-10?(m[1][3]-moments[1]*m[0][3]/n)/spread:0;
    double variance=spread>1e-10?noise_variance/spread:1e30;
    t->velocity_confidence[axis]=(float)(slope*slope/(slope*slope+9*variance));
  }
  degree=n>=4?3:2;
  for(r=0;r<degree;r++) for(c=0;c<degree;c++)m[r][c]=moments[r+c];
  memcpy(raw,m,sizeof raw);
  (void)solve_fit(raw,degree,&t->raw_velocity_px_s[axis],&t->raw_accel_px_s2[axis]);
  m[1][1]+=noise_variance/velocity_variance;
  m[1][3]+=noise_variance/velocity_variance*
    (t->target_px_s[axis]+curvature_support*t->target_px_s2[axis]*delta);
  if(degree==3)
  {
    /* Polynomial coefficient is acceleration/2. */
    m[2][2]+=4*noise_variance/acceleration_variance;
    m[2][3]+=2*noise_variance/acceleration_variance*t->target_px_s2[axis]*curvature_support;
  }
  if(!solve_fit(m,degree,speed,accel))return false;
  *accel*=curvature_support;
  return true;
}

/* Try every bounded/free combination of the three physical DOFs. A clipped X
 * is re-solved using body motion, rather than slowing every axis with it. */
static bool allocate(const float j[2][3],const float demand[2],const float weight[3],
                     const float low[3],const float high[3],float body_limit,float result[3])
{
  unsigned code,i; float best=1e30f; bool found=false;
  for(code=0;code<27;code++)
  {
    unsigned state=code,choice[3]; float v[3],r[2]={demand[0],demand[1]},a=0,b=0,d=0,det,cost=0;
    for(i=0;i<3;i++)
    {
      choice[i]=state%3; state/=3;
      v[i]=choice[i]==1?low[i]:(choice[i]==2?high[i]:0);
      r[0]-=j[0][i]*v[i]; r[1]-=j[1][i]*v[i];
      if(!choice[i]) { a+=weight[i]*j[0][i]*j[0][i]; b+=weight[i]*j[0][i]*j[1][i]; d+=weight[i]*j[1][i]*j[1][i]; }
    }
    det=a*d-b*b;
    if(det>1e-7f)
    {
      float s0=(d*r[0]-b*r[1])/det,s1=(a*r[1]-b*r[0])/det;
      for(i=0;i<3;i++)if(!choice[i])v[i]=weight[i]*(j[0][i]*s0+j[1][i]*s1);
    }
    else if(a+d>1e-7f)
    {
      for(i=0;i<3;i++)if(!choice[i])v[i]=weight[i]*(j[0][i]*r[0]+j[1][i]*r[1])/(a+d);
    }
    if(hypotf(v[0],v[1])>body_limit+.001f)continue;
    for(i=0;i<3;i++)if(v[i]<low[i]-.001f||v[i]>high[i]+.001f)break;
    if(i!=3)continue;
    r[0]=demand[0]; r[1]=demand[1];
    for(i=0;i<3;i++) { r[0]-=j[0][i]*v[i]; r[1]-=j[1][i]*v[i]; cost+=v[i]*v[i]/weight[i]; }
    if(hypotf(r[0],r[1])>.1f)continue;
    if(cost<best) { best=cost; memcpy(result,v,3*sizeof(float)); found=true; }
  }
  return found;
}

/* Braking room includes one real command opportunity (batch interval). */
static float room_speed(float room,float accel,float delay_s,float limit)
{
  if(room<=0)return 0;
  return fminf(limit,sqrtf(accel*accel*delay_s*delay_s+2*accel*room)-accel*delay_s);
}

static void record(GrabTracking_t *t,const GrabTracking_Input_t *in,const GrabTracking_Output_t *out)
{
  GrabTracking_Record_t *r;
  if(t->frozen)return;
  r=&t->records[t->record_head]; memset(r,0,sizeof(*r));
  r->time_ms=in->now_ms; r->sequence=in->sequence;
  r->sample_ms=in->sample_ms;
  r->feedback_age_ms=(uint16_t)fminf(in->feedback_age_ms,65534);
  r->batch_interval_ms=(uint16_t)fminf(in->batch_interval_ms,65534);
  r->request_age_ms=in->request_ms?(uint16_t)fminf(in->now_ms-in->request_ms,65534):65535;
  r->sent_age_ms=in->sent_ms?(uint16_t)fminf(in->now_ms-in->sent_ms,65534):65535;
  r->flags=(in->valid?1U:0U)|(in->actuating?2U:0U)|(out->stable?4U:0U)|(out->predicting?8U:0U);
  memcpy(r->error_px,in->error_px,sizeof(r->error_px));
  memcpy(r->target_px_s,out->target_px_s,sizeof(r->target_px_s));
  memcpy(r->actual_mm_s,in->actual_mm_s,sizeof(r->actual_mm_s));
  if(in->actuating)memcpy(r->command_mm_s,out->velocity_mm_s,sizeof(r->command_mm_s));
  r->x_mm=in->x_mm; r->forward_mm=in->motion_mm[0]; r->left_mm=in->motion_mm[1]; r->body_room_mm=out->body_room_mm;
  t->record_head=(t->record_head+1)%GRAB_TRACKING_TRACE;
  if(t->record_count<GRAB_TRACKING_TRACE)t->record_count++;
}

void GrabTracking_Update(GrabTracking_t *t,const GrabTracking_Config_t *c,
                         const GrabTracking_Input_t *in,GrabTracking_Output_t *out)
{
  unsigned axis,k; float measured[3],camera[3],dt,age_s,demand[2],correct[2];
  float weight[3]={1,1,c->x_weight},low[3],high[3],scale=1;
  bool new_measurement=in->valid && (!t->have_sample ||
    (in->sequence!=t->last_sequence && in->sample_ms!=t->last_sample_ms));
  if(t->have_update) {
    timing_add(&t->control_timing,in->now_ms-t->last_update_ms);
    if(!new_measurement)timing_add(&t->prediction_timing,in->now_ms-t->last_update_ms);
  }
  if(new_measurement) {
    t->visual_updates++;
    if(in->receive_time_valid)timing_add(&t->visual_delay,in->now_ms-in->received_ms);
  }
  else t->timer_updates++;
  t->control_sequence++;t->last_rx_ms=in->receive_time_valid?in->received_ms:0;t->last_control_ms=in->now_ms;
  uint32_t age;
  memset(out,0,sizeof(*out));
  dt=t->have_update?bounded((in->now_ms-t->last_update_ms)/1000.f,0,.2f):0;
  t->last_update_ms=in->now_ms; t->have_update=true;
  push(t->motion,&t->motions,in->now_ms,in->motion_mm);
  out->body_room_mm=t->reference_valid?fmaxf(0,50-(in->motion_mm[1]-t->reference_left_mm)):0;
  out->x_room_mm=fminf(in->x_max_mm-in->x_mm,in->x_mm-in->x_min_mm);
  out->x_target_mm=in->x_mm;
  {
    float a=0,b=0,d=0;
    out->model_valid=c->x_weight>0;
    for(k=0;k<3;k++)
    {
      if(!isfinite(in->j[0][k]) || !isfinite(in->j[1][k]))out->model_valid=false;
      a+=in->j[0][k]*in->j[0][k]; b+=in->j[0][k]*in->j[1][k]; d+=in->j[1][k]*in->j[1][k];
    }
    out->model_valid=out->model_valid && a*d-b*b>1e-6f;
  }
  if(in->valid && (!t->have_sample || (in->sequence!=t->last_sequence && in->sample_ms!=t->last_sample_ms)))
  {
    float blend; uint32_t delta=t->have_sample?in->sample_ms-t->last_sample_ms:0;
    if(t->have_sample && delta>c->grace_ms) { t->samples=0; memset(t->target_px_s,0,sizeof(t->target_px_s)); memset(t->target_px_s2,0,sizeof(t->target_px_s2)); }
    motion_at(t,in->sample_ms>=c->latency_ms?in->sample_ms-c->latency_ms:0,camera);
    for(axis=0;axis<2;axis++)
    {
      measured[axis]=in->error_px[axis];
      for(k=0;k<3;k++)measured[axis]-=in->j[axis][k]*camera[k];
      t->error_square[axis]+=in->error_px[axis]*in->error_px[axis];
      k=(unsigned)fminf(100,ceilf(fabsf(in->error_px[axis]))); t->error_bins[axis][k]++;
    }
    measured[2]=0;
    memcpy(t->last_compensated_px,measured,sizeof t->last_compensated_px);
    if(t->samples && delta && delta<=c->grace_ms)
    {
      for(k=0;k<3;k++)for(axis=0;axis<2;axis++)
      {
        float ahead=delta/1000.f+k*.05f;
        float residual=measured[axis]-(t->prior_target[axis]+t->prior_velocity[axis]*ahead+.5f*t->prior_accel[axis]*ahead*ahead);
        t->latency_square[k]+=residual*residual;
      }
      t->latency_count++;
    }
    push(t->samples_data,&t->samples,in->sample_ms,measured);
    blend=delta?(float)delta/(100.f+delta):1;
    for(axis=0;axis<2;axis++)
    {
      float velocity,acceleration,noise_variance=.25f;
      if(in->actuating)
      {
        float unmatched=0;
        for(k=0;k<3;k++)
        {
          unmatched+=in->j[axis][k]*(t->previous_mm_s[k]-in->actual_mm_s[k]);
        }
        /* Staggered encoders, motor response and the Pi's temporal filter
         * leave camera-motion uncertainty. Do not fit that residual as a
         * high-confidence material acceleration. This 50 ms uncertainty
         * scales change fit weight, not a target speed cap or motion limit. */
        noise_variance+=unmatched*unmatched*.0025f;
      }
      if(fit(t,axis,c->fit_ms,delta/1000.f,noise_variance,&velocity,&acceleration))
      {
        float prior=t->target_px_s[axis];
        float response=delta?(float)delta/(100.f-85.f*t->velocity_confidence[axis]+delta):1;
        t->target_px_s[axis]=prior+response*(velocity-prior);
        t->target_px_s2[axis]+=blend*(acceleration-t->target_px_s2[axis]);
      }
      t->prior_target[axis]=measured[axis]; t->prior_velocity[axis]=t->target_px_s[axis]; t->prior_accel[axis]=t->target_px_s2[axis];
    }
    t->prior_ms=in->sample_ms;
    memcpy(t->last_error_px,in->error_px,sizeof(t->last_error_px));
    memcpy(t->last_motion_mm,camera,sizeof(camera));
    t->have_sample=true; t->last_sequence=in->sequence; t->last_sample_ms=in->sample_ms; t->error_count++;
  }
  age=t->have_sample?in->now_ms-t->last_sample_ms:UINT32_MAX;
  out->predicting=!in->valid || age>c->control_ms;
  out->visual_pause=age>c->grace_ms;
  if(out->visual_pause || !out->model_valid)
  { memset(t->previous_mm_s,0,sizeof(t->previous_mm_s)); t->stable_pending=false; record(t,in,out); return; }
  age_s=(age+c->latency_ms)/1000.f;
  for(axis=0;axis<2;axis++)
  {
    float decay=out->predicting?(c->grace_ms?bounded(1-(float)age/c->grace_ms,0,1):0):1;
    out->target_px_s[axis]=(t->target_px_s[axis]+t->target_px_s2[axis]*age_s)*decay;
    out->target_px_s2[axis]=t->target_px_s2[axis];
    out->predicted_px[axis]=t->last_error_px[axis]+t->target_px_s[axis]*age_s+.5f*t->target_px_s2[axis]*age_s*age_s;
    for(k=0;k<3;k++)out->predicted_px[axis]+=in->j[axis][k]*(in->motion_mm[k]-t->last_motion_mm[k]);
    /* A missing coordinate contributes no repeated old-error acceleration. */
    correct[axis]=out->predicting?0:-c->lambda_s*out->predicted_px[axis]*
      (fabsf(out->predicted_px[axis])<=c->tolerance_px?.5f:1);
    demand[axis]=-out->target_px_s[axis]+correct[axis];
  }
  for(k=0;k<3;k++) { low[k]=-(k==2?c->x_speed_mm_s:c->body_speed_mm_s); high[k]=-low[k]; }
  high[1]=room_speed(out->body_room_mm,c->accel_mm_s2,fmaxf(c->batch_ms,in->batch_interval_ms)/1000.f,c->body_speed_mm_s);
  high[2]=room_speed(in->x_max_mm-in->x_mm,c->accel_mm_s2,c->horizon_ms/1000.f,c->x_speed_mm_s);
  low[2]=-room_speed(in->x_mm-in->x_min_mm,c->accel_mm_s2,c->horizon_ms/1000.f,c->x_speed_mm_s);
  out->model_valid=allocate(in->j,(float[2]){0,0},weight,low,high,c->body_speed_mm_s,out->velocity_mm_s);
  if(!allocate(in->j,demand,weight,low,high,c->body_speed_mm_s,out->velocity_mm_s))
  {
    /* Only scale when reallocation is exhausted; a speed cap is not a travel edge. */
    float lo=0,hi=1;
    for(k=0;k<14;k++)
    {
      float mid=(lo+hi)*.5f,requested[2]={mid*demand[0],mid*demand[1]},candidate[3];
      if(allocate(in->j,requested,weight,low,high,c->body_speed_mm_s,candidate))
      { lo=mid; memcpy(out->velocity_mm_s,candidate,sizeof(candidate)); } else hi=mid;
    }
    scale=lo;
    out->window_end=scale<.15f && hypotf(demand[0],demand[1])>2 &&
      (out->body_room_mm<.5f || out->x_room_mm<.5f);
  }
  if(out->window_end)memset(out->velocity_mm_s,0,sizeof(out->velocity_mm_s));
  else
  {
    float change[3],length=0;
    for(k=0;k<3;k++) { change[k]=out->velocity_mm_s[k]-t->previous_mm_s[k]; length+=change[k]*change[k]; }
    length=sqrtf(length);
    if(dt>0 && length>c->accel_mm_s2*dt)
      for(k=0;k<3;k++)out->velocity_mm_s[k]=t->previous_mm_s[k]+change[k]*c->accel_mm_s2*dt/length;
    for(k=0;k<3;k++)out->velocity_mm_s[k]=bounded(out->velocity_mm_s[k],low[k],high[k]);
  }
  memcpy(t->previous_mm_s,out->velocity_mm_s,sizeof(t->previous_mm_s));
  {
    float free_low[3]={-c->body_speed_mm_s,-c->body_speed_mm_s,-c->x_speed_mm_s};
    float free_high[3]={c->body_speed_mm_s,c->body_speed_mm_s,c->x_speed_mm_s};
    (void)allocate(in->j,(float[2]){-out->target_px_s[0],-out->target_px_s[1]},weight,free_low,free_high,c->body_speed_mm_s,out->feedforward_mm_s);
    (void)allocate(in->j,correct,weight,free_low,free_high,c->body_speed_mm_s,out->correction_mm_s);
  }
  out->x_target_mm=bounded(in->x_mm+bounded(out->velocity_mm_s[2]*c->horizon_ms/1000.f,-c->lead_mm,c->lead_mm),in->x_min_mm,in->x_max_mm);
  if(!out->predicting && fabsf(in->error_px[0])<=c->tolerance_px && fabsf(in->error_px[1])<=c->tolerance_px)
  { if(!t->stable_pending) { t->stable_pending=true; t->stable_since=in->now_ms; } out->stable=in->now_ms-t->stable_since>=200; }
  else t->stable_pending=false;
  record(t,in,out);
}
const GrabTracking_Record_t *GrabTracking_Record(const GrabTracking_t *t,unsigned index)
{
  if(index>=t->record_count)return 0;
  return &t->records[(t->record_head+GRAB_TRACKING_TRACE-t->record_count+index)%GRAB_TRACKING_TRACE];
}
