#include "grab_tracking.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static GrabTracking_t track;
static GrabTracking_Config_t cfg;
static GrabTracking_Input_t input;
static GrabTracking_Output_t output;

static void setup(void)
{
  GrabTracking_Defaults(&cfg);
  assert(cfg.control_ms==20 && cfg.batch_ms==0);
  GrabTracking_Init(&track);
  memset(&input,0,sizeof(input));
  input.j[0][0]=2; input.j[1][1]=input.j[1][2]=2;
  input.x_mm=50; input.x_min_mm=0; input.x_max_mm=100;
  input.valid=true;
  GrabTracking_Reference(&track,0);
  GrabTracking_Begin(&track);
}

static void sample(uint32_t now,float ex,float ey,float forward,float left,float extension)
{
  input.now_ms=input.sample_ms=now;
  input.sequence++; input.error_px[0]=ex; input.error_px[1]=ey;
  input.motion_mm[0]=forward; input.motion_mm[1]=left; input.motion_mm[2]=extension;
  input.x_mm=50+extension;
  GrabTracking_Update(&track,&cfg,&input,&output);
}

static void own_motion_and_duplicates(void)
{
  unsigned i,count;
  setup(); cfg.latency_ms=0;
  for(i=1;i<=12;i++) sample(i*20,2*i,4*i,i,i,i);
  assert(fabsf(output.target_px_s[0])<.1f && fabsf(output.target_px_s[1])<.1f);
  count=track.samples;
  GrabTracking_Update(&track,&cfg,&input,&output);
  assert(track.samples==count);
  input.sequence++; /* USB batch: new sequence, same receive timestamp. */
  GrabTracking_Update(&track,&cfg,&input,&output);
  assert(track.samples==count);
}

static void zero_error_feedforward_and_stop(void)
{
  unsigned i;
  setup(); cfg.latency_ms=0;
  for(i=1;i<=20;i++) sample(i*20,0,0,-(float)i,0,0);
  assert(output.target_px_s[0]>95 && output.velocity_mm_s[0]<-40);
  for(i=21;i<=60;i++) sample(i*20,0,0,-20,0,0);
  assert(fabsf(output.target_px_s[0])<1 && fabsf(output.velocity_mm_s[0])<1);
}

static void reallocation_and_budget(void)
{
  unsigned i;
  setup(); cfg.latency_ms=0; input.x_mm=100;
  input.x_min_mm=0; input.x_max_mm=100;
  for(i=1;i<=12;i++)
  {
    input.now_ms=input.sample_ms=i*20; input.sequence++;
    input.error_px[1]=-(float)i*2; input.motion_mm[2]=50;
    GrabTracking_Update(&track,&cfg,&input,&output);
  }
  assert(output.velocity_mm_s[2]<=0 && output.velocity_mm_s[1]>30);
  input.motion_mm[1]=50;
  input.now_ms+=50; input.sample_ms=input.now_ms; input.sequence++;
  GrabTracking_Update(&track,&cfg,&input,&output);
  assert(output.body_room_mm<=.01f && output.velocity_mm_s[1]<=0);
  GrabTracking_Begin(&track); /* Restart does not buy another 50 mm. */
  assert(track.reference_left_mm==0);
  GrabTracking_Update(&track,&cfg,&input,&output);
  assert(output.body_room_mm<=.01f);
}

static void gap(void)
{
  unsigned i;
  setup(); cfg.latency_ms=0;
  for(i=1;i<=20;i++) sample(i*20,0,0,-(float)i,0,0);
  input.valid=false; input.now_ms+=100;
  GrabTracking_Update(&track,&cfg,&input,&output);
  assert(output.predicting && !output.visual_pause && output.velocity_mm_s[0]<-1);
  input.now_ms+=200;
  GrabTracking_Update(&track,&cfg,&input,&output);
  assert(output.visual_pause && output.velocity_mm_s[0]==0);
  input.valid=true; sample(input.now_ms+20,0,0,-20,0,0);
  assert(!output.visual_pause);
}

static void delayed_noise_and_rank(void)
{
  setup(); cfg.latency_ms=50;
  /* A moving camera observing a stationary target, 50ms exposure latency.
   * Fitting must remove its motion even with deterministic subpixel noise. */
  for(unsigned i=1;i<=80;i++)
  {
    float now=i*20,exposure=fmaxf(0,now-50);
    sample((uint32_t)now,.02f*exposure+.15f*sinf(i),0,.01f*now,0,0);
  }
  assert(fabsf(output.target_px_s[0])<5);
  memset(input.j,0,sizeof input.j);
  GrabTracking_Update(&track,&cfg,&input,&output); assert(!output.model_valid);
  setup(); cfg.grace_ms=0; sample(20,0,0,0,0,0);
  input.valid=false; GrabTracking_Update(&track,&cfg,&input,&output);
  for(unsigned i=0;i<3;i++)assert(isfinite(output.velocity_mm_s[i]));
}

static void acceleration_prediction(void)
{
  setup(); cfg.latency_ms=50;
  /* At 20 px/s² the curvature across 240 ms is subpixel. Allow the
   * continuous estimator to establish acceleration from a persistent trend,
   * rather than demanding a confident second derivative after four frames. */
  for(unsigned i=1;i<=100;i++) { float t=i*.02f; sample(i*20,10*t*t,0,0,0,0); }
  printf("acceleration fit v=%.3f a=%.3f\n",track.target_px_s[0],track.target_px_s2[0]);fflush(stdout);
  assert(track.target_px_s2[0]>15);
  input.now_ms+=50;
  { float t=.1f,expected=track.last_error_px[0]+track.target_px_s[0]*t+.5f*track.target_px_s2[0]*t*t;
    GrabTracking_Update(&track,&cfg,&input,&output); assert(fabsf(output.predicted_px[0]-expected)<1e-4f); }
}

/* A one-pixel step/bounce must not turn into a motor-speed-sized feedforward.
 * This reproduces the stationary COM14 observation without any camera motion. */
static void quantized_stationary_noise(void)
{
  float peak=0;
  setup();
  for(unsigned i=1;i<=160;i++)
  {
    float ey=(i%23<6 || i%23>18)?1:2;
    sample(1000+i*50,-1,ey,0,0,0);
    peak=fmaxf(peak,fabsf(output.target_px_s[1]));
  }
  fprintf(stderr,"one-pixel static noise: peak feedforward %.3f px/s\n",peak);
  assert(peak<5);
}

/* Noise suppression must not erase a slowly turning material. It is the
 * compensated world trend, not the current pixel-error window, that matters. */
static void slow_quantized_motion(void)
{
  setup();
  for(unsigned i=1;i<=100;i++)
  {
    float t=i*.05f;
    sample(1000+i*50,roundf(5*t),0,0,0,0);
  }
  fprintf(stderr,"slow quantized target: %.3f px/s\n",output.target_px_s[0]);
  assert(output.target_px_s[0]>3 && output.target_px_s[0]<7);
}

static void circular_targets(void)
{
  unsigned rpm,direction,k;
  for(rpm=6;rpm<=10;rpm+=2) for(direction=0;direction<2;direction++)
  {
    float q[3]={0},sum=0,max_error=0,startup_peak=0,dt=.02f;
    unsigned length=(unsigned)(15.f/rpm/dt),measured=0;
    setup(); cfg.latency_ms=0; input.actuating=true;
    for(k=1;k<=length;k++)
    {
      float t=k*dt,w=(direction?1.f:-1.f)*rpm*6.283185307f/60;
      float phase0=direction?-.785398163f:.785398163f;
      float target[2]={100*(sinf(phase0+w*t)-sinf(phase0)),100*(cosf(phase0+w*t)-cosf(phase0))};
      sample(k*20,2*(q[0]-target[0]),2*(q[1]+q[2]-target[1]),q[0],q[1],q[2]);
      assert(!output.window_end);
      assert(hypotf(output.velocity_mm_s[0],output.velocity_mm_s[1])<=cfg.body_speed_mm_s+.01f);
      assert(fabsf(output.velocity_mm_s[2])<=cfg.x_speed_mm_s+.01f);
      { float error=hypotf(input.error_px[0],input.error_px[1]); if(error>startup_peak)startup_peak=error;
        if(k>length*.65f) { measured++; sum+=error*error; if(error>max_error)max_error=error; } }
      for(unsigned axis=0;axis<3;axis++)
      { q[axis]+=output.velocity_mm_s[axis]*dt; input.actual_mm_s[axis]=output.velocity_mm_s[axis]; }
    }
    printf("ideal %u rpm direction %u: final-window RMS %.2f px, peak %.2f px; startup peak %.2f px\n",rpm,direction,sqrtf(sum/measured),max_error,startup_peak);
    fflush(stdout);
    assert(sqrtf(sum/measured)<5);
  }
}

static void replay_stationary_observation(const char *path)
{
  char line[1024]; unsigned rows=0; float peak=0;
  FILE *file=fopen(path,"r"); assert(file);
  setup();
  while(fgets(line,sizeof line,file))
  {
    char *row=strstr(line,"TRACK row "),*data; unsigned index,now,sequence,received;
    if(!row || !(data=strstr(row," data=")))continue;
    assert(sscanf(row,"TRACK row i=%u t=%u seq=%u rx=%u",&index,&now,&sequence,&received)==4);
    assert(sscanf(data," data=%f,%f",&input.error_px[0],&input.error_px[1])==2);
    input.now_ms=now; input.sample_ms=received; input.sequence=sequence;
    GrabTracking_Update(&track,&cfg,&input,&output); rows++;
    peak=fmaxf(peak,hypotf(output.target_px_s[0],output.target_px_s[1]));
  }
  fclose(file);
  fprintf(stderr,"actual stationary log replay: %u rows, peak %.3f px/s\n",rows,peak);
  assert(rows==128 && peak<5);
}

static void visual_stopping_model(GrabTracking_Mode_t mode)
{
  float history[801]={0},position=0,actual=0,peak=0;
  setup();cfg.mode=mode;cfg.body_accel_mm_s2=300;cfg.x_accel_mm_s2=120;
  input.feedback_age_ms=80;input.actuating=true;
  for(unsigned ms=0;ms<=8000;ms+=10)
  {
    unsigned n=ms/10;float target=ms<1500?30.f*ms/1000.f:45;
    float change=output.velocity_mm_s[0]-actual;
    actual+=fmaxf(-3.55f,fminf(3.55f,change));position+=actual*.01f;history[n]=position;
    if(ms%30==0){input.sequence++;input.sample_ms=1000+ms;input.error_px[0]=roundf(2*(history[n>=5?n-5:0]-target));}
    if(ms%20==0){input.now_ms=1000+ms;input.actual_mm_s[0]=actual;input.motion_mm[0]=position;GrabTracking_Update(&track,&cfg,&input,&output);}
    if(ms>=1500)peak=fmaxf(peak,position-45);
    assert(fabsf(output.target_px_s[0])<.001f && fabsf(output.target_px_s2[0])<.001f);
  }
  fprintf(stderr,"visual stop model mode=%u: overshoot=%.3f mm final_error=%.3f px\n",(unsigned)mode,peak,2*(position-45));
  assert(fabsf(2*(position-45))<2);
}

static void visual_low_frame_rate(GrabTracking_Mode_t mode,unsigned frame_ms)
{
  float peak=0,command=0;
  setup();cfg.mode=mode;
  for(unsigned ms=0;ms<=1000;ms+=20)
  {
    if(ms%frame_ms==0)
    {input.sequence++;input.sample_ms=1000+ms;input.error_px[0]=40;}
    input.now_ms=1000+ms;
    GrabTracking_Update(&track,&cfg,&input,&output);
    if(ms%frame_ms==0)command=output.velocity_mm_s[0];
    else assert(fabsf(output.velocity_mm_s[0]-command)<.001f);
    assert(!output.visual_pause);
    peak=fmaxf(peak,fabsf(output.velocity_mm_s[0]));
  }
  assert(track.error_count==1000/frame_ms+1);
  assert(peak>20); /* Valid 5/10 Hz observations must sustain following. */
  input.now_ms=input.sample_ms+frame_ms+40;
  GrabTracking_Update(&track,&cfg,&input,&output);
  assert(!output.visual_pause && fabsf(output.velocity_mm_s[0])<fabsf(command));
  input.now_ms=input.sample_ms+cfg.grace_ms+20;
  GrabTracking_Update(&track,&cfg,&input,&output);
  assert(output.visual_pause && output.velocity_mm_s[0]==0);
  sample(input.now_ms+20,40,0,0,0,0);
  assert(!output.visual_pause && output.velocity_mm_s[0]<0);
}

static void invalid_models(void)
{
  for(unsigned mode=GRAB_TRACK_PREDICT;mode<=GRAB_TRACK_FEEDBACK;mode++)
    for(unsigned invalid=0;invalid<2;invalid++)
    {
      setup();cfg.mode=(GrabTracking_Mode_t)mode;
      if(invalid)memset(input.j,0,sizeof input.j);
      else input.j[0][0]=NAN; /* Existing reference invalidation marker. */
      sample(1000,40,-20,0,0,0);
      assert(!output.model_valid);
      for(unsigned k=0;k<3;k++)assert(isfinite(output.velocity_mm_s[k]) && output.velocity_mm_s[k]==0);
    }
}

int main(int argc,char **argv)
{
  invalid_models();
  for(unsigned mode=GRAB_TRACK_VISUAL;mode<=GRAB_TRACK_FEEDBACK;mode++)
  {visual_low_frame_rate((GrabTracking_Mode_t)mode,100);visual_low_frame_rate((GrabTracking_Mode_t)mode,200);}
  /* Stopped material at the reference must not inherit an old target trend. */
  setup();cfg.mode=GRAB_TRACK_VISUAL;
  for(unsigned n=1;n<=20;n++)sample(n*20,0,0,-(float)n,0,0);
  assert(fabsf(output.velocity_mm_s[0])<.001f);
  assert(fabsf(output.target_px_s[0])<.001f);
  sample(420,40,0,-20,0,0);
  float held=output.velocity_mm_s[0];
  input.now_ms=440;GrabTracking_Update(&track,&cfg,&input,&output);
  assert(fabsf(output.velocity_mm_s[0]-held)<.001f); /* no repeated old-error acceleration */
  input.valid=false;input.now_ms=460;GrabTracking_Update(&track,&cfg,&input,&output);
  assert(fabsf(output.velocity_mm_s[0])<fabsf(held));
  sample(480,40,0,-20,0,0);input.valid=true;
  sample(500,40,0,-20,0,0);assert(output.velocity_mm_s[0]<0);
  setup();cfg.mode=GRAB_TRACK_FEEDBACK;cfg.body_accel_mm_s2=300;cfg.x_accel_mm_s2=120;
  sample(100,0,-80,0,0,0);sample(120,0,-80,0,0,0);
  assert(fabsf(output.velocity_mm_s[2])<=2.401f);
  /* A faster actual approach consumes more braking room than a stopped mechanism. */
  GrabTracking_t saved=track;input.actual_mm_s[2]=60;input.feedback_age_ms=100;
  sample(220,0,-6,0,0,0);float with_motion=output.velocity_mm_s[2];
  track=saved;input.actual_mm_s[2]=0;sample(220,0,-6,0,0,0);
  assert(with_motion<output.velocity_mm_s[2]);
  visual_stopping_model(GRAB_TRACK_VISUAL);visual_stopping_model(GRAB_TRACK_FEEDBACK);
  setup();
  sample(5000,155,26,0,0,0);
  assert(hypotf(output.velocity_mm_s[0],output.velocity_mm_s[1])<.001f);
  assert(fabsf(output.velocity_mm_s[2])<.001f);
  sample(5020,155,26,0,0,0);
  float first[3];memcpy(first,output.velocity_mm_s,sizeof first);
  assert(sqrtf(first[0]*first[0]+first[1]*first[1]+first[2]*first[2])<=6.001f);
  sample(5020,-155,-26,0,0,0);
  for(unsigned k=0;k<3;k++)assert(fabsf(output.velocity_mm_s[k]-first[k])<.001f);
  quantized_stationary_noise(); slow_quantized_motion();
  own_motion_and_duplicates(); zero_error_feedforward_and_stop();
  reallocation_and_budget(); gap(); delayed_noise_and_rank(); acceleration_prediction(); circular_targets();
  if(argc==2)replay_stationary_observation(argv[1]);
  puts("tracking controller: PASS");
  return 0;
}
