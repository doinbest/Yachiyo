#ifndef ARM_COSINE_PROFILE_H
#define ARM_COSINE_PROFILE_H
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* Velocity envelope in mm/s. K is a duration in 2 ms units, not a loop count.
 * The caller keeps a finite absolute endpoint and owns feedback/stop handling. */
typedef struct {
  float peak, floor, speed;
  uint32_t ramp_ms, phase_ms, tick;
  uint16_t rpm;
  bool active, falling;
} ArmCosineProfile;

static void ArmCosine_Start(ArmCosineProfile *p,float distance,float maximum,
                            float minimum,uint32_t k,uint32_t now)
{
  p->ramp_ms=2U*k;p->phase_ms=0;p->tick=now;
  /* Reserve both half-cosine ramps and 250 ms of feedback/dispatch travel. */
  p->floor=fminf(maximum,minimum);
  p->peak=fmaxf(p->floor,fminf(maximum,distance/(p->ramp_ms*.001f+.25f)));
  p->speed=p->floor;p->rpm=0;
  p->active=true;p->falling=false;
}
static float ArmCosine_Step(ArmCosineProfile *p,float remaining,float measured_speed,
                            uint32_t feedback_age,uint32_t now)
{
  uint32_t dt=now-p->tick;
  if(dt>40U)dt=40U; /* A delayed bus must not skip directly to full speed. */
  p->tick=now;
  float travel=fmaxf(p->speed,fabsf(measured_speed));
  float reserve=travel*(feedback_age*.001f+.04f);
  p->phase_ms+=dt;
  if(p->phase_ms>p->ramp_ms)p->phase_ms=p->ramp_ms;
  float blend=.5f-.5f*cosf(3.14159265358979323846f*p->phase_ms/p->ramp_ms);
  float rising=p->floor+(p->peak-p->floor)*blend;
  /* Invert the integral of a half-cosine braking curve:
   * r/(V*T/2) = u - sin(pi*u)/pi, v/V = (1-cos(pi*u))/2.
   * Position feedback determines braking, not elapsed time alone. */
  float ratio=fmaxf(0,remaining-reserve)/(.5f*p->peak*p->ramp_ms*.001f);
  float braking=p->peak;
  if(ratio<1) {
    float lo=0,hi=1;
    for(unsigned n=0;n<12;n++) {
      float u=(lo+hi)*.5f;
      if(u-sinf(3.14159265358979323846f*u)/3.14159265358979323846f<ratio)lo=u;else hi=u;
    }
    braking=fmaxf(p->floor,p->peak*(.5f-.5f*cosf(3.14159265358979323846f*(lo+hi)*.5f)));
  }
  if(braking<rising)p->falling=true;
  float next=fminf(rising,braking);
  p->speed=p->falling?fminf(p->speed,next):next;
  return p->speed;
}
#endif
