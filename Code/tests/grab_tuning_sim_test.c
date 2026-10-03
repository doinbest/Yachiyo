/* Compare tuning with production control/state logic and a synthetic plant.
 * The plant is 2 px/mm, follows commands immediately, and never loses vision.
 * These timings are model predictions, not measurements of the vehicle. */
#define main grab_legacy_regression_main
#include "grab_task_test.c"
#undef main

static unsigned simulate_profile(bool tuned, bool coupled, FILE *trace)
{
  float gain, speed, accel, dx=-95, dy=coupled?-60:-1, last_x;
  unsigned i, first_window=0; uint32_t origin;
  reset(); gain=Config[C_LAMBDA]; speed=Config[C_BODY_SPEED]; accel=Config[C_ACCEL];
  configure(true); visual_defaults_restore();
  setting("x_min","0"); setting("x_max","100"); setting("z_observe","0");
  setting("dx_tol","5"); setting("dy_tol","5");
  Config[C_LAMBDA]=tuned?gain:.6f; Config[C_BODY_SPEED]=tuned?speed:20;
  Config[C_ACCEL]=tuned?accel:20;
  Config[C_X_SPEED]=tuned?10:5;
  assert(GrabTask_Start("align"));
  for(i=0;i<180 && status().state!=GRAB_ALIGN;i++) step(true,1,-95,(int)dy);
  assert(status().state==GRAB_ALIGN);
  origin=clock_ms; last_x=physical[0];
  for(i=0;i<600;i++) {
    GrabTask_Status_t s=status();
    if(trace) fprintf(trace,"%s,%s,%lu,%ld,%ld,%.6f,%.6f,%.6f,%s\n",
      coupled?"coupled":"horizontal",
      tuned?"after":"before",(unsigned long)(clock_ms-origin),
      lroundf(dx),lroundf(dy),s.forward_mm_s,s.left_mm_s,physical[0],s.state_name);
    assert(s.state!=GRAB_ERROR && s.state!=GRAB_STOPPING);
    assert(hypotf(s.forward_mm_s,s.left_mm_s)<=Config[C_BODY_SPEED]+.001f);
    assert(dx<5.6f && !closures && !base_moves && s.recovery_used==0);
    if(!first_window && labs(lroundf(dx))<=5 && labs(lroundf(dy))<=5)
      first_window=clock_ms-origin;
    if(s.state==GRAB_HOLD) break;
    dx+=2*s.forward_mm_s*.05f;
    dy+=2*s.left_mm_s*.05f+2*(physical[0]-last_x); last_x=physical[0];
    step(true,1,(int)lroundf(dx),(int)lroundf(dy));
  }
  assert(status().state==GRAB_HOLD && status().stop_confirmed);
  assert(labs(lroundf(dx))<=5 && labs(lroundf(dy))<=5);
  printf("TUNING %s %s first_window_ms=%u hold_ms=%lu max_speed=%.3f gain=%.3f accel=%.3f\n",
    coupled?"coupled":"horizontal",tuned?"after":"before",first_window,(unsigned long)(clock_ms-origin),
    Config[C_BODY_SPEED],Config[C_LAMBDA],Config[C_ACCEL]);
  return clock_ms-origin;
}

int main(int argc,char **argv)
{
  unsigned before,after; FILE *trace=NULL;
  assert(argc<=2);
  if(argc==2) { trace=fopen(argv[1],"w"); assert(trace);
    fputs("scenario,profile,t_ms,dx_px,dy_px,vf_mm_s,vl_mm_s,x_mm,state\n",trace); }
  for(unsigned coupled=0;coupled<2;coupled++) {
    before=simulate_profile(false,coupled,trace); after=simulate_profile(true,coupled,trace);
    assert(after<before*.75f);
  }
  if(trace) fclose(trace);
  assert(after<before*.75f);
  puts("grab_tuning_sim_test: PASS (synthetic continuous vision only)");
  return 0;
}
