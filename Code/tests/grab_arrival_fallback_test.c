#define main legacy_grab_test_main
#include "grab_task_test.c"
#undef main
static void seed(unsigned i) {
  reset();configure(true);clock_ms=1000;Config[C_ARRIVAL_HOLD_MS]=500;
  Axes[i]=(GrabAxis_t){.mm=-39.974f,.target=-40,.flags=1,
    .position_valid=true,.state_valid=true,.quiet=true,.commanded=true,
    .position_tick=1000,.state_tick=1000,.quiet_since=500,.command_tick=600};
}
static bool sample(unsigned i,unsigned dt) {
  clock_ms+=dt;Axes[i].position_tick=Axes[i].state_tick=clock_ms;
  return arrived(i,clock_ms);
}
int main(void) {
  for(unsigned i=0;i<2;i++) {
    seed(i);assert(!arrived(i,clock_ms));
    clock_ms+=600;assert(!arrived(i,clock_ms)); /* Cache cannot accrue time. */
    assert(sample(i,1));assert(arrived(i,clock_ms));
    assert(strstr(output,"source=stable_feedback"));
    arrival_reset(i);Axes[i].target+=.01f;Axes[i].command_tick=clock_ms;
    assert(!sample(i,300));assert(!sample(i,400));assert(sample(i,100));
    seed(i);assert(!arrived(i,clock_ms));
    Axes[i].mm=-38;assert(!sample(i,600));assert(!sample(i,600));
    seed(i);assert(!arrived(i,clock_ms));
    for(unsigned n=0;n<20;n++){Axes[i].mm+=.04f;assert(!sample(i,100));} /* Slow creep. */
    seed(i);assert(!arrived(i,clock_ms));
    assert(!sample(i,2100));assert(!sample(i,400));assert(sample(i,100));
    seed(i);assert(!arrived(i,clock_ms));
    Axes[i].flags=9;assert(!sample(i,600));Axes[i].flags=1;
    assert(!sample(i,100));assert(sample(i,500));
    seed(i);assert(!arrived(i,clock_ms));stop_begin();
    assert(!sample(i,600));assert(!sample(i,500));
    seed(i);Axes[i].flags=3;assert(!arrived(i,clock_ms)); /* No 9F: use measured fallback. */
    physical[i]=destination[i]=Axes[i].target;arrival_reply_enabled=true;
    assert(!arrived(i,clock_ms)); /* Receive ignores legacy9F; measured hold remains. */
    assert(sample(i,500));
    seed(i);assert(!arrived(i,clock_ms));Axes[i].quiet=false;
    assert(!sample(i,600));Axes[i].quiet=true;Axes[i].quiet_since=clock_ms;
    assert(!sample(i,300));assert(sample(i,500));
  }
  /* Separate axis cannot inherit another axis's elapsed hold. */
  seed(0);Axes[1]=Axes[0];assert(!arrived(0,clock_ms));assert(sample(0,600));
  Axes[1].position_tick=Axes[1].state_tick=clock_ms;assert(!arrived(1,clock_ms));
  puts("grab_arrival_fallback_test: fresh samples, tolerance, drift, fault, stop and axis isolation PASS");
}
