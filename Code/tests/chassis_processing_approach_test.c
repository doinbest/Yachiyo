/* Prepared source only: turn at380, approach350, then station handoff.
 * Production route/profile/planner; motion/feedback use the existing fixture. */
#define main legacy_route_main
#include "chassis_route_test.c"
#undef main
static void confirm_paused(void)
{
 motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;motion.reason="cancelled";
 ChassisRoute_Process();ChassisRoute_Process();
 assert(!strcmp(status().state,"paused") && status().stop_confirmed);
}
static void run(bool planned,bool upper,unsigned pause_at,uint32_t mask)
{
 setup();
 if(upper){loc.origin.y_mm=loc.observer.feedback.y_mm=2250;
  loc.origin.yaw_rad=loc.observer.feedback.yaw_rad=-CHASSIS_MODEL_PI/2;}
 ChassisRoute_TaskOptions_t options={true,1200,100,upper?RADAR_START_UPPER:RADAR_START_LOWER};
 RadarMap_t map;RadarPlan_t plan;RadarMap_Reset(&map);map.mask=mask;
 assert(RadarPlan_BuildFromStart(&map,options.start_id,&plan));
 if(planned && !mask)for(unsigned i=1;i<plan.count;i++)if(plan.points[i].station==2){
  assert(plan.points[i-1].x_mm==1200 && plan.points[i-1].y_mm>=380);
 } /* The standard radar connector must not overshoot350 before the380 turn. */
 assert(ChassisRoute_TaskStartEx(planned?&plan:NULL,1000,&options));
 unsigned visits=0,turns=0,docks=0;
 for(unsigned guard=0;guard<500 && strcmp(status().state,"done");guard++){
  ChassisRoute_Status_t s=status();
  if(!strcmp(s.state,"station")){
   if(s.station==2){
    visits++;assert(s.target_y_mm==350 && loc.observer.feedback.y_mm==350);
    assert(motion.target_map_yaw_deg==180 && s.stop_confirmed);
   }
   assert(ChassisRoute_WorkBegin());
   if(s.station==2)loc.observer.feedback.y_mm=425; /* Mission-owned departure. */
   assert(ChassisRoute_WorkResume());continue;
  }
  if(ChassisMotion_IsBusy()){
   if(s.station==2 && !strcmp(s.phase,"work_turn")){
    turns++;assert(last_turn && motion.target_y_mm==380 && motion.target_map_yaw_deg==180);
    unsigned before=starts;ChassisRoute_Process();assert(starts==before && strcmp(status().state,"station"));
    if(pause_at==1){
     assert(ChassisRoute_TaskPause());confirm_paused();assert(ChassisRoute_TaskContinue());
     assert(last_turn && motion.target_y_mm==380 && motion.target_map_yaw_deg==180);
    }
    arrive(); /* Only completed turn+stop can submit approach. */
    assert(!last_turn && motion.target_y_mm==350 && motion.target_map_yaw_deg==180);
    assert(!strcmp(status().phase,"work_dock") && last_speed==50);
    assert(strcmp(status().state,"station"));
   }else if(s.station==2 && !strcmp(s.phase,"work_dock")){
    assert(!last_turn && motion.target_y_mm==350 && motion.target_map_yaw_deg==180);
    assert(loc.observer.feedback.y_mm==380 && last_speed==50);
    assert(strcmp(s.state,"station"));
    if(pause_at==2){
     loc.observer.feedback.y_mm=365; /* Mid-approach pause must not return to380. */
     assert(ChassisRoute_TaskPause());confirm_paused();assert(ChassisRoute_TaskContinue());
     assert(!last_turn && motion.target_y_mm==350 && motion.target_map_yaw_deg==180);
     assert(!strcmp(status().phase,"work_dock") && last_speed==50);
    }
    docks++;advance_clock(100);arrive();
    assert(!strcmp(status().state,"station") && status().stop_confirmed);
   }else {advance_clock(100);arrive();}
  }else ChassisRoute_Process();
 }
 assert(!strcmp(status().state,"done") && visits==2 && turns<=2 && docks==2);
 if(!mask)assert(turns==2);
}
int main(void)
{
 for(unsigned planned=0;planned<2;planned++)for(unsigned upper=0;upper<2;upper++)
  for(unsigned pause_at=0;pause_at<3;pause_at++)run(planned,upper,pause_at,0);
 run(true,false,2,1U<<9);run(true,true,2,1U<<9);
 puts("processing approach: both visits,380 turn then350 dock, fixed/radar, starts and pause PASS");return 0;
}
