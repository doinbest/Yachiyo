#define main legacy_route_test_main
#include "chassis_route_test.c"
#undef main

static void run_mission(bool planned,bool upper,bool cosine,bool pause,bool intermediate)
{
  setup();selected_profile=cosine?CHASSIS_PROFILE_COSINE:CHASSIS_PROFILE_LINEAR;
  RadarStart_Id_t start=upper?RADAR_START_UPPER:RADAR_START_LOWER;
  if(upper){loc.origin.y_mm=loc.observer.feedback.y_mm=2250;
    loc.origin.yaw_rad=loc.observer.feedback.yaw_rad=-CHASSIS_MODEL_PI/2;}
  ChassisRoute_TaskOptions_t options={true,1200,100,start};
  RadarMap_t map;RadarPlan_t plan;
  RadarMap_Reset(&map);assert(RadarPlan_BuildFromStart(&map,start,&plan));
  if(intermediate){
    for(unsigned i=0;i<plan.count;i++)if(plan.points[i].station==2){
      unsigned at=i+1;
      while(at<plan.count && !plan.points[at].station && plan.points[at].y_mm<425)at++;
      assert(plan.count<RADAR_PLAN_MAX_POINTS);
      memmove(&plan.points[at+1],&plan.points[at],(plan.count-at)*sizeof plan.points[0]);
      plan.points[at]=(RadarPlan_Point_t){1200,700,0,0};plan.count++;
    }
  }
  assert(ChassisRoute_TaskStartEx(planned?&plan:NULL,1000,&options));
  unsigned processing=0,guard=0;
  while(strcmp(status().state,"done")){
    assert(++guard<100);finish_leg();
    if(!strcmp(status().state,"done"))break;
    if(strcmp(status().state,"station")){ChassisRoute_Process();continue;}
    assert(ChassisRoute_WorkBegin());
    if(status().station!=2){assert(ChassisRoute_WorkResume());continue;}
    processing++;assert(status().target_y_mm==350);
    unsigned before=starts;
    assert(!ChassisRoute_WorkResume()); /* The350 observation point is not departure-ready. */
    assert(starts==before && !strcmp(status().state,"work"));
    loc.observer.feedback.x_mm=1200;loc.observer.feedback.y_mm=425;
    loc.observer.feedback.yaw_rad=CHASSIS_MODEL_PI;
    motion.action_id++;motion.stop_confirmed=false;
    assert(!ChassisRoute_WorkResume() && starts==before);
    motion.stop_confirmed=true;grab_busy=true;clearance=false;
    assert(!ChassisRoute_WorkResume() && starts==before);grab_busy=false;
    if(pause){
      assert(ChassisRoute_TaskPause());motion.state=CHASSIS_MOTION_DONE;
      motion.stop_confirmed=true;motion.reason="cancelled";
      ChassisRoute_Process();ChassisRoute_Process();assert(!strcmp(status().state,"paused"));
      assert(ChassisRoute_TaskContinue());motion.reason="feedback_arrived";
      assert(!strcmp(status().state,"work") && starts==before);
    }
    assert(ChassisRoute_WorkResume());
    assert(motion.target_x_mm==1200 && motion.target_y_mm==(intermediate?700:425));
    assert(last_turn==!intermediate && motion.target_map_yaw_deg==(intermediate?180:90));
    if(pause){
      assert(ChassisRoute_TaskPause());motion.state=CHASSIS_MOTION_DONE;
      motion.stop_confirmed=true;motion.reason="cancelled";
      ChassisRoute_Process();ChassisRoute_Process();assert(ChassisRoute_TaskContinue());
      assert(last_turn==!intermediate && motion.target_y_mm==(intermediate?700:425));
    }
    arrive();
    if(intermediate){
      ChassisRoute_Process();assert(last_turn && motion.target_y_mm==700 && motion.target_map_yaw_deg==90);
      arrive();
    }
    assert(!last_turn && motion.target_x_mm==1200 && motion.target_y_mm==1200);
    assert(!strcmp(status().phase,"move")); /* No replay of350->425 or radar's350 connector. */
  }
  assert(processing==2);
}
int main(void)
{
  for(unsigned planned=0;planned<2;planned++)
    for(unsigned upper=0;upper<2;upper++)
      for(unsigned cosine=0;cosine<2;cosine++)
        for(unsigned pause=0;pause<2;pause++){
          run_mission(planned,upper,cosine,pause,false);
          if(planned)run_mission(true,upper,cosine,pause,true);
        }
  puts("chassis_processing_exit_test: PASS (both batches, real radar/fixed, both starts/profiles, pause)");
}
