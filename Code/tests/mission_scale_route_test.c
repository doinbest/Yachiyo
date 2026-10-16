#define main legacy_route_main
#include "chassis_route_test.c"
#undef main

static void scaled_entry(float scale,float base)
{
  setup();ChassisRoute_TaskOptions_t options={true,1200,100,RADAR_START_LOWER};
  assert(ChassisRoute_TaskStartScaled(NULL,base,&options,scale));
  assert(!last_turn && fabsf(last_speed-fminf(base,300)*scale)<.01f);
  assert(fabsf(motion.linear_accel_mm_s2-500*scale)<.01f);
  assert(fabsf(motion.linear_decel_mm_s2-300*scale)<.01f);
}
static void whole_route(bool planned,float scale,float base)
{
  static const float caps[]={300,800,800,700,1000,50,800,800,50,50,800,1000,1000,50,800,800,50,50,800,800,800,300};
  static const float accelerations[]={500,800,800,800,800,300,800,800,300,300,800,300,800,300,800,800,300,300,800,800,800,500};
  setup();ChassisRoute_TaskOptions_t options={true,1200,100,RADAR_START_LOWER};
  RadarPlan_t plan={0};plan.valid=true;plan.count=CHASSIS_ROUTE_TARGET_COUNT+1;
  for(unsigned i=0;i<plan.count;i++)assert(ChassisRouteProfile_FixedPoint(i,&plan.points[i]));
  assert(ChassisRoute_TaskStartScaled(planned?&plan:NULL,base,&options,scale));
  unsigned index=0,seen=0;
  for(unsigned guard=0;guard<200 && strcmp(status().state,"done");guard++){
    if(!strcmp(status().state,"station")){
      assert(ChassisRoute_WorkBegin());if(status().station==2)loc.observer.feedback.y_mm=425;
      assert(ChassisRoute_WorkResume());
    }
    if(starts!=seen){
      seen=starts;
      if(last_turn)assert(turn_limit==100);
      else{
        assert(index<22);float expected=fminf(base,caps[index])*scale;
        assert(fabsf(last_speed-expected)<.01f);
        assert(fabsf(status().speed_mm_s-base*scale)<.01f && status().motion_scale==scale);
        if(index==11){assert(path_scale==scale && last_timeout>50000);if(scale<=.1f)assert(last_timeout>150000);}
        else{
          assert(fabsf(motion.linear_accel_mm_s2-accelerations[index]*scale)<.01f);
          assert(fabsf(motion.linear_decel_mm_s2-300*scale)<.01f);
        }
        index++;
        if(index==2){
          assert(ChassisRoute_TaskPause());motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;motion.reason="cancelled";
          ChassisRoute_Process();ChassisRoute_Process();
          assert(!ChassisRoute_TaskStartScaled(NULL,base,&options,1));
          assert(ChassisRoute_TaskContinue() && fabsf(last_speed-expected)<.01f);
          assert(status().motion_scale==scale);seen=starts;
        }
      }
    }
    advance_clock(100);if(ChassisMotion_IsBusy())arrive();else ChassisRoute_Process();
  }
  assert(index==22 && !strcmp(status().state,"done"));
}
static void corner_and_upper(void)
{
  setup();ChassisRoute_TaskOptions_t options={true,1200,50,RADAR_START_LOWER};
  assert(ChassisRoute_TaskStartScaled(NULL,1000,&options,1.5f));
  for(unsigned i=0;i<12 && strcmp(status().state,"station");i++){arrive();ChassisRoute_Process();}
  assert(ChassisRoute_WorkBegin());assert(ChassisRoute_WorkResume());
  assert(last_speed==1200 && corner_speed==1050 && corner_accel==1200);
  unsigned before=starts;loc.observer.feedback.y_mm=2095;corner_passed=true;
  motion.target_x_mm=1200;motion.target_y_mm=2100;ChassisRoute_Process();
  assert(starts==before && status().phase_speed_mm_s==1050);
  setup();options.start_id=RADAR_START_UPPER;loc.origin.y_mm=loc.observer.feedback.y_mm=2250;
  loc.origin.yaw_rad=loc.observer.feedback.yaw_rad=-CHASSIS_MODEL_PI/2;
  assert(ChassisRoute_TaskStartScaled(NULL,1000,&options,1.5f));
  assert(last_speed==450); /* Upper short diagonal keeps its baseline category. */
}
int main(void)
{
  scaled_entry(1.2f,1000);scaled_entry(.5f,200);scaled_entry(1,1000);
  whole_route(false,1.2f,1000);whole_route(true,1.2f,1000);
  whole_route(false,.5f,200);whole_route(false,.1f,1000);corner_and_upper();
  setup();ChassisRoute_TaskOptions_t options={true,1200,100,RADAR_START_LOWER};
  assert(!ChassisRoute_TaskStartScaled(NULL,1000,&options,NAN));
  assert(!ChassisRoute_TaskStartScaled(NULL,5000,&options,2));
  selected_profile=CHASSIS_PROFILE_COSINE;assert(!ChassisRoute_TaskStartScaled(NULL,1000,&options,1.2f));
  puts("mission scale route: PASS");return 0;
}
