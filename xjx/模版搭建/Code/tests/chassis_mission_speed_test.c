#define main legacy_route_main
#include "chassis_route_test.c"
#undef main

typedef struct {float x,y,speed,accel;bool curve;} Expected;
static const Expected moves[]={
 {2100,300,300,500,false},{2100,1200,800,800,false},
 {2100,2100,800,800,false},{1200,2100,700,800,false},
 {1200,380,1000,800,false},{1200,350,50,300,false},
 {1200,1200,800,800,false},{425,1200,800,800,false},
 {350,1200,50,300,false},{425,1200,50,300,false},
 {1200,1200,800,800,false},{1200,2100,800,300,true},
 {1200,380,1000,800,false},{1200,350,50,300,false},
 {1200,1200,800,800,false},{425,1200,800,800,false},
 {350,1200,50,300,false},{425,1200,50,300,false},
 {1200,1200,800,800,false},{2100,1200,800,800,false},
 {2100,300,800,800,false},{2250,150,300,500,false}
};
static void run_route(bool planned,float ceiling)
{
 setup();ChassisRoute_TaskOptions_t options={true,1200,100,RADAR_START_LOWER};
 RadarPlan_t plan={0};plan.valid=true;plan.count=CHASSIS_ROUTE_TARGET_COUNT+1U;
 for(unsigned i=0;i<plan.count;i++)assert(ChassisRouteProfile_FixedPoint(i,&plan.points[i]));
 assert(ChassisRoute_TaskStartEx(planned?&plan:NULL,ceiling,&options));
 unsigned index=0,seen=0,turns=0;
 for(unsigned guard=0;guard<200 && strcmp(status().state,"done");guard++){
   if(!strcmp(status().state,"station")){
     assert(ChassisRoute_WorkBegin());
     if(status().station==2U){
       /* The mission returns locally to425 before handing route ownership
        * back; the observation target is350 in route status. */
       loc.observer.feedback.y_mm=425;
     }
     assert(ChassisRoute_WorkResume());
   }
   if(starts!=seen){
     seen=starts;
     if(last_turn){assert(turn_limit==100);turns++;}
     else{
       assert(index<sizeof moves/sizeof moves[0]);const Expected *e=&moves[index++];
       assert(motion.target_x_mm==e->x && motion.target_y_mm==e->y);
       assert(last_speed==fminf(ceiling,e->speed));
       assert(status().phase_speed_mm_s==last_speed);
       if(!e->curve){
         assert(motion.linear_accel_mm_s2==e->accel);
         assert(motion.linear_decel_mm_s2==300);
         assert(motion.heading_kp==.1f && motion.heading_kd==0);
       }else assert(!strcmp(status().phase,"work_curve"));
       /* A pause must retain the accepted per-phase allocation. */
       if(index==2){
         float accel=motion.linear_accel_mm_s2,speed=last_speed;
         assert(ChassisRoute_TaskPause());motion.state=CHASSIS_MOTION_DONE;
         motion.stop_confirmed=true;motion.reason="cancelled";
         ChassisRoute_Process();ChassisRoute_Process();
         assert(ChassisRoute_TaskContinue());ChassisRoute_Process();
         assert(last_speed==speed && motion.linear_accel_mm_s2==accel);
         seen=starts;
       }
     }
   }
   advance_clock(100);if(ChassisMotion_IsBusy())arrive();else ChassisRoute_Process();
 }
 assert(!strcmp(status().state,"done"));assert(index==22 && turns==16);
}
int main(void)
{
 run_route(false,1000);run_route(true,1000);run_route(false,200);
 setup();selected_profile=CHASSIS_PROFILE_COSINE;
 assert(ChassisRoute_TaskStart(NULL,1000));assert(!strcmp(status().phase,"entry_curve"));
 assert(last_speed==1000); /* Independent curve/cosine behavior is retained. */
 setup();assert(ChassisRoute_QrTestStart(200,1200));
 assert(last_speed==200 && motion.linear_accel_mm_s2==300);
 puts("formal mission allocation:22 translations,16 turns, planned/fixed, low ceiling, pause and independent tests PASS");
}
