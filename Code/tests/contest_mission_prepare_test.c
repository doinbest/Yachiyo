#define main legacy_prepare_main
#include "contest_mission_test.c"
#undef main

static void reset_prepare(void)
{
 memset(&grab,0,sizeof grab);memset(&route,0,sizeof route);memset(&motion,0,sizeof motion);
 grab.reason="idle";grab.result="none";route.state=route.phase=route.reason="idle";
 starts=origin_calls=feedback_selections=0;feedback_enabled=false;start_fixture_custom=true;
 reject_route=0;
 start_feedback_missing=start_imu_missing=origin_rejected=false;start_wheel_flags=3;start_wheel_rpm=0;
 memset(&localization,0,sizeof localization);QR_Init();ContestMission_Init();
}
static void ready_localization(void)
{localization.feedback_valid=localization.speed_valid=true;}
int main(void)
{
 /* First start establishes map pose only after qualified fresh stopped wheels. */
 reset_prepare();start_feedback_missing=true;assert(ContestMission_Start(NULL,&request));
 step();assert(feedback_enabled && feedback_selections==1 && !origin_calls && !starts);
 ContestMission_Report();assert(strstr(report,"reason=start_feedback_wait"));
 for(unsigned i=0;i<20;i++)step();
 assert(ContestMission_IsBusy() && !starts && !origin_calls);
 start_feedback_missing=false;start_wheel_rpm=3;step();assert(!origin_calls && !starts);
 start_wheel_rpm=0;start_wheel_flags=0x89;step();assert(!origin_calls && !starts);
 start_wheel_flags=0;step();assert(!origin_calls && !starts);
 start_wheel_flags=3;start_imu_missing=true;step();assert(!origin_calls && !starts);
 start_imu_missing=false;origin_rejected=true;step();assert(!starts);
 origin_rejected=false;step();assert(origin_calls==2 && !starts && localization.origin_valid);
 ready_localization();step();assert(starts==1 && origin_calls==2 && feedback_selections==1);

 /* A matching valid origin is reused. A physically prepared new start can re-anchor stale pose. */
 reset_prepare();localization.origin=(ChassisModel_Pose_t){2250,150,CHASSIS_MODEL_PI};
 localization.observer.feedback=localization.origin;localization.origin_valid=true;ready_localization();
 assert(ContestMission_Start(NULL,&request));step();assert(starts==1 && !origin_calls);
 reset_prepare();localization.origin=(ChassisModel_Pose_t){2250,150,CHASSIS_MODEL_PI};
 localization.observer.feedback=(ChassisModel_Pose_t){350,1200,0};localization.origin_valid=true;ready_localization();
 assert(ContestMission_Start(NULL,&request));step();assert(origin_calls==1 && !starts);
 ready_localization();step();assert(starts==1 && origin_calls==1);

 /* Stop during preparation prevents automatic departure; explicit continue resumes the check. */
 reset_prepare();start_feedback_missing=true;assert(ContestMission_Start(NULL,&request));step();
 ContestMission_Stop();step();ContestMission_Report();assert(strstr(report,"state=paused"));
 start_feedback_missing=false;for(unsigned i=0;i<10;i++)step();assert(!starts && !origin_calls);
 assert(ContestMission_Continue());step();
 grab.state=GRAB_COMPLETE;grab.result="travel_ready";step();assert(origin_calls==1 && !starts);
 ready_localization();step();assert(starts==1);
 /* Stopping after origin creation preserves that anchor while feedback rebuilds. */
 reset_prepare();assert(ContestMission_Start(NULL,&request));step();assert(origin_calls==1 && !starts);
 ContestMission_Stop();step();assert(!starts);
 ready_localization();assert(ContestMission_Continue());step();
 grab.state=GRAB_COMPLETE;grab.result="travel_ready";step();
 assert(starts==1 && origin_calls==1);
 /* Verified IMU loss revokes the anchor; recovery before departure must not deadlock. */
 reset_prepare();assert(ContestMission_Start(NULL,&request));step();assert(origin_calls==1 && !starts);
 start_imu_missing=true;localization.origin_valid=false;step();assert(origin_calls==1 && !starts);
 start_imu_missing=false;step();assert(origin_calls==2 && !starts);
 ready_localization();step();assert(starts==1);
 reset_prepare();localization.origin=(ChassisModel_Pose_t){2250,150,CHASSIS_MODEL_PI};
 localization.observer.feedback=localization.origin;localization.origin_valid=true;ready_localization();
 reject_route=1;assert(ContestMission_Start(NULL,&request));step();assert(!starts && !origin_calls);
 start_imu_missing=true;localization.origin_valid=false;step();assert(!origin_calls);
 start_imu_missing=false;step();assert(origin_calls==1 && !starts);
 ready_localization();step();assert(starts==1);
 reset_prepare();assert(ContestMission_Start(NULL,&request));step();assert(origin_calls==1 && !starts);
 ContestMission_Stop();step();localization.origin_valid=false;
 for(unsigned i=0;i<5;i++)step();
 assert(origin_calls==1 && !starts);assert(ContestMission_Continue());step();
 grab.state=GRAB_COMPLETE;grab.result="travel_ready";step();assert(origin_calls==2 && !starts);
 ready_localization();step();assert(starts==1);
 puts("contest_mission_prepare_test: PASS (feedback wait, health gates, automatic origin, reuse, pause)");
}
