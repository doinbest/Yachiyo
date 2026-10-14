#define main legacy_mission_test_main
#include "contest_mission_test.c"
#undef main

static void uart_code(const char *code)
{
  for(;*code;code++)QR_ReceiveData((uint8_t)*code);
  QR_ReceiveData('\r');
}
static void reset_scan_fixture(void)
{
  memset(&grab,0,sizeof grab);memset(&route,0,sizeof route);memset(&motion,0,sizeof motion);
  grab.reason="idle";grab.result="none";route.state=route.phase=route.reason="idle";
  for(unsigned i=0;i<3;i++)inventory[i].state=TURNTABLE_EMPTY;
  raw=transfers=resumes=starts=screen_refreshes=0;qr_scanning=screen_error=false;screen_sent=true;
  QR_Init();ContestMission_Init();
}
static void reach_scanner(void)
{
  step();assert(starts==1);
  route.state="station";route.station=5;route.visit=1;route.stop_confirmed=true;
  route.target_x_mm=2100;route.target_y_mm=1250;
  for(unsigned i=0;i<5;i++)step();
  assert(qr_scanning && !raw && !resumes);
}
int main(void)
{
  const char *actual="356+123+536+231",*other="156+123+516+231";
  ContestMission_Request_t scan={MISSION_CODE_QR,"",200,50,RADAR_START_LOWER,false,false,false,CHASSIS_PROFILE_LINEAR};
  reset_scan_fixture();uart_code(actual);strcpy(screen_code,other);
  inventory[0]=(Turntable_Inventory_t){TURNTABLE_OCCUPIED,4};
  inventory[1].state=TURNTABLE_UNKNOWN;
  assert(ContestMission_Start(NULL,&scan));
  for(unsigned i=0;i<3;i++)assert(inventory[i].state==TURNTABLE_EMPTY && inventory[i].color==0);
  assert(!screen_code[0]);reach_scanner();
  /* A cached UART result and any simulated update are not this scan. */
  assert(QR_SimulatedSet(actual));for(unsigned i=0;i<5;i++)step();
  assert(qr_scanning && !screen_code[0] && !resumes);
  uart_code(actual);step();ContestMission_Report();
  assert(strstr(report,"state=qr_display") && strstr(report,"code_source=qr") &&
         strstr(report,"exec_code=356+123+536+231") && !strcmp(screen_code,actual));
  /* Busy display and a late different frame cannot change the adopted code. */
  screen_sent=false;uart_code(other);for(unsigned i=0;i<20;i++)step();
  assert(!resumes);ContestMission_Report();assert(strstr(report,"exec_code=356+123+536+231"));
  screen_error=true;unsigned refreshes=screen_refreshes;
  for(unsigned i=0;i<15;i++)step();
  assert(screen_refreshes>refreshes && !strcmp(screen_code,actual) && !resumes);
  screen_error=false;screen_sent=true;
  for(unsigned i=0;i<10 && !resumes;i++)step();
  assert(resumes==1);uart_code(other);step();ContestMission_Report();
  assert(strstr(report,"exec_code=356+123+536+231") && strstr(report,"stored=0 placed=0"));

  reset_scan_fixture();assert(ContestMission_Start(NULL,&scan));reach_scanner();
  uart_code("113+123+113+231");step();ContestMission_Report();
  assert(strstr(report,"reason=task_code_invalid") && strstr(report,"exec_code=none"));
  assert(!strcmp(screen_code,"113+123+113+231") && !resumes);
  step();assert(qr_scanning);uart_code(actual);step();
  for(unsigned i=0;i<10 && !resumes;i++)step();
  assert(resumes==1);

  reset_scan_fixture();ContestMission_Request_t bad={MISSION_CODE_FIXED,"113+123+113+231",200,50,RADAR_START_LOWER,false,false,false,CHASSIS_PROFILE_LINEAR};
  unsigned poses=pose_calls;assert(!ContestMission_Start(NULL,&bad));
  assert(!strcmp(ContestMission_Reason(),"task_code_invalid") && pose_calls==poses);
  assert(ContestMission_Start(NULL,&request));reach_scanner();
  uart_code(actual);step();ContestMission_Report();
  assert(strstr(report,"exec_code=156+123+516+231") && !strcmp(screen_code,actual));
  /* Start and lidar are independent of execution-code source. */
  for(unsigned upper=0;upper<2;upper++)for(unsigned radar=0;radar<2;radar++){
    reset_scan_fixture();navigation_calls=qr_pose_calls=0;navigation_result=0;
    start_fixture_custom=true;memset(&localization,0,sizeof localization);
    localization.origin_valid=localization.feedback_valid=localization.speed_valid=true;
    localization.origin=(ChassisModel_Pose_t){2250,upper?2250:150,upper?-CHASSIS_MODEL_PI/2:CHASSIS_MODEL_PI};
    localization.observer.feedback=localization.origin;
    scan.start_id=upper?RADAR_START_UPPER:RADAR_START_LOWER;scan.radar_enabled=radar;
    assert(ContestMission_Start(NULL,&scan));for(unsigned j=0;j<5;j++)step();
    if(radar){assert(!starts && navigation_calls && !qr_pose_calls);
      navigation_result=-1;step();assert(!starts && !strcmp(ContestMission_Reason(),"radar_no_route"));
      assert(ContestMission_Continue());navigation_result=1;for(unsigned j=0;j<5;j++)step();
      assert(navigation_start==scan.start_id);
    }else assert(!navigation_calls);
    assert(starts==1 && route_start==scan.start_id && route_planned==(bool)radar);
    assert(qr_pose_calls==upper);ContestMission_Report();
    assert(strstr(report,upper?"start=upper":"start=lower"));
    assert(strstr(report,radar?"radar=1":"radar=0"));
  }
  puts("contest_mission_scan_test: PASS (both sources, cache rejection, new identical UART, invalid order, display retry, frozen code)");
}
