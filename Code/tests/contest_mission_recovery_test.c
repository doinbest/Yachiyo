#define main legacy_mission_main
#include "contest_mission_test.c"
#undef main
static void fixture(void)
{
  tick=1000;memset(&grab,0,sizeof grab);memset(&route,0,sizeof route);memset(&motion,0,sizeof motion);
  grab.reason="idle";grab.result="none";route.state=route.phase=route.reason="idle";
  memset(&recovery_diagnostic,0,sizeof recovery_diagnostic);memset(&mission_recovery,0,sizeof mission_recovery);
  recovery_starts=0;cancel_defer=false;start_fixture_custom=false;grab_resume_pending=false;qr_scanning=false;
  QR_Init();ContestMission_Init();assert(ContestMission_Start(NULL,&request));step();
}
static void finish_recovery(bool success)
{
  mission_recovery.active=false;mission_recovery.success=success;
  mission_recovery.checked_mask=success?0xff:0;mission_recovery.reason=success?"complete":"motor_fault";
  if(success)recovery_diagnostic.locked=false;
  for(unsigned i=0;i<10;i++)step();
}
int main(void)
{
  request.profile=CHASSIS_PROFILE_COSINE;fixture();
  assert(selected_profile==CHASSIS_PROFILE_COSINE);
  request.profile=CHASSIS_PROFILE_LINEAR;
  ContestMission_Stop();step();assert(ContestMission_HasPendingWork());
  assert(!ContestMission_Start(NULL,&request) && selected_profile==CHASSIS_PROFILE_COSINE);
  selected_profile=CHASSIS_PROFILE_LINEAR; /* Simulated controller reinitialization. */
  assert(ContestMission_Continue());step();
  assert(selected_profile==CHASSIS_PROFILE_COSINE);ContestMission_Report();assert(strstr(report,"profile=cosine"));
  assert(ContestMission_RequestBusRecovery(true));
  for(unsigned i=0;i<5 && !recovery_starts;i++)step();
  selected_profile=CHASSIS_PROFILE_LINEAR;
  finish_recovery(true);assert(selected_profile==CHASSIS_PROFILE_COSINE && ContestMission_IsBusy());
  assert(ContestMission_RequestBusRecovery(false));
  for(unsigned i=0;i<5 && !mission_recovery.active;i++)step();
  selected_profile=CHASSIS_PROFILE_LINEAR;finish_recovery(true);
  ContestMission_Report();assert(strstr(report,"state=paused") && strstr(report,"profile=cosine"));
  assert(ContestMission_Continue());step();assert(selected_profile==CHASSIS_PROFILE_COSINE);
  fixture();unsigned origin=origin_calls,start_count=starts;
  recovery_diagnostic=(MotorBus_Diagnostic_t){.locked=true,.reason="uart_error",.uart_error=HAL_UART_ERROR_NE};
  for(unsigned i=0;i<5 && !recovery_starts;i++)step();
  assert(recovery_starts==1 && ContestMission_HasPendingWork());
  assert(!ContestMission_Start(NULL,&request));
  finish_recovery(true);assert(ContestMission_IsBusy() && origin_calls==origin && starts==start_count);
  /* An operator pause revokes automatic resume even if recovery then succeeds. */
  fixture();recovery_diagnostic=(MotorBus_Diagnostic_t){.locked=true,.reason="reply_timeout"};
  for(unsigned i=0;i<5 && !recovery_starts;i++)step();
  ContestMission_Stop();
  finish_recovery(true);ContestMission_Report();assert(strstr(report,"state=paused"));
  assert(ContestMission_Continue());
  /* Failed validation waits; explicit bus retry preserves this match. */
  fixture();assert(ContestMission_RequestBusRecovery(false));
  for(unsigned i=0;i<5 && !recovery_starts;i++)step();
  finish_recovery(false);
  assert(ContestMission_HasPendingWork() && ContestMission_CanConfigure());
  assert(ContestMission_RequestBusRecovery(false));
  for(unsigned i=0;i<5 && recovery_starts<2;i++)step();
  finish_recovery(true);
  ContestMission_Report();assert(strstr(report,"state=paused") && recovery_starts==2);
  /* Recover at the user's actual boundary: three stored, empty first ring,
   * never restart scanning/raw collection or take material before alignment. */
  fixture();raw=proc=place=transfers=ring_aligns=ring_captures=rough_picks=0;
  scan_station();station(4,2);alignment_blocked=true;
  route.phase="work_turn";ContestMission_Process();step();
  route.state="station";route.station=2;route.visit=3;route.stop_confirmed=true;
  route.target_x_mm=1200;route.target_y_mm=350;
  for(unsigned i=0;i<12;i++)step();
  assert(grab.state==GRAB_HOLD && !rough_picks);
  recovery_diagnostic=(MotorBus_Diagnostic_t){.locked=true,.reason="uart_error",.uart_error=HAL_UART_ERROR_NE};
  for(unsigned i=0;i<5 && !recovery_starts;i++)step();
  finish_recovery(true);
  ContestMission_Report();assert(strstr(report,"stored=3") && strstr(report,"placed=0"));
  assert(raw==3 && !rough_picks && ring_aligns==1);alignment_blocked=false;
  /* End-match cancellation cancels recovery and cannot later restart it. */
  assert(ContestMission_RequestBusRecovery(true));step();ContestMission_Cancel();
  for(unsigned i=0;i<10;i++)step();
  assert(!ContestMission_Continue() && !mission_recovery.active);
  /* Communication failure during recovery retries; health rejection above
   * still waits for the operator. Pause revokes only automatic continuation. */
  fixture();assert(ContestMission_RequestBusRecovery(true));
  for(unsigned i=0;i<5 && !recovery_starts;i++)step();
  mission_recovery.active=false;mission_recovery.success=false;mission_recovery.reason="reply_timeout";
  for(unsigned i=0;i<20 && recovery_starts<2;i++)step();
  assert(recovery_starts==2);
  ContestMission_Stop();finish_recovery(true);ContestMission_Report();assert(strstr(report,"state=paused"));
  fixture();assert(ContestMission_RequestBusRecovery(true));
  for(unsigned i=0;i<5 && !recovery_starts;i++)step();
  mission_recovery.active=false;mission_recovery.reason="uart_error";mission_recovery.uart_error=HAL_UART_ERROR_DMA;
  for(unsigned i=0;i<20;i++)step();
  assert(recovery_starts==1 && ContestMission_CanConfigure());
  fixture();assert(ContestMission_RequestBusRecovery(true));
  for(unsigned i=0;i<200;i++)
  {
    if(mission_recovery.active){mission_recovery.active=false;mission_recovery.success=false;
      mission_recovery.reason="uart_error";mission_recovery.uart_error=HAL_UART_ERROR_NE;}
    step();
  }
  ContestMission_Report();assert(strstr(report,"state=bus_recovery_wait") && strstr(report,"bus_recovery_retry_exhausted"));
  assert(recovery_starts>1 && ContestMission_HasPendingWork());
  assert(ContestMission_RequestBusRecovery(false));
  for(unsigned i=0;i<5 && !mission_recovery.active;i++)step();
  finish_recovery(true);ContestMission_Report();assert(strstr(report,"state=paused"));
  fixture();assert(ContestMission_RequestBusRecovery(true));
  for(unsigned i=0;i<5 && !recovery_starts;i++)step();
  mission_recovery.active=false;mission_recovery.reason="reply_timeout";step();
  unsigned attempts=recovery_starts;ContestMission_Cancel();
  for(unsigned i=0;i<20;i++)step();
  assert(recovery_starts==attempts && !ContestMission_Continue());
  /* New transport errors during the fresh post-recovery stop proof belong
   * to the same retry episode, not to an unrelated final-stop timeout. */
  fixture();assert(ContestMission_RequestBusRecovery(true));
  for(unsigned i=0;i<5 && !recovery_starts;i++)step();
  mission_recovery.active=false;mission_recovery.success=true;mission_recovery.checked_mask=0xff;
  step(); /* enters fresh stop verification */
  recovery_diagnostic=(MotorBus_Diagnostic_t){.locked=true,.reason="uart_error",.uart_error=HAL_UART_ERROR_NE,.address=7,.function=0x36};
  step();ContestMission_Report();assert(strstr(report,"state=bus_recovery_retry"));
  MotorBus_Recovery_t evidence;assert(ContestMission_BusRecoveryEvidenceGet(&evidence));
  assert(!strcmp(evidence.stage,"stop_verify") && evidence.uart_error==HAL_UART_ERROR_NE && evidence.address==7);
  for(unsigned i=0;i<20 && recovery_starts<2;i++)step();
  assert(recovery_starts==2);finish_recovery(true);
  /* The pasted run reached stop_wait after recovery; pause must also revoke
   * auto_resume here, even though the physical stop was already confirmed. */
  fixture();assert(ContestMission_RequestBusRecovery(true));
  for(unsigned i=0;i<10 && !recovery_starts;i++)step();
  mission_recovery.active=false;mission_recovery.success=true;mission_recovery.checked_mask=0xff;
  recovery_diagnostic.locked=false;
  for(unsigned i=0;i<20;i++){step();if(strstr(report,"state=resuming "))break;}
  assert(strstr(report,"state=resuming "));
  motion.state=CHASSIS_MOTION_ERROR;motion.reason="feedback_invalid";
  ContestMission_Process();ContestMission_Report();assert(strstr(report,"state=stop_wait ") && strstr(report,"auto_resume=1"));
  ContestMission_Stop();ContestMission_Report();assert(strstr(report,"auto_resume=0"));
  puts("mission recovery: retained route, explicit retry, pause/stop override automatic resume PASS");
}
