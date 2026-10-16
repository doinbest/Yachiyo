/* Source-only: capture the cause before a pause can replace bus diagnostics. */
#define main existing_mission_main
#define ConsoleTx_Write FixtureConsoleWrite
#define GrabTask_Pause FixtureGrabPause
#include "contest_mission_test.c"
#undef ConsoleTx_Write
#undef GrabTask_Pause
#undef main

static unsigned trigger_lines;
static bool expect_trigger;
static char trigger_line[288];
bool ConsoleTx_Write(const uint8_t *data,uint16_t length)
{
  if(length>=13 && !memcmp(data,"OK bus first ",13)){
    assert(length<sizeof trigger_line);memcpy(trigger_line,data,length);trigger_line[length]=0;trigger_lines++;
  }
  return FixtureConsoleWrite(data,length);
}
bool GrabTask_Pause(void)
{
  assert(trigger_lines==(expect_trigger?1U:0U));
  recovery_diagnostic.reason="cancelled_reply";
  recovery_diagnostic.address=6;recovery_diagnostic.function=0x42;
  return FixtureGrabPause();
}
static void begin(void)
{
  tick=1000;memset(&grab,0,sizeof grab);memset(&route,0,sizeof route);memset(&motion,0,sizeof motion);
  grab.reason="idle";grab.result="none";route.state=route.phase=route.reason="idle";
  memset(&recovery_diagnostic,0,sizeof recovery_diagnostic);memset(&mission_recovery,0,sizeof mission_recovery);
  for(unsigned i=0;i<3;i++)inventory[i].state=TURNTABLE_EMPTY;
  cancel_defer=false;start_fixture_custom=false;grab_resume_pending=false;qr_scanning=false;trigger_lines=0;
  QR_Init();ContestMission_Init();assert(ContestMission_Start(NULL,&request));step();
}
int main(void)
{
  begin();expect_trigger=true;
  recovery_diagnostic=(MotorBus_Diagnostic_t){.locked=true,.reason="reply_timeout",.address=2,.function=0xf6,
    .owner=MOTOR_BUS_CHASSIS,.tick_ms=1234,.rx_bytes=4,.rx_sample_length=4,.rx_sample={7,0xfd,0x9f,0x6b}};
  assert(ContestMission_RequestBusRecovery(true));
  assert(strstr(trigger_line,"reason=reply_timeout addr=2 func=0xF6"));
  assert(strstr(trigger_line,"hex=07FD9F6B") && strstr(trigger_line,"fault_ms=1234"));
  assert(recovery_diagnostic.function==0x42 && trigger_lines==1);
  assert(ContestMission_RequestBusRecovery(true) && trigger_lines==1);
  begin();expect_trigger=false;
  recovery_diagnostic.reason="reply_timeout"; /* An unlocked historic cause is not current. */
  assert(ContestMission_RequestBusRecovery(false) && !trigger_lines);
  puts("mission recovery trigger is logged before stop cancellation; history and duplicate calls stay passive PASS");
  return 0;
}
