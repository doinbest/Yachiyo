#include "GrabRoute.h"
#include "GrabTask.h"
#include "chassis_route.h"
#include "contest_screen.h"
#include "QR.h"
#include "console_tx.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum { GR_IDLE, GR_DRIVING, GR_SCAN_WAIT, GR_DISPLAY_WAIT, GR_HANDOFF, GR_ERROR, GR_CANCELLED, GR_DONE } GrabRoute_State;
static GrabRoute_State State;
static uint32_t StartTick, WaitTick, CodeSequence;
static const char *Reason;
static const char TestCode[] = "156+123+516+231";
bool GrabRoute_StartPlanned(const RadarPlan_t *plan,float speed)
{
  if(GrabRoute_IsBusy() || GrabTask_IsBusy() || !GrabTask_ConfigReady("pick")) return false;
  if(!ChassisRoute_PlanStart(plan,speed,true)) return false;
  StartTick=HAL_GetTick();CodeSequence=0U;State=GR_DRIVING;Reason="to_scan";
  return true;
}

static void Report(uint8_t Error)
{
  static const char *Names[]={"idle","driving","scan_wait","display_wait","handoff","error","cancelled","done"};
  ChassisRoute_Status_t Route;
  char Text[240];
  int Length;
  ChassisRoute_StatusGet(&Route);
  Length=snprintf(Text,sizeof(Text),"%s grab route state=%s segment=%lu source=simulated code=%s elapsed_ms=%lu reason=%s\r\n",
      Error?"ERR":"OK",Names[State],(unsigned long)Route.segment,TestCode,
      (unsigned long)(HAL_GetTick()-StartTick),Reason);
  if(Length>0 && Length<(int)sizeof(Text)) (void)ConsoleTx_Write((const uint8_t*)Text,(uint16_t)Length);
}

void GrabRoute_Init(void)
{
  State=GR_IDLE;StartTick=WaitTick=CodeSequence=0U;Reason="idle";
}

bool GrabRoute_IsBusy(void)
{
  return State==GR_DRIVING || State==GR_SCAN_WAIT || State==GR_DISPLAY_WAIT || State==GR_HANDOFF;
}

void GrabRoute_Stop(void)
{
  if(GrabRoute_IsBusy())
  {
    if(State!=GR_HANDOFF) (void)ChassisRoute_Cancel();
    State=GR_CANCELLED;Reason="operator_stop";
  }
  if(GrabTask_IsBusy()) GrabTask_Stop();
}

static void Fail(const char *Why)
{
  State=GR_ERROR;Reason=Why;
  (void)ChassisRoute_Cancel();
  Report(1U);
}

void GrabRoute_Process(void)
{
  ChassisRoute_Status_t Route;
  QR_SnapshotTypeDef Code;
  if(!GrabRoute_IsBusy()) return;
  if(State==GR_HANDOFF)
  {
    GrabTask_Status_t Grab;
    GrabTask_StatusGet(&Grab);
    if(Grab.state==GRAB_COMPLETE && !strcmp(Grab.result,"stored"))
    { State=GR_DONE;Reason="single_piece_stored";Report(0U); }
    else if(Grab.state==GRAB_ERROR || (Grab.state==GRAB_IDLE && !strcmp(Grab.result,"cancelled")))
    { State=GR_ERROR;Reason=Grab.reason;Report(1U); }
    return; /* This scope ends after one storage action; it never starts a second piece. */
  }
  ChassisRoute_StatusGet(&Route);
  if(!strcmp(Route.state,"error") || !strcmp(Route.state,"cancelled"))
  { Fail(Route.reason);return; }
  /* A normal arrival includes deceleration and feedback-confirmed stopping. */
  if(!strcmp(Route.state,"stopping")) return;
  if(CodeSequence)
  {
    QR_SnapshotGet(&Code);
    if(Code.Sequence!=CodeSequence || Code.Source!=QR_SOURCE_SIMULATED || strcmp(Code.Code,TestCode))
    { Fail("task_code_changed");return; }
  }
  if(State==GR_DRIVING)
  {
    if(strcmp(Route.state,"station")) return;
    if(!Route.stop_confirmed) { Fail("station_stop_unconfirmed");return; }
    if(Route.planned?(Route.station==5U && Route.visit==1U):Route.segment==2U)
    { State=GR_SCAN_WAIT;WaitTick=HAL_GetTick();Reason="simulated_scan_wait";Report(0U); }
    else if(Route.planned?(Route.station==4U && Route.visit==2U):Route.segment==4U)
    {
      if(!ChassisRoute_StationFinish()) { Fail("handoff_failed");return; }
      if(!GrabTask_Start("pick")) { Fail("grab_start_failed");return; }
      State=GR_HANDOFF;Reason="single_pick_started";Report(0U);
    }
    return;
  }
  if(strcmp(Route.state,"station") ||
     (Route.planned?(Route.station!=5U || Route.visit!=1U):Route.segment!=2U) || !Route.stop_confirmed)
  { Fail("station_lost");return; }
  if(State==GR_SCAN_WAIT && HAL_GetTick()-WaitTick>=1000U)
  {
    if(!QR_SimulatedSet(TestCode) || QR_ColorGet(0U,0U)!=1U ||
       !ContestScreen_ProgressSet(0U,0U) || !ContestScreen_TaskCodeRefresh(TestCode))
    { Fail("task_code_invalid");return; }
    QR_SnapshotGet(&Code);CodeSequence=Code.Sequence;
    State=GR_DISPLAY_WAIT;WaitTick=HAL_GetTick();Reason="screen_tx_pending";Report(0U);
  }
  else if(State==GR_DISPLAY_WAIT)
  {
    if(ContestScreen_HasError()) { Fail("screen_tx_error");return; }
    if(ContestScreen_TaskCodeSent())
    {
      if(!ChassisRoute_StationResume()) { Fail("route_resume_failed");return; }
      State=GR_DRIVING;Reason="to_material";Report(0U);
    }
    else if(HAL_GetTick()-WaitTick>=5000U) Fail("screen_tx_timeout");
  }
}

bool GrabRoute_Command(unsigned Count,char *Tokens[])
{
  char *End;
  float Speed;
  if(Count<2U || strcmp(Tokens[0],"grab")) return false;
  if(Count==2U && !strcmp(Tokens[1],"status")) { Report(0U);return false; }
  if(strcmp(Tokens[1],"route")) return false;
  if(GrabRoute_IsBusy() || GrabTask_IsBusy() || ChassisRoute_IsBusy())
  { static const char Busy[]="ERR grab route busy\r\n";(void)ConsoleTx_Write((const uint8_t*)Busy,sizeof(Busy)-1U);return true; }
  if(Count!=3U) { Reason="format";Report(1U);return true; }
  Speed=strtof(Tokens[2],&End);
  if(End==Tokens[2] || *End || !isfinite(Speed) || Speed<10.0f || Speed>5000.0f)
  { Reason="speed_range";Report(1U);return true; }
  if(!GrabTask_ConfigReady("pick")) { Reason="grab_config_missing";Report(1U);return true; }
  if(!ChassisRoute_StationStart(Speed)) { Reason="route_not_ready";Report(1U);return true; }
  StartTick=HAL_GetTick();CodeSequence=0U;State=GR_DRIVING;Reason="to_scan";Report(0U);
  return true;
}
