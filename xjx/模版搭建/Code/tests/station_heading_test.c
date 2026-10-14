#include "station_heading.h"
#include "Camera.h"
#include "GrabTask.h"
#include "chassis_motion.h"
#include "chassis_localization.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static uint32_t tick;
static Camera_PoseSnapshot_t frame;
static StationHeading_Config_t config;
static ChassisMotion_Status_t motion;
static ChassisStop_Status_t independent_stop;
static unsigned requests,commits,turns;
static bool ready=true,accept=true;
static HAL_StatusTypeDef camera_start_result;
uint32_t HAL_GetTick(void){return tick;}
bool GrabTask_HeadingReady(void){return ready;}
void GrabTask_HeadingConfigGet(unsigned station,StationHeading_Config_t *out){(void)station;*out=config;}
HAL_StatusTypeDef Camera_PoseStart(void){requests++;if(camera_start_result!=HAL_OK)return camera_start_result;memset(&frame,0,sizeof frame);frame.Active=1;return HAL_OK;}
void Camera_PoseSnapshotGet(Camera_PoseSnapshot_t *out){*out=frame;}
void Camera_RequestStop(void){frame.Active=0;}
void Mecanum_Feedback_Enable(bool enable){(void)enable;}
void ChassisMotion_StatusGet(ChassisMotion_Status_t *out){*out=motion;}
void ChassisMotion_StopStatusGet(ChassisStop_Status_t *out){*out=independent_stop;}
bool ChassisMotion_IsBusy(void){return motion.state==CHASSIS_MOTION_RUNNING;}
void ChassisLocalization_Get(ChassisLocalization_Status_t *out){memset(out,0,sizeof *out);out->origin_valid=out->feedback_valid=out->speed_valid=true;out->feedback_tick=tick;for(unsigned i=0;i<4;i++){out->wheels[i].state_valid=true;out->wheels[i].state_flags=1;out->wheels[i].state_ms=tick;}}
bool ChassisHeading_Apply(float yaw){if(!accept)return false;commits++;motion.map_yaw_rad=yaw;return true;}
bool ChassisMotion_TurnToPrecision(float yaw,float speed,float tol,uint32_t timeout){assert(yaw==180 && speed==10 && tol==.1f && timeout);turns++;motion.state=CHASSIS_MOTION_RUNNING;return true;}
static bool sample(int angle,int stop){tick+=100;frame.HasFrame=frame.Valid=1;frame.Sequence++;frame.ReceivedTick=tick;frame.AngleCdeg=angle;frame.Stop=stop;return StationHeading_Process();}
static void reset(void){memset(&independent_stop,0,sizeof independent_stop);tick=0;requests=commits=turns=0;ready=accept=true;camera_start_result=HAL_OK;memset(&motion,0,sizeof motion);motion.heading_valid=motion.map_anchor_valid=motion.stop_confirmed=true;motion.map_yaw_rad=CHASSIS_MODEL_PI;config=(StationHeading_Config_t){.zero_deg=0,.gain=1,.tolerance_deg=.1f,.spread_deg=.05f,.settle_ms=0,.turn_limit_deg=1,.wait_ms=1200,.observation_z_mm=0,.height_matches=true};StationHeading_Reset();}
static void intermittent_samples(void)
{
  /* Invalid reports and long gaps must not discard consistent new samples. */
  for(unsigned uncal=0;uncal<2;uncal++) {
    reset();config.wait_ms=12000;if(uncal)config.gain=NAN;
    StationHeading_Begin(2,180,10);assert(!StationHeading_Process());
    assert(!sample(0,0));
    tick=900;frame.Valid=0;frame.Sequence++;frame.ReceivedTick=tick;
    assert(!StationHeading_Process());
    tick=1700;assert(!sample(0,1));
    tick=3500;assert(!StationHeading_Process()); /* Old snapshot is not a third sample. */
    tick=4100;assert(sample(0,0));
    assert(!strcmp(StationHeading_Reason(),uncal?"heading_uncalibrated":"heading_verified"));
    assert(!commits && !turns);
  }
  /* Without calibration, three inconsistent images must not qualify either. */
  reset();config.gain=NAN;config.wait_ms=12000;
  StationHeading_Begin(2,180,10);StationHeading_Process();
  assert(!sample(100,0));assert(!sample(300,0));assert(!sample(500,0));
  tick=4999;assert(!StationHeading_Process());
  tick=5000;assert(StationHeading_Process());assert(!strcmp(StationHeading_Reason(),"heading_no_observation"));
  /* No fresh data, repeated snapshots or temporarily lost readiness cannot extend 5 s. */
  reset();config.wait_ms=12000;StationHeading_Begin(2,180,10);StationHeading_Process();
  sample(0,0);sample(0,0);ready=false;tick=5000;assert(StationHeading_Process());
  assert(!commits && !turns);
  /* Reordered samples are not new evidence. */
  reset();config.wait_ms=12000;StationHeading_Begin(2,180,10);StationHeading_Process();
  assert(!sample(0,0));assert(!sample(0,0));
  tick+=100;frame.Sequence=1;frame.ReceivedTick=tick;assert(!StationHeading_Process());
  assert(!commits);frame.Sequence=2;assert(sample(0,0));
  /* Consistency covers image noise and wrapping, but never mixes moving poses. */
  reset();config.gain=NAN;config.wait_ms=5000;
  StationHeading_Begin(2,180,10);StationHeading_Process();
  assert(!sample(100,0));tick=1800;assert(!sample(110,0));tick=4000;assert(sample(90,0));
  assert(fabsf(StationHeading_ImageAngle()-1.0f)<.001f && !commits && !turns);
  reset();config.gain=NAN;StationHeading_Begin(2,180,10);StationHeading_Process();
  assert(!sample(8995,0));assert(!sample(-8995,0));assert(sample(8995,0));
  assert(fabsf(fabsf(StationHeading_ImageAngle())-90)<.1f);
  reset();config.wait_ms=5000;StationHeading_Begin(2,180,10);StationHeading_Process();
  sample(0,0);sample(0,0);ready=false;assert(!StationHeading_Process());
  ready=true;assert(!StationHeading_Process());
  assert(!sample(0,0));assert(!sample(0,0));assert(sample(0,0));
  reset();config.wait_ms=12000;camera_start_result=HAL_BUSY;
  StationHeading_Begin(2,180,10);assert(!StationHeading_Process());
  tick=4999;assert(!StationHeading_Process());tick=5000;assert(StationHeading_Process());
  assert(!commits && !turns);
  /* Applied corrections never abandon an in-flight turn at the sampling deadline. */
  reset();config.wait_ms=5000;StationHeading_Begin(2,180,10);StationHeading_Process();
  sample(50,0);sample(50,0);sample(50,0);StationHeading_Process();assert(commits==1 && turns==1);
  tick=6000;assert(!StationHeading_Process());
  motion.state=CHASSIS_MOTION_DONE;assert(!StationHeading_Process());
  assert(!sample(0,0));assert(!sample(0,0));assert(sample(0,0));
  /* Before map application succeeds, a busy commit still obeys the deadline. */
  reset();config.wait_ms=12000;accept=false;StationHeading_Begin(2,180,10);StationHeading_Process();
  sample(50,0);sample(50,0);sample(50,0);assert(!commits);
  tick=5000;assert(StationHeading_Process());assert(!commits && !turns);
}
int main(void){
  intermittent_samples();
  /* Trial observations must not apply an already configured correction. */
  reset();StationHeading_ObserveBegin(2,180,5);StationHeading_Process();
  assert(!sample(951,0));assert(!sample(951,1));assert(sample(951,0));
  assert(!strcmp(StationHeading_Reason(),"heading_observed") && commits==0 && turns==0);

  /* A standalone stop is physical proof, not distance-task arrival. */
  reset();config.gain=NAN;motion.stop_confirmed=false;
  StationHeading_Begin(2,180,10);
  assert(!StationHeading_Process() && requests==0);
  independent_stop.tx_complete=true;
  assert(!StationHeading_Process() && requests==0);
  independent_stop.wheels_stopped=true;ready=false;
  assert(!StationHeading_Process() && requests==0);
  ready=true;assert(!StationHeading_Process() && requests==1);
  assert(!motion.stop_confirmed); /* Never manufacture arrival. */
  assert(!sample(50,1));assert(!sample(50,1));assert(sample(50,1));
  assert(!strcmp(StationHeading_Reason(),"heading_uncalibrated") && commits==0 && turns==0);
  reset();motion.stop_confirmed=false;independent_stop.tx_complete=independent_stop.wheels_stopped=true;
  StationHeading_Begin(2,180,10);StationHeading_Process();assert(requests==1);
  independent_stop.wheels_stopped=false; /* Production StopStatus invalidates after new motion. */
  assert(!StationHeading_Process() && !frame.Active && requests==1);

  reset();config.gain=NAN;StationHeading_Begin(2,180,10);assert(!StationHeading_Process());
  assert(!sample(50,1));assert(!sample(50,0));assert(sample(50,1));
  assert(commits==0 && turns==0 && !strcmp(StationHeading_Reason(),"heading_uncalibrated"));
  for(int stop=0;stop<=1;stop++){
    reset();StationHeading_Begin(2,180,10);StationHeading_Process();
    assert(!sample(50,stop));assert(!sample(50,stop));assert(!sample(50,stop));
    assert(commits==1);StationHeading_Process();assert(turns==1);
    StationHeading_Pause();motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;
    StationHeading_Process();assert(requests==2);assert(!sample(0,stop));assert(!sample(0,stop));assert(sample(0,stop));
    assert(!strcmp(StationHeading_Reason(),"heading_verified"));
  }
  reset();StationHeading_Begin(2,180,10);StationHeading_Process();sample(0,0);
  for(unsigned i=0;i<10;i++){assert(!StationHeading_Process());}
  assert(commits==0);
  frame.Valid=0;frame.Sequence++;assert(!StationHeading_Process());
  ready=false;assert(!sample(0,1));assert(!sample(0,1));assert(commits==0);
  reset();StationHeading_Begin(2,180,10);StationHeading_Process();tick=1300;assert(StationHeading_Process());assert(!strcmp(StationHeading_Reason(),"heading_no_observation"));
  reset();config.turn_limit_deg=0;StationHeading_Begin(2,180,10);StationHeading_Process();sample(50,1);sample(50,1);sample(50,1);assert(!StationHeading_Process() && turns==0);assert(!strcmp(StationHeading_Reason(),"heading_turn_unverified"));
  reset();StationHeading_Begin(2,180,10);StationHeading_Process();sample(0,0);
  tick=1300;assert(StationHeading_Process()); /* Frozen valid snapshot cannot stall forever. */
  reset();StationHeading_Begin(2,180,10);StationHeading_Process();sample(0,0);sample(0,1);assert(sample(0,0));assert(commits==0); /* Noise-consistent map stays fixed. */
  reset();StationHeading_Begin(2,180,10);StationHeading_Process();sample(50,1);sample(50,1);sample(50,1);
  StationHeading_Pause();config.gain=NAN;StationHeading_Process();sample(0,1);sample(0,1);assert(!sample(0,1));
  /* A loose stability spread must never preserve an incorrect map or enlarge the allowed turn. */
  reset();config.spread_deg=.3f;config.turn_limit_deg=.25f;motion.map_yaw_rad=180.4f*CHASSIS_MODEL_PI/180;
  StationHeading_Begin(2,180,10);StationHeading_Process();sample(20,1);sample(20,1);sample(20,1);
  assert(commits==1 && fabsf(remainderf(motion.map_yaw_rad*180/CHASSIS_MODEL_PI-180.2f,360))<.001f);
  motion.map_yaw_rad=180.4f*CHASSIS_MODEL_PI/180;assert(!StationHeading_Process() && turns==0);
  assert(!strcmp(StationHeading_Reason(),"heading_turn_unverified"));
  /* Paused time is not part of the next observation opportunity. */
  reset();config.gain=NAN;StationHeading_Begin(2,180,10);StationHeading_Process();sample(50,1);
  StationHeading_Pause();tick+=20000;assert(!StationHeading_Process());assert(!StationHeading_Process());
  assert(!sample(50,1));assert(!sample(50,1));assert(sample(50,1));assert(commits==0 && turns==0);
  /* A higher ready pose may be observed, but cannot borrow the lower-pose calibration. */
  reset();config.observation_z_mm=0;config.height_matches=false;
  StationHeading_Begin(2,180,10);StationHeading_Process();
  assert(!sample(50,1));assert(!sample(50,1));assert(sample(50,1));
  assert(commits==0 && turns==0 && StationHeading_ObservationZ()==0);
  assert(!strcmp(StationHeading_Reason(),"heading_height_unmatched"));
  puts("station heading: observation-only, STOP independence, pause/reacquire, missing/duplicate/invalid samples PASS");
}
