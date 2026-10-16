/* Legacy mission fixtures keep the heading owner inert unless explicitly enabled. */
#include "station_heading.h"
#include <math.h>
#include <stddef.h>
#ifdef MISSION_TEST_HAS_GRAB_TASK
#include "GrabTask.h"
#endif
unsigned test_heading_starts,test_heading_processes,test_heading_pauses,test_heading_competition_starts;
static float test_heading_speed;
uint8_t test_heading_source;
bool test_heading_done;
float test_heading_image;
const char *test_heading_result;
static bool observation;
void StationHeading_Reset(void){test_heading_source=STATION_HEADING_RING;test_heading_starts=test_heading_processes=test_heading_pauses=test_heading_competition_starts=0;test_heading_speed=NAN;test_heading_done=false;observation=false;test_heading_image=.5f;test_heading_result=NULL;}
void StationHeading_Begin(unsigned s,float yaw,float speed){(void)s;(void)yaw;test_heading_speed=speed;test_heading_starts++;}
void StationHeading_ObserveBegin(unsigned s,float yaw,float speed){StationHeading_Begin(s,yaw,speed);observation=true;test_heading_done=false;}
void StationHeading_BeginSource(unsigned s,float y,float v,uint8_t source){StationHeading_Begin(s,y,v);test_heading_source=source;}
void StationHeading_ObserveBeginSource(unsigned s,float y,float v,uint8_t source){StationHeading_ObserveBegin(s,y,v);test_heading_source=source;}
void StationHeading_CompetitionBeginSource(unsigned s,float y,float v,uint8_t source){StationHeading_BeginSource(s,y,source==STATION_HEADING_RING?fminf(v,10):v,source);test_heading_competition_starts++;
#ifdef MISSION_TEST_HAS_GRAB_TASK
  test_heading_done=false;
#endif
}
float StationHeading_TurnSpeed(void){return test_heading_speed;}
const char *StationHeading_Method(void){return test_heading_competition_starts && test_heading_source==STATION_HEADING_RING?"direct_ring":observation?"observe":"mapped";}
uint8_t StationHeading_Source(void){return test_heading_source;}
void StationHeading_Pause(void){test_heading_pauses++;}
bool StationHeading_Process(void){test_heading_processes++;
#ifdef MISSION_TEST_HAS_GRAB_TASK
  /* Mirror the production H_PREPARE wait, including actual fresh arm reads.
   * Vision/yaw themselves are covered by station_heading's dedicated tests. */
  if(!GrabTask_HeadingReady())return false;
  test_heading_done=true;
#endif
  return test_heading_done;
}
const char *StationHeading_Reason(void){return test_heading_done?(test_heading_result?test_heading_result:observation?"heading_observed":"heading_uncalibrated"):"heading_observing";}
float StationHeading_ImageAngle(void){return test_heading_image;}
float StationHeading_BodyError(void){return NAN;}

float StationHeading_ObservationZ(void){return 0;}

/* Hardware seams for fixtures that do not link the real driver modules. */
#ifndef MISSION_TEST_HAS_GRAB_TASK
unsigned test_station_vision_prepares,test_station_vision_cancels;
uint8_t test_station_vision_target;
bool GrabTask_StationVisionPrepare(uint8_t target)
{test_station_vision_prepares++;test_station_vision_target=target;return true;}
void GrabTask_StationVisionCancel(void){test_station_vision_cancels++;}
bool GrabTask_HeadingReady(void){return true;}
extern bool GrabTask_AlignPlaceStartEx(const char *,uint8_t,bool);
bool GrabTask_AlignPlaceStartForSlot(const char *s,uint8_t target,bool raised,uint8_t slot)
{(void)slot;return GrabTask_AlignPlaceStartEx(s,target,raised);}
bool GrabTask_PrepareProcessedTray3(void){return true;}
bool GrabTask_OwnsTrayGoal(void){return false;}
#endif
#ifndef MISSION_TEST_HAS_MOTION
bool ChassisMotion_TurnToPrecision(float y,float s,float t,uint32_t ms){(void)y;(void)s;(void)t;(void)ms;return false;}
#endif
