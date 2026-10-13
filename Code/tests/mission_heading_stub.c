/* Legacy mission fixtures keep the heading owner inert unless explicitly enabled. */
#include "station_heading.h"
#include <math.h>
#include <stddef.h>
unsigned test_heading_starts,test_heading_processes,test_heading_pauses;
bool test_heading_done;
float test_heading_image;
const char *test_heading_result;
static bool observation;
void StationHeading_Reset(void){test_heading_starts=test_heading_processes=test_heading_pauses=0;test_heading_done=false;observation=false;test_heading_image=.5f;test_heading_result=NULL;}
void StationHeading_Begin(unsigned s,float yaw,float speed){(void)s;(void)yaw;(void)speed;test_heading_starts++;}
void StationHeading_ObserveBegin(unsigned s,float yaw,float speed){StationHeading_Begin(s,yaw,speed);observation=true;test_heading_done=false;}
void StationHeading_Pause(void){test_heading_pauses++;}
bool StationHeading_Process(void){test_heading_processes++;return test_heading_done;}
const char *StationHeading_Reason(void){return test_heading_done?(test_heading_result?test_heading_result:observation?"heading_observed":"heading_uncalibrated"):"heading_observing";}
float StationHeading_ImageAngle(void){return test_heading_image;}
float StationHeading_BodyError(void){return NAN;}

float StationHeading_ObservationZ(void){return 0;}

/* Hardware seams for fixtures that do not link the real driver modules. */
#ifndef MISSION_TEST_HAS_GRAB_TASK
bool GrabTask_HeadingReady(void){return true;}
#endif
#ifndef MISSION_TEST_HAS_MOTION
bool ChassisMotion_TurnToPrecision(float y,float s,float t,uint32_t ms){(void)y;(void)s;(void)t;(void)ms;return false;}
#endif
