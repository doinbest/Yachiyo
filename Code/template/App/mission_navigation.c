#include "mission_navigation.h"
#include "radar_scan.h"
#include "lidar_servo.h"
static unsigned Step;
static uint32_t Tick,MapBaseline;
static const char *Failure;
void MissionNavigation_Reset(void){Step=0;Failure=0;}
void MissionNavigation_Pause(void)
{
  if(Step==1 || Step==2){RadarScan_Stop();Step=0;}
}
int MissionNavigation_Process(RadarStart_Id_t start,uint32_t generation,RadarPlan_t *plan,const char **reason)
{
  RadarScan_Status_t s;uint32_t now=HAL_GetTick();
  if(Failure){*reason=Failure;return -1;}
  if(!Step){
    RadarScan_StatusGet(&s);
    if(s.state==RADAR_SCAN_PREPARING || s.state==RADAR_SCAN_WAIT_WRAP || s.state==RADAR_SCAN_COLLECTING){*reason="radar_busy";return 0;}
    if(LidarServo_PulseSet(LIDAR_SERVO_SCAN_PULSE_US)!=HAL_OK){*reason="radar_servo_failed";return -1;}
    Tick=now;Step=1;
  }
  if(Step==1){
    *reason="radar_deploying";if(now-Tick<1000)return 0;
    RadarScan_StatusGet(&s);MapBaseline=s.map_id;
    if(!RadarScan_StartAtCycles(RadarScan_Params(),generation,start,15)){*reason="radar_start_wait";return 0;}
    Step=2;
  }
  if(Step==2){
    RadarScan_StatusGet(&s);*reason="radar_scanning";
    if(s.state==RADAR_SCAN_PREPARING || s.state==RADAR_SCAN_WAIT_WRAP || s.state==RADAR_SCAN_COLLECTING)return 0;
    if(s.state!=RADAR_SCAN_READY || !s.map_valid || !s.map_applicable || s.map_id==MapBaseline || s.origin_generation!=generation)
      Failure="radar_scan_failed";
    else if(!RadarScan_MapParams() || RadarScan_MapParams()->start_id!=start ||
            !RadarPlan_BuildFromStart(RadarScan_Map(),start,plan))Failure="radar_no_route";
    if(LidarServo_PulseSet(LIDAR_SERVO_STOW_PULSE_US)!=HAL_OK)Failure="radar_servo_failed";
    Tick=now;Step=3;
  }
  if(Step==3){*reason="radar_stowing";if(now-Tick<1000)return 0;Step=4;}
  if(Failure){*reason=Failure;return -1;}
  *reason="radar_plan_ready";return 1;
}
