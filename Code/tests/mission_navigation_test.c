#include "mission_navigation.h"
#include "radar_scan.h"
#include "lidar_servo.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t tick;
static RadarScan_Status_t scan;
static RadarMap_Params_t params;
static RadarMap_t map;
static unsigned scans,stops;static uint16_t pulse;static bool route_ok=true;
uint32_t HAL_GetTick(void){return tick;}
HAL_StatusTypeDef LidarServo_PulseSet(uint16_t p){pulse=p;return HAL_OK;}
void RadarScan_StatusGet(RadarScan_Status_t *s){*s=scan;}
const RadarMap_Params_t *RadarScan_Params(void){return &params;}
const RadarMap_Params_t *RadarScan_MapParams(void){return &params;}
const RadarMap_t *RadarScan_Map(void){return &map;}
void RadarScan_Stop(void){stops++;scan.state=RADAR_SCAN_STOPPED;}
bool RadarScan_StartAtCycles(const RadarMap_Params_t *p,uint32_t g,RadarStart_Id_t id,uint8_t circles)
{(void)p;assert(circles==15);scans++;scan.state=RADAR_SCAN_COLLECTING;scan.origin_generation=g;params.start_id=id;return true;}
bool RadarPlan_BuildFromStart(const RadarMap_t *m,RadarStart_Id_t id,RadarPlan_t *p)
{assert(m==&map);p->valid=route_ok;p->start_id=id;return route_ok;}
static RadarPlan_t plan;static const char *reason;
static int step(RadarStart_Id_t id){return MissionNavigation_Process(id,7,&plan,&reason);}
int main(void){
 for(unsigned id=1;id<=2;id++){
  memset(&scan,0,sizeof scan);MissionNavigation_Reset();tick=0;
  assert(!step(id) && pulse==2450);tick=999;assert(!step(id));tick=1000;assert(!step(id) && scan.state==RADAR_SCAN_COLLECTING);
  scan.state=RADAR_SCAN_READY;scan.map_valid=scan.map_applicable=true;scan.map_id++;
  assert(!step(id) && pulse==1450);tick=2000;assert(step(id)==1 && plan.start_id==id);
 }
 MissionNavigation_Reset();tick=3000;assert(!step(2));tick=4000;assert(!step(2));
 MissionNavigation_Pause();assert(stops && scan.state==RADAR_SCAN_STOPPED);assert(!step(2));tick=5000;assert(!step(2));
 scan.state=RADAR_SCAN_READY; /* cached map cannot pass */
 assert(step(2)==0 || !strcmp(reason,"radar_scan_failed"));tick=6000;assert(step(2)==-1);
 MissionNavigation_Reset();tick=7000;assert(!step(1));tick=8000;assert(!step(1));
 scan.state=RADAR_SCAN_READY;scan.map_id++;route_ok=false;
 (void)step(1);tick=9000;assert(step(1)==-1 && !strcmp(reason,"radar_no_route"));
 puts("mission navigation: both starts, fresh maps, stow, pause/retry, no route PASS");return 0;
}
