#ifndef MISSION_NAVIGATION_H
#define MISSION_NAVIGATION_H
#include "radar_map.h"
void MissionNavigation_Reset(void);
void MissionNavigation_Pause(void);
/* 0 waiting, 1 frozen plan ready, -1 failed; retry after explicit Reset. */
int MissionNavigation_Process(RadarStart_Id_t start,uint32_t generation,RadarPlan_t *plan,const char **reason);
#endif
