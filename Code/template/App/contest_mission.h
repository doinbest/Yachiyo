/** @file contest_mission.h @brief 后向机械臂两批搬运任务，显式启动，主循环推进。 */
#ifndef CONTEST_MISSION_H
#define CONTEST_MISSION_H
#include <stdbool.h>
#include "radar_map.h"
void ContestMission_Init(void);
bool ContestMission_Start(const RadarPlan_t *plan,float speed);
/** @brief 右下至原料台，蓝色入第一仓并后向收回；不改二维码。 */
bool ContestMission_StartSingle(float speed);
void ContestMission_Process(void);
void ContestMission_Stop(void);
bool ContestMission_IsBusy(void);
bool ContestMission_HasRun(void);
/** @brief 最近启动/推进原因，供控制台显示具体缺项或拒绝原因。 */
const char *ContestMission_Reason(void);
void ContestMission_Report(void);
/** @brief 参数缺项时原地等待；补齐当前取放场景参数后显式续作，保留物料进度。 */
bool ContestMission_Continue(void);
bool ContestMission_CanConfigure(void);
#endif
