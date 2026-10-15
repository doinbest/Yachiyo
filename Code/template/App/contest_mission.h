/** @file contest_mission.h @brief 后向机械臂两批搬运任务，显式启动，主循环推进。 */
#ifndef CONTEST_MISSION_H
#define CONTEST_MISSION_H
#include <stdbool.h>
#include "radar_map.h"
#include "motor_bus.h"
#include "chassis_motion.h"
/** Cached failure from the current episode, including post-recovery stop checks. */
bool ContestMission_BusRecoveryEvidenceGet(MotorBus_Recovery_t *out);
void ContestMission_Init(void);
typedef enum { MISSION_CODE_QR, MISSION_CODE_FIXED } ContestMission_CodeSource_t;
typedef struct {
  ContestMission_CodeSource_t code_source;
  char fixed_code[16];
  float route_speed_mm_s, turn_speed_deg_s;
  RadarStart_Id_t start_id;
  bool radar_enabled;
  bool align_raised; /* Station vision at absolute Z=0; false uses configured alignment heights. */
  bool visual_heading_enabled; /* Locked per match; default false. */
  ChassisMotion_Profile_t profile; /* Locked per match; zero defaults to linear. */
} ContestMission_Request_t;
bool ContestMission_Start(const RadarPlan_t *plan,const ContestMission_Request_t *request);
/** @brief 右下至原料台，蓝色入第一仓并后向收回；不改二维码。 */
bool ContestMission_StartSingle(float speed);
/** Complete the currently aligned ring using a selected tray; no route/QR/inventory initialization. */
bool ContestMission_StartRingTake(uint8_t slot);
/** Independent test after grab pose work; 2=processing, 3=temporary, 1..10 deg/s. */
bool ContestMission_StartHeadingTest(unsigned station,float speed_deg_s);
bool ContestMission_StartHeadingTestSource(unsigned station,float speed_deg_s,uint8_t source);
/** Single turn by received image angle times polarity (+1/-1), at 5..10 deg/s. Console default: -1. */
bool ContestMission_StartHeadingProbe(unsigned station,float polarity,float speed_deg_s);
bool ContestMission_StartHeadingProbeSource(unsigned station,float polarity,float speed_deg_s,uint8_t source);
void ContestMission_Process(void);
void ContestMission_Stop(void);
/** End this match after feedback-confirmed stopping; no stage can be resumed. */
void ContestMission_Cancel(void);
bool ContestMission_IsBusy(void);
bool ContestMission_HasRun(void);
bool ContestMission_HasPendingWork(void);
bool ContestMission_OwnsDisplay(void);
/** @brief 最近启动/推进原因，供控制台显示具体缺项或拒绝原因。 */
const char *ContestMission_Reason(void);
void ContestMission_Report(void);
/** @brief 参数缺项时原地等待；补齐当前取放场景参数后显式续作，保留物料进度。 */
bool ContestMission_Continue(void);
/** Preserve a live match, recover transport, then verify stop. Manual calls wait for Continue. */
bool ContestMission_RequestBusRecovery(bool auto_resume);
bool ContestMission_CanConfigure(void);
#endif
