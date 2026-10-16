/** @file GrabTask.h
 * @brief B2协同对准及单件外部/车载仓位取放，主循环串行推进并确认实际反馈。
 */
#ifndef GRAB_TASK_H
#define GRAB_TASK_H
#include "mechanical_arm.h"
#include "Camera.h"
#include "grab_tracking.h"
#include "station_heading.h"
#include <stdbool.h>

/* Shared by camera requests and the read-only firmware capability report. */
#define GRAB_ALIGN_COLOR CAMERA_COLOR_BLUE
#define GRAB_PICK_COLOR CAMERA_COLOR_RED

typedef enum
{
  GRAB_IDLE, GRAB_PREPARE, GRAB_ACQUIRE, GRAB_ALIGN, GRAB_SETTLE,
  GRAB_DESCEND, GRAB_CLOSE, GRAB_LIFT, GRAB_TURN, GRAB_PLACE, GRAB_RELEASE, GRAB_RETRACT, GRAB_HOLD, GRAB_STOPPING, GRAB_ERROR, GRAB_HOMING, GRAB_VISION_PAUSE, GRAB_VISION_GRACE,
  GRAB_X_RETRACT, GRAB_INDEX, GRAB_CAR_EXTEND, GRAB_RETURN_TURN,
  GRAB_EXTERNAL_EXTEND, GRAB_OBSERVE_Z, GRAB_OBSERVE_X, GRAB_COMPLETE,
  GRAB_TRACK_OBSERVE, GRAB_TRACK_MATCH, GRAB_TRACK_FOLLOW, GRAB_TRACK_PAUSE,
  GRAB_ROUTE_POSE, GRAB_MANUAL_Z, GRAB_CALIBRATION, GRAB_QR_SCAN
} GrabTask_State_t;
typedef struct
{
  GrabTask_State_t state;
  const char *state_name, *mode, *reason, *missing;
  const char *reference_cause; /**< Latest cause that invalidated or verified pickup references. */
  const char *operation, *scene, *result; /**< align/store/take/return; action scene; software result. */
  uint8_t slot; /**< Onboard slot 1..3, or 0 for alignment/return. */
  bool busy, stop_requested, stop_confirmed;
  bool base_home_degraded; /**< Approximate rear anchor accepted after a failed Base HOME. */
  float x_mm, x_target_mm, z_mm, z_target_mm, forward_mm_s, left_mm_s;
  int16_t dx, dy;
  uint32_t rx_seq; /**< MCU receive sequence, not camera capture sequence. */
  const char *vision_state;
  uint32_t age_ms, elapsed_ms;
  uint32_t recovery_used; /**< Successful automatic recoveries in this task; diagnostic only. */
  uint32_t recovery_count; /**< Consecutive new valid observations in the recovery window, 0..3. */
  int32_t recovery_left_ms; /**< -1 before standstill/outside recovery; MCU time only. */
  uint32_t loss_ms; /**< Active interruption or last completed one, measured from last good RX; ms. */
  uint32_t loss_max_ms; /**< Longest interruption in this task, including recovery confirmation; ms. */
  uint32_t grace_count; /**< New valid observations during short-loss tolerance, 0..3. */
} GrabTask_Status_t;

/** @brief 初始化RAM标定缺项；仅启动时调用，不用于停车。 */
void GrabTask_Init(void);
/** QR test readiness/config only; no motion or reference change. */
bool GrabTask_QrReady(float *scan_y);
/** After chassis-confirmed arrival: wait, then move Z only until new UART code. */
bool GrabTask_QrScanStart(uint32_t initial_sequence);
bool GrabTask_QrScanCompetitionStart(uint32_t initial_sequence);
/* Competition-only handoff: leave QR search without stopping chassis/ID8. */
bool GrabTask_QrDepartStart(void);
bool GrabTask_QrDepartureActive(void);
/* Schedule recognition of the next raw item while this item returns. */
void GrabTask_NextRawColorSet(uint8_t color);
/* Competition preparation only: B2 target 1..6 or 10..12, no actuator action.
 * Matching preparation is adopted once, requiring NEW images after pose ready. */
bool GrabTask_StationVisionPrepare(uint8_t target);
void GrabTask_StationVisionCancel(void);
/** @brief Launch Z/X collision and Base near HOME without waiting for other axes to finish.
 * Completion requires 0x3B flags and feedback: X/Z zero + reached, Base stable + enabled.
 * Base near-home does not imply numerical zero. Boot also waits for ID8's independent HOME
 * before arming IMU verification; explicit grab rehome only establishes the three arm references.
 */
void GrabTask_BootHomeStart(void);
/** @brief 持续主循环调用；align/pick共用短漏检容忍与停稳恢复。
 * 容忍期间只按上次速度减速，不沿旧误差加速或判定对准；停稳后重发一次B2。
 */
void GrabTask_Process(void);
/** @brief fixed/align/pick；返回受理，不等于到位或抓持确认。必须先排除其他任务。 */
bool GrabTask_Start(const char *mode);
/** @brief 统一静态对准入口：B2颜色1..6或编号环10..12，Z绝对高度mm。
 * 先横向接近、按新坐标渐进下降；stay保持低位，否则停止视觉控制后抬至最高。
 * 不夹取、不操作车载转盘；完成后允许人工夹爪及独立高度调整。
 */
bool GrabTask_AlignStart(uint8_t target,float z_mm,bool stay);
/** @brief 人工确认当前观察姿态；确认由统一任务保存，停止不清除，不执行运动。
 * 手动Base后的参考策略沿用延期P001，不在本次收敛中另行改变。 */
bool GrabTask_CalibrationReferenceSet(void);
bool GrabTask_CalibrationReferenceValid(void);
/** @brief Start forward/X sampling in the manually confirmed pose; no Base/Z/grip reposition. */
bool GrabTask_CalibrationStart(uint8_t color);
/** @brief Only the sampling task owns actuators; completed/error records are inactive. */
bool GrabTask_CalibrationActive(void);
/** @brief 原子应用前后与X响应到共用J；X输入像素/电机度，侧向列保留。 */
bool GrabTask_VisualResponseSet(float forward_dx,float forward_dy,float x_dx,float x_dy);
/** @brief 只读检查所选模式标定完整性和数值/机械边界，不访问硬件。 */
bool GrabTask_ConfigReady(const char *mode);
#define GRAB_TASK_POSE 1
/** @brief 从后向锚点切换姿态；按新鲜停稳反馈补齐抬Z/收X/转Base，work为后向参考+117.5°。
 * 手动Base就近回零已受理时，先局部验证该轴参考，再衔接准备；停止取消衔接。
 * @return 受理；work_ready前打开夹爪2450us；travel_ready保持夹爪，不重新回零。
 */
bool GrabTask_PoseStart(bool work);
/** Fresh X-retracted/Z-lifted proof during a pose switch; Base may still move.
 * Includes QR pose. Does not mean the complete pose is ready; false while paused or in other tasks. */
bool GrabTask_PoseClearanceReady(void);
void GrabTask_HeadingConfigGet(unsigned station,StationHeading_Config_t *out);
void GrabTask_LineHeadingConfigGet(StationHeading_Config_t *out);
/** Refresh stationary Base/X/Z evidence without moving or opening the claw. */
bool GrabTask_HeadingReady(void);
/** Match QR approach: retracted arm, Base clockwise +35 deg from rear reference. */
bool GrabTask_QrPoseStart(void);
/** @brief 任务内部单件取放；operation=store/take，scene=raw/proc/rough/temp1/temp2。 */
bool GrabTask_TransferStart(const char *operation,const char *scene,uint8_t slot,uint8_t color);
/** @brief HOME/ZERO等参考变化通知：X/Z清除绝对边界/位置，BASE清除J及参考；
 * ALL仅清除X/Z，all home调用者另通知BASE；不自动重标定或改变硬件。
 * 手动Base位置运动不调用：保留锚点/J，作业姿态核验完成后再允许视觉任务。
 */
void GrabTask_ReferenceInvalidate(MechanicalArm_AxisTypeDef axis);
/** @brief align HOLD仍拥有机构；单件COMPLETE释放占用，可显式开始下一件。 */
bool GrabTask_IsBusy(void);
/** @brief 兼容命名：所有静态对准的停稳HOLD允许人工夹爪、独立调高/松爪。 */
bool GrabTask_RingManualAllowed(void);
/** Adopt a completed empty-claw numbered-ring test for fresh capture and automatic tray placement. */
bool GrabTask_RingTakePrepare(uint8_t *ring);
bool GrabTask_RingTakePrepareAtZ(uint8_t *ring,float release_z_mm);
/* Independent full ring flow: validate empty claw and this request release Z. */
bool GrabTask_RingFlowReady(float release_z_mm);
bool GrabTask_AlignPlaceStartAtZForSlot(const char *scene,uint8_t target,uint8_t slot,float release_z_mm);
/** @brief 取消自动推进并请求停车；重复请求保留停稳证据，保持夹爪输出与Z使能，检查状态确认停车。 */
void GrabTask_Stop(void);
/** Cancel an owned task and discard its paused continuation; preserve claw output. */
void GrabTask_Cancel(void);
/** Competition pause preserves the exact servo/axis phase; continue is explicit. */
bool GrabTask_Pause(void);
/** Rebuild post-recovery physical stop evidence without replacing the resume stage. */
bool GrabTask_BusRecoveryVerify(void);
bool GrabTask_Continue(void);
bool GrabTask_TrackedStoreStart(uint8_t color,uint8_t slot,GrabTracking_Mode_t mode);
/** Select travel posture as the end of the newly accepted storage task.
 * Preserved by mission pause; standalone storage still returns to observation. */
bool GrabTask_StoreFinishTravel(void);
bool GrabTask_CarPickStart(uint8_t slot);
/* One-request depth; lift returns to the same captured upper reference. */
bool GrabTask_CarPickStartEx(uint8_t slot,float drop_mm);
/* Standalone ring run/take: parallel empty-claw lift/retraction, then wait
 * for lifted Z and retracted X before turning with material (no X30). NAN depth
 * selects the configured car_pick_drop; accepted goals survive pause/resume. */
bool GrabTask_RingCarPickStart(uint8_t slot,float drop_mm);
/* Current aligned work pose -> pickup -> selected tray -> X30 -> work pose. */
bool GrabTask_RingStoreStart(uint8_t slot,float pick_drop_mm,float car_drop_mm);
bool GrabTask_HeldPlaceStart(const char *scene,uint8_t target,uint8_t material_color);
/** rough/temp1 use ring 1..3; temp2 uses first-layer material color 1..6.
 * Empty-claw alignment captures actual X/Z and the scene release target,
 * then pick from the tray and place without another vision request. */
bool GrabTask_AlignPlaceStart(const char *scene,uint8_t target);
bool GrabTask_AlignPlaceCapture(void);
bool GrabTask_AlignedPlaceStart(const char *scene,uint8_t target,uint8_t material_color);
bool GrabTask_ProcessedStoreStart(uint8_t color,uint8_t ring,uint8_t slot);
/** Per-task station alignment choice: raised uses absolute Z=0 before final
 * vertical pickup/release. Standalone station entry points also default to raised alignment. */
bool GrabTask_HeldPlaceStartEx(const char *scene,uint8_t target,uint8_t material_color,bool raised);
bool GrabTask_AlignPlaceStartEx(const char *scene,uint8_t target,bool raised);
/* Competition supplies the tray before alignment; legacy alignment has no implicit tray. */
bool GrabTask_AlignPlaceStartForSlot(const char *scene,uint8_t target,bool raised,uint8_t slot);
bool GrabTask_PrepareProcessedTray3(void);
bool GrabTask_OwnsTrayGoal(void);
bool GrabTask_ProcessedStoreStartEx(uint8_t color,uint8_t ring,uint8_t slot,bool raised);
/** @brief 消费本任务发起的机械臂事件；非本任务返回0。 */
uint8_t GrabTask_MotorEventHandle(const MechanicalArm_EventTypeDef *event);
/** @brief 主循环只读状态快照；距离mm、速度mm/s、年龄/耗时ms。 */
void GrabTask_StatusGet(GrabTask_Status_t *status);
/** @brief 空格ASCII grab命令，最多6字段/63字节；返回是否属于本模块。 */
bool GrabTask_Command(unsigned count, char *tokens[]);
#endif
