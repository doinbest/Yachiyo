/** @file turntable.h @brief ID 8 车载三仓转盘；主循环业务接口。 */
#ifndef TURNTABLE_H
#define TURNTABLE_H
#include <stdbool.h>
#include <stdint.h>

#define TURNTABLE_SLOT_COUNT 3U
typedef enum
{
  TURNTABLE_UNKNOWN = 0,
  TURNTABLE_EMPTY,
  TURNTABLE_OCCUPIED
} Turntable_InventoryState_t;
typedef struct
{
  Turntable_InventoryState_t state;
  uint8_t color; /**< B2 颜色 1..6；空仓或待核对时为 0。 */
} Turntable_Inventory_t;
typedef struct
{
  bool reference_valid, arrived, moving, feedback_valid;
  uint8_t slot;
  const char *state, *reason;
  float angle_deg, target_deg, speed_rpm;
} Turntable_Status_t;

/** @brief 上电初始化；三仓库存为待核对，第二/三仓角度未设置。 */
void Turntable_Init(void);
/** @brief 兼容主循环启动调度；上电自动回零已关闭，不查询或驱动ID8。
 * @param arm_ready 三轴回零已验证；当前不触发转盘操作。 */
void Turntable_BootHomeProcess(bool arm_ready);
/** @brief MotorBus_Process 后调用，读取本模块事件并推进反馈确认。 */
void Turntable_Process(void);
bool Turntable_IsBusy(void);
/** @brief 请求 ID8 停止；普通停止保留参考、参数和库存。
 * 总线隔离时只能确认停止帧发送，status 报 stopped_unverified。 */
void Turntable_Stop(void);
/** @brief 单独使能/失能 ID 8；应答后读取状态确认，保留仓位参数、参考及库存。 */
bool Turntable_Enable(bool enabled);
/** @brief 将当前单圈机械零点保存到 ID 8 驱动器，并同步第一仓软件参考。 */
bool Turntable_SaveZero(void);
/** @brief 单圈就近回零，反馈完成后自动同步第一仓参考；不清位置计数。 */
bool Turntable_Home(void);
void Turntable_StatusGet(Turntable_Status_t *out);
bool Turntable_ReferenceValid(void);
bool Turntable_SlotConfigured(uint8_t slot);
/** @brief 人工对齐第一仓后读取当前位置作为软件偏移；不回零/清零。 */
bool Turntable_Origin(void);
/** @brief 最短方向分度；true 仅表示受理，到位检查 StatusGet.arrived。 */
bool Turntable_Index(uint8_t slot);
/** @brief 相对当前位置点动，degree 为度；不要求已建立仓位参考。 */
bool Turntable_Jog(float degree);
/** @brief 返回最小编号的空仓，0 表示没有已确认的空仓。 */
uint8_t Turntable_FindEmpty(void);
/** @brief 无总线读取；slot 为 1..3，无效仓返回 UNKNOWN。 */
Turntable_Inventory_t Turntable_InventoryGet(uint8_t slot);
/** @brief 记录软件动作结果或人工核对；不表示传感器确认。 */
bool Turntable_InventorySet(uint8_t slot, Turntable_InventoryState_t state, uint8_t color);
/** @brief ASCII 命令 tokens[0]="turntable"；非本模块命令返回 false。 */
bool Turntable_Command(unsigned count, char *tokens[]);
#endif
