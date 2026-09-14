#ifndef HWT101_STORE_H
#define HWT101_STORE_H
#include "main.h"
#include <stdbool.h>

typedef struct
{
  const char *state;
  bool busy, has_saved;
  uint32_t sequence;
  float bias_dps;
  HAL_StatusTypeDef hal;
} HWT101_StoreStatus_t;

/** @brief 启动Flash记录加载；SPI已初始化。末尾两个4KiB扇区专用于IMU。 */
void HWT101_Store_Init(SPI_HandleTypeDef *hspi);
/** @brief 主循环在Drift_Process之后调用；轮询擦写，不等待Flash内部操作。 */
void HWT101_Store_Process(void);
/** @brief 异步写入删除记录；HAL_OK表示开始，state=forgotten才表示完成。
 * @note 忙时返回HAL_BUSY；不整片擦除。 */
HAL_StatusTypeDef HWT101_Store_Forget(void);
/** @brief 取消本次启动待执行的恢复，不删除Flash记录。 */
void HWT101_Store_CancelRestore(void);
/** @brief 只读存储状态，非空输出地址。 */
void HWT101_Store_StatusGet(HWT101_StoreStatus_t *status);
#endif
