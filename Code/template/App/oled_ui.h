#ifndef OLED_UI_H
#define OLED_UI_H

#include "key.h"

/** @brief 初始化默认总览页。I2C须已初始化；OLED失败不阻止其他任务。 */
void OledUi_Init(I2C_HandleTypeDef *I2c);
/** @brief 更新已验证的15字符任务码及第二组选择下标（0~2）；NULL表示未收到。 */
void OledUi_TaskCodeSet(const char *TaskCode, uint8_t SelectedIndex);
/** @brief 主循环一次取键后调用。返回1表示已消费（翻页或非总览业务键）。 */
uint8_t OledUi_KeyHandle(KeyEvent_t Key);
/** @brief 设置启动拒绝原因短标签；NULL清除提示。任务错误优先保留。 */
void OledUi_NoticeSet(const char *Reason);
/** @brief 非消费式读取缓存，每200ms生成显示；每次最多发送一行。 */
void OledUi_Process(void);

#endif /* OLED_UI_H */
