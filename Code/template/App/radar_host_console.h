/**
 * @file radar_host_console.h
 * @brief F1兼容网页雷达服务的有界USART1命令与分页输出。
 */
#ifndef RADAR_HOST_CONSOLE_H
#define RADAR_HOST_CONSOLE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void RadarHostConsole_Init(void);
/** @brief 主循环调用；一次最多提交一个完整的低优先级页面。 */
void RadarHostConsole_Process(void);
/** @return 仅当命令属于精确的`radar host`命名空间时为true。 */
bool RadarHostConsole_Command(unsigned count,char *tokens[]);

#ifdef __cplusplus
}
#endif
#endif
