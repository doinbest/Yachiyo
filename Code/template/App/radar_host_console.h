/**
 * @file radar_host_console.h
 * @brief F1兼容网页雷达服务的有界USART1命令与分页输出。
 */
#ifndef RADAR_HOST_CONSOLE_H
#define RADAR_HOST_CONSOLE_H

#include <stdbool.h>
#include <stdint.h>
#include "radar_stability.h"

#ifdef __cplusplus
extern "C" {
#endif

void RadarHostConsole_Init(void);
/** @brief 主循环调用；一次最多提交一个完整的低优先级页面。 */
void RadarHostConsole_Process(void);
/** @return 仅当命令属于精确的`radar host`命名空间时为true。 */
bool RadarHostConsole_Command(unsigned count,char *tokens[]);
/** Shared CPU-only CCM freeze: owner 1 formal, owner 2 host. */
bool RadarStableTx_Freeze(uint8_t owner,uint32_t scan_id,const RadarStablePoint *points,uint16_t count);
const RadarStablePoint *RadarStableTx_Points(uint8_t owner,uint32_t scan_id,uint16_t *count);
void RadarStableTx_Release(uint8_t owner);

#ifdef __cplusplus
}
#endif
#endif
