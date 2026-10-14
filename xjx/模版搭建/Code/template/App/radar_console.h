/** @file radar_console.h @brief Radar ASCII commands and paced snapshot pages. */
#ifndef RADAR_CONSOLE_H
#define RADAR_CONSOLE_H
#include <stdbool.h>
#include <stdint.h>
void RadarConsole_Init(void);
/** @brief Main-loop only, after scan/motion processing; never starts movement. */
void RadarConsole_Process(void);
bool RadarConsole_Command(unsigned count,char *tokens[]);
/** `radar host ...`由独立服务处理，不改变实车地图、导航或旧分页状态。 */
/** @brief Static/debug capture or continuous live owns the stationary UART. */
bool RadarConsole_ScanBusy(void);
#endif
