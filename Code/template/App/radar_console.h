/** @file radar_console.h @brief Radar ASCII commands and paced snapshot pages. */
#ifndef RADAR_CONSOLE_H
#define RADAR_CONSOLE_H
#include <stdbool.h>
#include <stdint.h>
void RadarConsole_Init(void);
/** @brief Main-loop only, after scan/motion processing; never starts movement. */
void RadarConsole_Process(void);
bool RadarConsole_Command(unsigned count,char *tokens[]);
bool RadarConsole_ScanBusy(void);
#endif
