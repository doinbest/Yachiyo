#ifndef NAVIGATION_H
#define NAVIGATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "navigation_port.h"
#include "navigation_types.h"

extern NavigationResult g_navigation_result;

void Navigation_SetPort(const NavigationPort *port);
void Navigation_Init(void);
void Navigation_Start(void);
void Navigation_Poll(void);

#ifdef __cplusplus
}
#endif

#endif
