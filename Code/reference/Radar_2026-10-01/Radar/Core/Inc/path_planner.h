#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "navigation_types.h"

uint8_t PathPlanner_BuildMissionPath(uint32_t blocked_mask,
                                     NavigationResult *result);
uint8_t PathPlanner_BuildMissionPathLimited(uint32_t blocked_mask,
                                            NavigationResult *result,
                                            uint16_t capacity);

#ifdef __cplusplus
}
#endif

#endif
