#ifndef CHASSIS_HEADING_H
#define CHASSIS_HEADING_H
#include <stdbool.h>
/** Main-loop atomic map heading correction. No IMU reset, XY/origin/generation change.
 * Caller supplies a qualified static observation; fresh stationary feedback is checked here. */
bool ChassisHeading_Apply(float map_yaw_rad);
#endif
