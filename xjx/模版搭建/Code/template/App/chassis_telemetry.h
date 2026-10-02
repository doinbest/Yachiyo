#ifndef CHASSIS_TELEMETRY_H
#define CHASSIS_TELEMETRY_H
#include <stdbool.h>
#include <stdint.h>
/** @brief Initialize stream state and shared localization once during startup. */
void ChassisTelemetry_Init(void);
/** @brief Emit optional snapshots; call localization separately before motion control. */
void ChassisTelemetry_Process(void);
/** Explicit host nonce; resets trace segment. MCU reset always disables streaming. */
bool ChassisTelemetry_Stream(bool on, uint32_t session);
bool ChassisTelemetry_Origin(float x_mm, float y_mm, float yaw_rad);
bool ChassisTelemetry_ConfirmPositionUnits(uint32_t units_per_rev);
/** Console tokens: handles only the added commands; false lets legacy dispatcher run. */
bool ChassisTelemetry_Command(unsigned count, char *tokens[]);
#endif
