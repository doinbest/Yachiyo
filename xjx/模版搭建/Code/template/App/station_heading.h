#ifndef STATION_HEADING_H
#define STATION_HEADING_H
#include <stdbool.h>
#include <stdint.h>
/* Independent of material acquisition/recovery waits. */
#define STATION_HEADING_WAIT_MS 5000U
#define STATION_HEADING_IMAGE_SPREAD_DEG 0.3f
typedef struct {
  float zero_deg,gain,tolerance_deg,spread_deg,settle_ms,turn_limit_deg;
  uint32_t wait_ms;
  float observation_z_mm; /* Actual feedback, never the nominal clearance height. */
  bool height_matches; /* Calibration applies only at its configured observation height. */
} StationHeading_Config_t;
void StationHeading_Reset(void);
void StationHeading_Begin(unsigned station,float nominal_deg,float speed_deg_s);
/** Measurement only, even when calibration is present; never changes map or turns. */
void StationHeading_ObserveBegin(unsigned station,float nominal_deg,float speed_deg_s);
/** Does not stop motors: mission's existing pause/cancel chain owns physical stopping. */
void StationHeading_Pause(void);
/** True only when this opportunity is finished; a known outstanding deviation never times out as success. */
bool StationHeading_Process(void);
const char *StationHeading_Reason(void);
float StationHeading_ImageAngle(void);
float StationHeading_BodyError(void);
float StationHeading_ObservationZ(void);
#endif
