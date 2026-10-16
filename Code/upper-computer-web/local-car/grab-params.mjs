// Default values and bounds are generated from the firmware parameter definition.
import {GRAB_FORM_DEFAULTS} from './grab-parameter-metadata.mjs';
export {GRAB_FORM_DEFAULTS};

// Only these tuning keys are sent when the operator applies the debug preset.
export const GRAB_TEST_PRESET = Object.freeze(Object.fromEntries(
  ['travel_mm','body_speed','accel','x_speed','z_speed','z_pick_speed','qr_z_speed','vision_wait_ms','lambda','loss_grace_ms']
    .map(key=>[key,GRAB_FORM_DEFAULTS[key]])
));
