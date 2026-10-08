// Generated from Code/template/App/grab_parameters.def; do not edit.
// Defaults include values installed after verified homing; not device readback.
export const GRAB_PARAMETER_METADATA = Object.freeze({
  "x_ppm": {
    "min": 0.0,
    "max": 100000.0,
    "step": "any",
    "exclusiveMin": true
  },
  "z_ppm": {
    "min": 0.0,
    "max": 100000.0,
    "step": "any",
    "exclusiveMin": true
  },
  "x_min": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "x_max": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "z_min": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "z_max": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "x_pre": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "z_observe": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "z_grab": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "z_lift": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "pos_tol": {
    "min": 0.0,
    "max": 100.0,
    "step": "any",
    "exclusiveMin": true
  },
  "stop_speed": {
    "min": 0.0,
    "max": 100.0,
    "step": "any",
    "exclusiveMin": true
  },
  "close_ms": {
    "min": 1.0,
    "max": 120000.0,
    "step": 1
  },
  "age_ms": {
    "min": 50.0,
    "max": 5000.0,
    "step": 1
  },
  "feedback_ms": {
    "min": 50.0,
    "max": 5000.0,
    "step": 1
  },
  "accel": {
    "min": 0.0,
    "max": 1000.0,
    "step": "any",
    "exclusiveMin": true
  },
  "travel_mm": {
    "min": 0.0,
    "max": 10000.0,
    "step": "any"
  },
  "lead_mm": {
    "min": 0.0,
    "max": 10000.0,
    "step": "any",
    "exclusiveMin": true
  },
  "dx_tol": {
    "min": 0.0,
    "max": 100.0,
    "step": "any",
    "exclusiveMin": true
  },
  "dy_tol": {
    "min": 0.0,
    "max": 100.0,
    "step": "any",
    "exclusiveMin": true
  },
  "ref_u": {
    "min": 0.0,
    "max": 639.0,
    "step": 1
  },
  "ref_v": {
    "min": 0.0,
    "max": 479.0,
    "step": 1
  },
  "j00": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "j01": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "j02": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "j10": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "j11": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "j12": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "lambda": {
    "min": 0.001,
    "max": 10.0,
    "step": "any"
  },
  "body_speed": {
    "min": 0.0,
    "max": 300.0,
    "step": "any",
    "exclusiveMin": true
  },
  "x_speed": {
    "min": 0.0,
    "max": 100.0,
    "step": "any",
    "exclusiveMin": true
  },
  "z_speed": {
    "min": 0.0,
    "max": 333.333,
    "step": "any",
    "exclusiveMin": true
  },
  "stable_ms": {
    "min": 200.0,
    "max": 120000.0,
    "step": 1
  },
  "frames": {
    "min": 3.0,
    "max": 100.0,
    "step": 1
  },
  "retries": {
    "min": 0.0,
    "max": 10.0,
    "step": 1
  },
  "x_weight": {
    "min": 0.01,
    "max": 100.0,
    "step": "any"
  },
  "z_place": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "loss_grace_ms": {
    "min": 0.0,
    "max": 300.0,
    "step": 1
  },
  "z_proc_grab": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "z_proc_lift": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "z_car_lift": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "z_proc_place": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "z_temp_place": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "z_stack_place": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "x_car": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "base_car_offset": {
    "min": -360.0,
    "max": 360.0,
    "step": "any"
  },
  "vision_wait_ms": {
    "min": 1500.0,
    "max": 120000.0,
    "step": 1
  },
  "x_transfer_speed": {
    "min": 0.0,
    "max": 100.0,
    "step": "any",
    "exclusiveMin": true
  },
  "z_pick_speed": {
    "min": 0.0,
    "max": 333.333,
    "step": "any",
    "exclusiveMin": true
  }
});
export const GRAB_FORM_DEFAULTS = Object.freeze({
  "x_ppm": 31.746031746,
  "z_ppm": 480.0,
  "x_min": 0.0,
  "x_max": 100.0,
  "z_min": -150.0,
  "z_max": 0.0,
  "x_pre": 0.0,
  "z_observe": 0.0,
  "z_grab": -80.0,
  "z_lift": -40.0,
  "pos_tol": 1.0,
  "stop_speed": 2.0,
  "close_ms": 500.0,
  "age_ms": 1000.0,
  "feedback_ms": 2000.0,
  "accel": 300.0,
  "travel_mm": 0.0,
  "lead_mm": 12.0,
  "dx_tol": 1.0,
  "dy_tol": 1.0,
  "ref_u": 334.0,
  "ref_v": 338.0,
  "j00": 2.0,
  "j01": 0.0,
  "j02": 0.0,
  "j10": 0.0,
  "j11": 2.0,
  "j12": 2.0,
  "lambda": 2.5,
  "body_speed": 160.0,
  "x_speed": 60.0,
  "z_speed": 55.55555556,
  "stable_ms": 200.0,
  "frames": 3.0,
  "retries": 2.0,
  "x_weight": 2.333333,
  "z_place": -60.0,
  "loss_grace_ms": 250.0,
  "z_car_lift": 0.0,
  "x_car": 100.0,
  "base_car_offset": -180.0,
  "vision_wait_ms": 12000.0,
  "x_transfer_speed": 60.0,
  "z_pick_speed": 111.11111111
});
