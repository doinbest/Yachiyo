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
  "z_stack_place": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "z_rough_align": {
    "min": -10000.0,
    "max": 10000.0,
    "step": "any"
  },
  "z_temp_align": {
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
  },
  "qr_scan_y": {
    "min": 1100.0,
    "max": 1300.0,
    "step": "any"
  },
  "qr_z_min": {
    "min": -150.0,
    "max": 0.0,
    "step": "any"
  },
  "qr_z_speed": {
    "min": 0.0,
    "max": 333.333,
    "step": "any",
    "exclusiveMin": true
  },
  "qr_wait_ms": {
    "min": 0.0,
    "max": 10000.0,
    "step": 1
  },
  "base_home_degrade": {
    "min": 0.0,
    "max": 1.0,
    "step": 1
  },
  "track_pick_tol_px": {
    "min": 1.0,
    "max": 100.0,
    "step": "any"
  },
  "track_pick_hold_ms": {
    "min": 0.0,
    "max": 10000.0,
    "step": 1
  },
  "track_store_drop_mm": {
    "min": 0.0,
    "max": 10000.0,
    "step": "any",
    "exclusiveMin": true
  },
  "car_pick_drop_mm": {
    "min": 0.0,
    "max": 150.0,
    "step": "any",
    "exclusiveMin": true
  },
  "arrival_hold_ms": {
    "min": 200.0,
    "max": 10000.0,
    "step": 1
  },
  "head_proc_zero": {
    "min": -90.0,
    "max": 90.0,
    "step": "any"
  },
  "head_proc_gain": {
    "min": -20.0,
    "max": 20.0,
    "step": "any"
  },
  "head_temp_zero": {
    "min": -90.0,
    "max": 90.0,
    "step": "any"
  },
  "head_temp_gain": {
    "min": -20.0,
    "max": 20.0,
    "step": "any"
  },
  "head_line_zero": {
    "min": -90.0,
    "max": 90.0,
    "step": "any"
  },
  "head_line_gain": {
    "min": -20.0,
    "max": 20.0,
    "step": "any"
  },
  "head_tol": {
    "min": 0.0,
    "max": 2.0,
    "step": "any",
    "exclusiveMin": true
  },
  "head_spread": {
    "min": 0.0,
    "max": 2.0,
    "step": "any",
    "exclusiveMin": true
  },
  "head_settle_ms": {
    "min": 0.0,
    "max": 10000.0,
    "step": 1
  },
  "head_proc_turn": {
    "min": 0.0,
    "max": 10.0,
    "step": "any"
  },
  "head_temp_turn": {
    "min": 0.0,
    "max": 10.0,
    "step": "any"
  },
  "release_ms": {
    "min": 1.0,
    "max": 120000.0,
    "step": 1
  },
  "car_pick_wait_ms": {
    "min": 0.0,
    "max": 10000.0,
    "step": 1
  }
});
export const GRAB_FORM_DEFAULTS = Object.freeze({
  "x_ppm": 31.746031746,
  "z_ppm": 480.0,
  "x_min": 0.0,
  "x_max": 130.0,
  "z_min": -150.0,
  "z_max": 0.0,
  "x_pre": 0.0,
  "z_observe": 0.0,
  "z_grab": -50.0,
  "z_lift": 0.0,
  "pos_tol": 1.0,
  "stop_speed": 2.0,
  "close_ms": 300.0,
  "age_ms": 1000.0,
  "feedback_ms": 2000.0,
  "accel": 300.0,
  "travel_mm": 0.0,
  "lead_mm": 12.0,
  "dx_tol": 1.0,
  "dy_tol": 1.0,
  "j00": 1.422222222222,
  "j01": 0.0,
  "j02": 0.0,
  "j10": 0.0,
  "j11": 1.454545454545,
  "j12": 1.454545454545,
  "lambda": 2.5,
  "body_speed": 160.0,
  "x_speed": 60.0,
  "z_speed": 250.0,
  "stable_ms": 200.0,
  "frames": 3.0,
  "retries": 2.0,
  "x_weight": 2.333333,
  "z_place": -20.0,
  "loss_grace_ms": 250.0,
  "z_car_lift": 0.0,
  "z_proc_place": -120.0,
  "z_stack_place": -110.0,
  "z_rough_align": -100.0,
  "z_temp_align": -100.0,
  "base_car_offset": -117.5,
  "vision_wait_ms": 12000.0,
  "x_transfer_speed": 60.0,
  "z_pick_speed": 250.0,
  "qr_scan_y": 1200.0,
  "qr_z_min": -50.0,
  "qr_z_speed": 250.0,
  "qr_wait_ms": 1000.0,
  "base_home_degrade": 1.0,
  "track_pick_tol_px": 100.0,
  "track_pick_hold_ms": 0.0,
  "track_store_drop_mm": 20.0,
  "car_pick_drop_mm": 40.0,
  "arrival_hold_ms": 200.0,
  "release_ms": 300.0,
  "car_pick_wait_ms": 300.0
});
export const TRACK_PARAMETER_DEFAULTS = Object.freeze({
  "body_speed": 160.0,
  "x_speed": 60.0,
  "accel": 300.0,
  "lambda": 2.5,
  "x_weight": 2.333333,
  "lead_mm": 12.0,
  "tol_px": 5.0,
  "control_ms": 20.0,
  "batch_ms": 0.0,
  "latency_ms": 50.0,
  "fit_ms": 240.0,
  "horizon_ms": 120.0,
  "grace_ms": 250.0,
  "body_accel": 300.0,
  "x_accel": 120.0
});
export const TRACK_COMPARISON_DEFAULTS = Object.freeze({...TRACK_PARAMETER_DEFAULTS, ...{
  "body_speed": 160.0,
  "x_speed": 60.0,
  "lambda": 2.0,
  "horizon_ms": 60.0
}});
export const TRACK_PARAMETER_METADATA = Object.freeze({
  "body_speed": {
    "min": 1.0,
    "max": 300.0,
    "step": "any"
  },
  "x_speed": {
    "min": 1.0,
    "max": 100.0,
    "step": "any"
  },
  "accel": {
    "min": 1.0,
    "max": 1000.0,
    "step": "any"
  },
  "lambda": {
    "min": 0.01,
    "max": 10.0,
    "step": "any"
  },
  "x_weight": {
    "min": 0.01,
    "max": 100.0,
    "step": "any"
  },
  "lead_mm": {
    "min": 0.1,
    "max": 12.0,
    "step": "any"
  },
  "tol_px": {
    "min": 1.0,
    "max": 20.0,
    "step": "any"
  },
  "control_ms": {
    "min": 20.0,
    "max": 100.0,
    "step": 1
  },
  "batch_ms": {
    "min": 0.0,
    "max": 200.0,
    "step": 1
  },
  "latency_ms": {
    "min": 0.0,
    "max": 200.0,
    "step": 1
  },
  "fit_ms": {
    "min": 100.0,
    "max": 500.0,
    "step": 1
  },
  "horizon_ms": {
    "min": 50.0,
    "max": 200.0,
    "step": 1
  },
  "grace_ms": {
    "min": 0.0,
    "max": 250.0,
    "step": 1
  },
  "body_accel": {
    "min": 1.0,
    "max": 1000.0,
    "step": "any"
  },
  "x_accel": {
    "min": 1.0,
    "max": 1000.0,
    "step": "any"
  }
});
