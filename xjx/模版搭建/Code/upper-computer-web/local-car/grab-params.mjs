// 上位机视觉抓取参数集中区。修改此文件后刷新页面；数值不会自动发给 STM32。
// 预填值供单项表单使用，不是设备读回，也不会修改固件的上电默认值。
export const GRAB_FORM_DEFAULTS = Object.freeze({
  z_speed:27.777778,loss_grace_ms:250,age_ms:1000,accel:20,
  body_speed:20,lambda:0.6,travel_mm:0,lead_mm:2,
  dx_tol:5,dy_tol:5,ref_u:334,ref_v:338,
  j00:2,j01:0,j02:0,j10:0,j11:2,j12:2,
  x_ppm:31.746032,x_min:0,x_max:100,x_pre:0,z_observe:0,
  z_ppm:480,z_min:-80,z_max:0,z_grab:-80,z_place:-60,z_lift:-40,
  pos_tol:1,stop_speed:2,feedback_ms:2000,close_ms:500
});

// 点击“逐项发送调试预设”时才发送这里列出的键。只写当前要调的参数，避免覆盖其余实测标定。
export const GRAB_TEST_PRESET = Object.freeze({
  travel_mm:0,
  body_speed:20,
  accel:20,
  lambda:0.6,
  loss_grace_ms:250
});
