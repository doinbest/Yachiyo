#ifndef TURNTABLE_CONFIG_H
#define TURNTABLE_CONFIG_H
/* Emm 固件，默认 1.8 度/16 细分；用户确认转盘 1:1 直连。 */
#define TURNTABLE_MOTOR_ID           1U
#define TURNTABLE_SPEED_RPM          8U     /* 赛事 6-10 秒/圈，即 6-10 RPM。 */
#define TURNTABLE_ACCELERATION       10U    /* Emm 加速度档位，不是 RPM/s。 */
#define TURNTABLE_DIRECTION          0U     /* 0=CW，1=CCW。 */
#define TURNTABLE_DWELL_MS           2000U
#define TURNTABLE_PULSES_PER_REV     3200U
#define TURNTABLE_POSITION_PER_REV   65536U /* 实时反馈单位，与命令脉冲不同。 */
#define TURNTABLE_POSITION_WINDOW    91     /* 约 0.5 度，另要求到位标志、零速。 */
#define TURNTABLE_POLL_MS            50U
#define TURNTABLE_REPLY_TIMEOUT_MS   250U
#define TURNTABLE_MOVE_TIMEOUT_MS    10000U
#define TURNTABLE_STOP_TIMEOUT_MS    2000U
#define TURNTABLE_POWERUP_WAIT_MS    1000U
#define TURNTABLE_KEY_DEBOUNCE_MS    20U
#endif
