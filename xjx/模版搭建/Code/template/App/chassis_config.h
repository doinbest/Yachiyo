/**
 * @file    chassis_config.h
 * @brief   四轮麦克纳姆底盘的集中参数配置。
 *
 * 修改轮径、细分或电机安装方向时，只需要检查本文件。
 */
#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#include "motor_id_config.h"

/* 四个轮子按照：左上、左下、右下、右上（0~3）排列。 */
#define CHASSIS_MOTOR_ID_FRONT_LEFT       MOTOR_ID_CHASSIS_FRONT_LEFT
#define CHASSIS_MOTOR_ID_REAR_LEFT        MOTOR_ID_CHASSIS_REAR_LEFT
#define CHASSIS_MOTOR_ID_REAR_RIGHT       MOTOR_ID_CHASSIS_REAR_RIGHT
#define CHASSIS_MOTOR_ID_FRONT_RIGHT      MOTOR_ID_CHASSIS_FRONT_RIGHT

/*
 * 电机正转方向配置。
 * 1 表示 Emm dir=1 时该轮向车体前方滚动；0 表示 Emm dir=0 时向前滚动。
 * 默认按左右电机镜像安装填写。第一次上板必须架空底盘逐轮核对；若某轮
 * 与期望相反，只翻转对应宏的 0/1，不修改麦轮运动学公式。
 */
#define CHASSIS_MOTOR_FRONT_LEFT_FORWARD_DIR    0U
#define CHASSIS_MOTOR_REAR_LEFT_FORWARD_DIR     0U
#define CHASSIS_MOTOR_REAR_RIGHT_FORWARD_DIR    1U
#define CHASSIS_MOTOR_FRONT_RIGHT_FORWARD_DIR   1U

/*
 * 作者暂报80mm，尚未确认具体 SKU 或有效滚动直径。
 * 请以实际购买规格或实测有效滚动直径为准；该参数直接决定1米位移精度。
 */
#define CHASSIS_WHEEL_DIAMETER_MM          80.0f

/* 临时假设1.8°、16细分：200*16=3200命令脉冲/圈；需读回配置确认。 */
#define CHASSIS_MOTOR_FULL_STEPS_PER_REV   200U
#define CHASSIS_MOTOR_MICROSTEP            16U
#define CHASSIS_COMMAND_PULSES_PER_REV     \
  (CHASSIS_MOTOR_FULL_STEPS_PER_REV * CHASSIS_MOTOR_MICROSTEP)

/* 物理编码器分辨率未知；0表示不可用，禁止用于Emm 0x36位置回包。 */
#define CHASSIS_ENCODER_COUNTS_PER_REV     0U

/* 电机轴与麦轮直驱时减速比为1；若增加减速器，填写“电机圈数/车轮圈数”。 */
#define CHASSIS_MOTOR_TO_WHEEL_RATIO       1.0f

/*
 * 旋转运动需要车轮中心构成矩形的轴距和轮距。
 * 作者近似测量轴距210mm、轮距240mm；后续精测轮心间距。
 */
#define CHASSIS_WHEELBASE_MM               210.0f
#define CHASSIS_TRACK_WIDTH_MM             240.0f

/* 低速进行首次底盘测试，确认方向后再逐步提高。 */
#define CHASSIS_TEST_SPEED_RPM             120U
#define CHASSIS_TEST_ACCELERATION          20U
#define CHASSIS_VELOCITY_ACCELERATION      20U
#define CHASSIS_UART_COMMAND_INTERVAL_MS   10U

/* 控制台极性测试使用独立低速参数，不影响正式底盘运动参数。 */
#define CHASSIS_POLARITY_TEST_SPEED_RPM     20U  /* 整车前后极性测试速度。 */
#define CHASSIS_POLARITY_TEST_ACCELERATION  10U  /* 极性测试加速度。 */
#define CHASSIS_POLARITY_TEST_MAX_MM        200U  /* 单次前后测试最大距离。 */
#define CHASSIS_WHEEL_TEST_MAX_RPM          100U  /* 单轮连续测试最大转速。 */

/* 按键测试的航向保持任务周期和初始PID参数，必须在实车上重新整定。 */
#define CHASSIS_KEY_HEADING_PERIOD_MS      50U
#define CHASSIS_KEY_HEADING_KP             0.02f
#define CHASSIS_KEY_HEADING_KI             0.0f
#define CHASSIS_KEY_HEADING_KD             0.001f
#define CHASSIS_KEY_HEADING_INTEGRAL_LIMIT 20.0f
#define CHASSIS_KEY_HEADING_OUTPUT_LIMIT   0.5f

/* 将原按键测试的120RPM换算为车体前向线速度，保持原测试速度不变。 */
#define CHASSIS_KEY_TEST_SPEED_MM_S \
  ((float)CHASSIS_TEST_SPEED_RPM * 3.14159265358979323846f * \
   CHASSIS_WHEEL_DIAMETER_MM / \
   (60.0f * CHASSIS_MOTOR_TO_WHEEL_RATIO))

#define CHASSIS_KEY_TEST_DISTANCE_MM       1000.0f
#define CHASSIS_MOVE_FINISH_MARGIN_MS      1000U

/* 独立补偿直行测试：先核对正方向，再进行落地调试。 */
#define CHASSIS_HEADING_TEST_YAW_SIGN      1.0f /* HWT逆时针角度增大用+1，否则-1。 */
#define CHASSIS_HEADING_TEST_KP            0.02f
#define CHASSIS_HEADING_TEST_OUTPUT_LIMIT  0.15f /* rad/s，首次测试仅使用P控制。 */

/* 参数版本与确认状态；数值变更时同步网页模型和版本。 */
#define CHASSIS_CONFIG_REVISION           "geom-20260912-v1"
#define CHASSIS_WHEELBASE_APPROXIMATE      1U
#define CHASSIS_TRACK_APPROXIMATE          1U
#define CHASSIS_DIAMETER_ASSUMED           1U
#define CHASSIS_GEAR_RATIO_ASSUMED         1U
#define CHASSIS_PULSES_PER_REV_ASSUMED     1U
#define CHASSIS_FORWARD_DIR_VERIFIED       0U
#define CHASSIS_PHYSICAL_ENCODER_BITS      0U
#define CHASSIS_EMM_POSITION_UNITS_PER_REV 65536U
/* 0x36协议位置单位不等于命令脉冲数，也不代表物理编码器分辨率。 */
#define CHASSIS_MOTION_PERIOD_MS           20U
#define CHASSIS_MOTION_MAX_GAP_MS          100U
#define CHASSIS_MOTION_DEFAULT_RAMP_MS     500U
#define CHASSIS_MOTION_MAX_LINEAR_MM_S     5000.0f
#define CHASSIS_MOTION_MAX_OMEGA_RAD_S     0.15f
/* Emm F6/FD speed field is documented as 0..3000 RPM; loaded speed is unverified. */
#define CHASSIS_EMM_MAX_COMMAND_RPM         3000U
#define CHASSIS_MOTION_MAX_WHEEL_RPM        ((float)CHASSIS_EMM_MAX_COMMAND_RPM)
#define CHASSIS_MOTION_MAX_DURATION_MS     60000U

/* Route speed profile; verify acceleration and stopping distance on the chassis. */
#define CHASSIS_DISTANCE_SPEED_MM_S       50.0f
#define CHASSIS_ROUTE_DEFAULT_SPEED_MM_S  5000.0f
#define CHASSIS_ROUTE_AUTO_DWELL_MS       100U
#define CHASSIS_DISTANCE_KP               2.5f
#define CHASSIS_DISTANCE_RESPONSE_S       0.25f
#define CHASSIS_DISTANCE_CREEP_MM         20.0f
#define CHASSIS_DISTANCE_CREEP_MM_S       50.0f
#define CHASSIS_DISTANCE_CORRECTION_SPEED_MM_S 20.0f
#define CHASSIS_DISTANCE_CORRECTION_MM   100.0f
#define CHASSIS_DISTANCE_CORRECTIONS       2U
#define CHASSIS_DISTANCE_ACCEL_MM_S2      1000.0f
#define CHASSIS_DISTANCE_TOLERANCE_MM     10.0f
#define CHASSIS_DISTANCE_HEADING_DEG      2.0f
#define CHASSIS_DISTANCE_TIMEOUT_MS      60000U
#define CHASSIS_DISTANCE_MAX_TIMEOUT_MS  400000U /* Full route: up to 1750 mm at 10 mm/s plus settling. */
#define CHASSIS_DISTANCE_STOP_TIMEOUT_MS 3000U
#define CHASSIS_DISTANCE_STOP_STABLE_MS   500U
#define CHASSIS_DISTANCE_STOP_GROUPS      3U
#define CHASSIS_DISTANCE_STOP_RPM         1
#define CHASSIS_DISTANCE_STOP_DELTA_MM    2.0f

#endif /* CHASSIS_CONFIG_H */
