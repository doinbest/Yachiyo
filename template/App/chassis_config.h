/**
 * @file    chassis_config.h
 * @brief   四轮麦克纳姆底盘的集中参数配置。
 *
 * 修改轮径、细分或电机安装方向时，只需要检查本文件。
 */
#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* 四个轮子按照：左上、左下、右下、右上（0~3）排列。 */
#define CHASSIS_MOTOR_ID_FRONT_LEFT       1U
#define CHASSIS_MOTOR_ID_REAR_LEFT        2U
#define CHASSIS_MOTOR_ID_REAR_RIGHT       3U
#define CHASSIS_MOTOR_ID_FRONT_RIGHT      4U

/*
 * 电机正转方向配置。
 * 1 表示 Emm dir=1 时该轮向车体前方滚动；0 表示 Emm dir=0 时向前滚动。
 * 默认按左右电机镜像安装填写。第一次上板必须架空底盘逐轮核对；若某轮
 * 与期望相反，只翻转对应宏的 0/1，不修改麦轮运动学公式。
 */
#define CHASSIS_MOTOR_FRONT_LEFT_FORWARD_DIR    1U
#define CHASSIS_MOTOR_REAR_LEFT_FORWARD_DIR     1U
#define CHASSIS_MOTOR_REAR_RIGHT_FORWARD_DIR    0U
#define CHASSIS_MOTOR_FRONT_RIGHT_FORWARD_DIR   0U

/*
 * 淘宝商品页未公开具体 SKU，暂按直径100mm配置。
 * 请以实际购买规格或实测有效滚动直径为准；该参数直接决定1米位移精度。
 */
#define CHASSIS_WHEEL_DIAMETER_MM          100.0f

/* X42S为1.8°步进电机且驱动器设置16细分：200*16=3200命令脉冲/圈。 */
#define CHASSIS_MOTOR_FULL_STEPS_PER_REV   200U
#define CHASSIS_MOTOR_MICROSTEP            16U
#define CHASSIS_COMMAND_PULSES_PER_REV     \
  (CHASSIS_MOTOR_FULL_STEPS_PER_REV * CHASSIS_MOTOR_MICROSTEP)

/* X42S V1.0手册标注14位编码器：2^14=16384计数/圈，仅用于反馈换算。 */
#define CHASSIS_ENCODER_COUNTS_PER_REV     16384U

/* 电机轴与麦轮直驱时减速比为1；若增加减速器，填写“电机圈数/车轮圈数”。 */
#define CHASSIS_MOTOR_TO_WHEEL_RATIO       1.0f

/*
 * 旋转运动需要车轮中心构成矩形的轴距和轮距。
 * 当前四按键只测试平移，这两个值暂设为0；增加旋转命令前必须实测填写。
 */
#define CHASSIS_WHEELBASE_MM               0.0f
#define CHASSIS_TRACK_WIDTH_MM             0.0f

/* 低速进行首次底盘测试，确认方向后再逐步提高。 */
#define CHASSIS_TEST_SPEED_RPM             120U
#define CHASSIS_TEST_ACCELERATION          20U
#define CHASSIS_UART_COMMAND_INTERVAL_MS   10U

#define CHASSIS_KEY_TEST_DISTANCE_MM       1000.0f
#define CHASSIS_MOVE_FINISH_MARGIN_MS      1000U

#endif /* CHASSIS_CONFIG_H */
