/**
 * @file    mechanical_arm_config.h
 * @brief   机械臂三轴安装方向配置。
 */
#ifndef __MECHANICAL_ARM_CONFIG_H
#define __MECHANICAL_ARM_CONFIG_H

/*
 * 方向约定：俯视小车时，照片拍摄方向为小车前进方向。
 * 宏值对应张大头位置命令中的dir：0为驱动器CW，1为驱动器CCW。
 * 这里填写的是结合电机安装方向后的实车标定值。
 */
#define MECHANICAL_ARM_BASE_POSITIVE_DIR  1U
/* Base位置输入正值时，底座从上往下俯视应顺时针旋转。 */

#define MECHANICAL_ARM_Z_POSITIVE_DIR     0U
/* 2026-09-27实测当前安装：CCW向下，因此正值上升使用CW；负值下降。 */

#define MECHANICAL_ARM_X_POSITIVE_DIR     1U
/* X位置输入正值时，机械臂应向外伸出；输入负值时向内收回。 */

#define MECHANICAL_ARM_COMMAND_PULSES_PER_REV  3200U
/* 1.8度步进电机使用16细分时，一圈位置命令对应3200个脉冲。 */

#define MECHANICAL_ARM_DEGREES_PER_REV         360.0f
/* 电机轴旋转一圈对应的角度，用于控制台角度和命令脉冲换算。 */

#endif /* __MECHANICAL_ARM_CONFIG_H */
