#ifndef __MOTOR_ID_CONFIG_H
#define __MOTOR_ID_CONFIG_H

/* 车体俯视且车头朝上，四个底盘电机从左上角开始逆时针编号。 */
#define MOTOR_ID_CHASSIS_FRONT_LEFT   1U  /* 左前轮（俯视左上角）。 */
#define MOTOR_ID_CHASSIS_REAR_LEFT    2U  /* 左后轮（俯视左下角）。 */
#define MOTOR_ID_CHASSIS_REAR_RIGHT   3U  /* 右后轮（俯视右下角）。 */
#define MOTOR_ID_CHASSIS_FRONT_RIGHT  4U  /* 右前轮（俯视右上角）。 */

/* 机械臂三轴编号接续底盘电机。 */
#define MOTOR_ID_ARM_BASE             5U  /* 机械臂底座旋转轴。 */
#define MOTOR_ID_ARM_Z                6U  /* 机械臂Z轴升降。 */
#define MOTOR_ID_ARM_X                7U  /* 机械臂X轴伸缩。 */

#endif /* __MOTOR_ID_CONFIG_H */
