/**
 * @file    chassis_key_test.h
 * @brief   四按键麦克纳姆底盘1米平移测试程序。
 */
#ifndef CHASSIS_KEY_TEST_H
#define CHASSIS_KEY_TEST_H

#include "jy61p.h"

/**********************************************************
*** 按键底盘测试初始化
**********************************************************/
/**
  * @brief    初始化四按键状态和运动互锁状态
  * @param    无
  * @retval   无
  * @note     必须在 GPIO、UART5 DMA 和电机上电等待完成后调用
  */
void Chassis_Key_Init(void);

/**********************************************************
*** 按键底盘测试主循环
**********************************************************/
/**
  * @brief    扫描方向按键，并控制底盘前后左右移动1米
  * @param    imu_angle ：最新的JY61P角度数据，必须已经收到过有效帧
  * @retval   无
  * @note     应在 while(1) 中持续调用；运动期间每50ms更新一次航向PID
  */
void Chassis_Key_Process(const JY61P_Angle_t *imu_angle);

#endif /* CHASSIS_KEY_TEST_H */
