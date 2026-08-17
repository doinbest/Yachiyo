/**
 * @file    chassis_key_test.h
 * @brief   四按键麦克纳姆底盘1米平移测试程序。
 */
#ifndef CHASSIS_KEY_TEST_H
#define CHASSIS_KEY_TEST_H

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
  * @param    无
  * @retval   无
  * @note     应在 while(1) 中持续调用；每次有效按下只发送一组同步命令
  */
void Chassis_Key_Process(void);

#endif /* CHASSIS_KEY_TEST_H */
