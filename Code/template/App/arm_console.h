#ifndef __ARM_CONSOLE_H
#define __ARM_CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "mechanical_arm.h"

/** @brief 初始化USART1文本控制台；UART须已配置，RX DMA由main启动。
 * @return HAL_OK或句柄/端口错误HAL_ERROR；初始化不控制执行器。 */
HAL_StatusTypeDef ArmConsole_Init(UART_HandleTypeDef *huart);
/** @brief 接收回调追加一个ASCII字节；仅缓存，不执行命令。 */
void ArmConsole_ReceiveData(uint8_t Data);
/** IRQ: discard a damaged command until the next line boundary; Ctrl+C still works. */
void ArmConsole_ReceiveFault(void);
/** @brief Service latched Ctrl+C before autonomous tasks each main-loop pass. */
void ArmConsole_StopProcess(void);
/** @brief 主循环执行命令及日志；status只读缓存，角度deg、速度rpm或mm/s。 */
void ArmConsole_Process(void);
/** @brief 复位等待期间暂停主循环业务；继续服务RX/TX及停车命令。 */
uint8_t ArmConsole_ResetPending(void);
/** @brief Manual preflight/recovery owns command dispatch; block physical task starts. */
uint8_t ArmConsole_OperationBusy(void);
/** @brief 回复发送完成且等待200ms后请求软件复位；失败或5s超时取消。 */
uint8_t ArmConsole_ResetProcess(void);
/** @brief 查询连续角度日志开关；默认关闭，不影响采样和标定。
 * @return 1为开启，0为关闭；主循环使用。 */
uint8_t ArmConsole_ImuStreamEnabled(void);
/** @brief main分派已取出的电机事件，输出应答与异步提示符，不再次取事件。 */
void ArmConsole_MotorEventHandle(const MechanicalArm_EventTypeDef *Event);


#ifdef __cplusplus
}
#endif

#endif
