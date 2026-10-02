/** @file GrabRoute.h @brief 前四段路线、模拟扫码与首件抓取的非阻塞调度。 */
#ifndef GRAB_ROUTE_H
#define GRAB_ROUTE_H
#include <stdbool.h>
void GrabRoute_Init(void);
/** @brief 主循环在路线处理后调用。扫码计时从第2段停车确认开始。 */
void GrabRoute_Process(void);
/** @brief 仅表示组合路线仍在调度；抓取阶段由GrabTask_IsBusy表示。 */
bool GrabRoute_IsBusy(void);
/** @brief 取消自动推进；请求路线/抓取停止，不松开夹爪。 */
void GrabRoute_Stop(void);
/** @brief 处理grab route <mm/s>；grab status附加路线状态后返回false。 */
bool GrabRoute_Command(unsigned count, char *tokens[]);
#endif
