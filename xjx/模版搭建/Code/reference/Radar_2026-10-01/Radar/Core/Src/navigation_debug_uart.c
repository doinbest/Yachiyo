#include "navigation_debug_uart.h"

#include "navigation.h"
#include "navigation_report.h"
#include "usart.h"

#define NAV_DEBUG_UART_TIMEOUT_MS 100U

static NavigationReportSession g_report_session;

static void uart3_write(const char *data, uint16_t length, void *context)
{
  (void)context;
  (void)HAL_UART_Transmit(&huart3,
                          (uint8_t *)data,
                          length,
                          NAV_DEBUG_UART_TIMEOUT_MS);
}

void NavigationDebugUart_Init(void)
{
  NavigationReportSession_Init(&g_report_session);
}

void NavigationDebugUart_Poll(void)
{
  NavigationReportSession_Poll(&g_report_session,
                               &g_navigation_result,
                               uart3_write,
                               0);
}
