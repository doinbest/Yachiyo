#include "navigation_port.h"

#include "main.h"

static uint32_t get_tick(void)
{
  return HAL_GetTick();
}

static const NavigationPort g_default_navigation_port = {
    LdsReceiver_Init,
    LdsReceiver_ConfigureAndStart,
    LdsReceiver_Stop,
    LdsReceiver_Poll,
    LdsReceiver_ReadEvent,
    LdsReceiver_GetStatus,
    LdsReceiver_GetStats,
    get_tick};

const NavigationPort *NavigationPort_Default(void)
{
  return &g_default_navigation_port;
}
