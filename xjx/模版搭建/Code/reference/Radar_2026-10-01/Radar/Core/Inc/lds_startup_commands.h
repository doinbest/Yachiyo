#ifndef LDS_STARTUP_COMMANDS_H
#define LDS_STARTUP_COMMANDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define LDS_STARTUP_COMMAND_LENGTH 6U

static uint8_t LdsStartupCommand_Count(void)
{
  return 5U;
}

static const char *LdsStartupCommand_Get(uint8_t index)
{
  static const char *const commands[] = {
      "LMDMMH",
      "LOCONH",
      "LFFF1H",
      "LSSS1H",
      "LSTARH"};

  return index < LdsStartupCommand_Count() ? commands[index] : 0;
}

#ifdef __cplusplus
}
#endif

#endif
