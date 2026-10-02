#ifndef NAVIGATION_PORT_H
#define NAVIGATION_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "lds_receiver.h"

typedef struct
{
  void (*receiver_init)(void);
  void (*receiver_start)(void);
  void (*receiver_stop)(void);
  void (*receiver_poll)(void);
  uint8_t (*read_event)(LdsReceiverEvent *event);
  LdsReceiverStatus (*receiver_status)(void);
  void (*receiver_stats)(LdsParserStats *stats);
  uint32_t (*get_tick)(void);
} NavigationPort;

const NavigationPort *NavigationPort_Default(void);

#ifdef __cplusplus
}
#endif

#endif
