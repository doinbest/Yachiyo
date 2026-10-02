#ifndef NAVIGATION_REPORT_H
#define NAVIGATION_REPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "navigation_types.h"

typedef void (*NavigationReportWriter)(const char *data,
                                       uint16_t length,
                                       void *context);

typedef struct
{
  uint8_t sent;
} NavigationReportSession;

void NavigationReport_Emit(const NavigationResult *result,
                           NavigationReportWriter writer,
                           void *context);
void NavigationReportSession_Init(NavigationReportSession *session);
void NavigationReportSession_Poll(NavigationReportSession *session,
                                  const NavigationResult *result,
                                  NavigationReportWriter writer,
                                  void *context);

#ifdef __cplusplus
}
#endif

#endif
