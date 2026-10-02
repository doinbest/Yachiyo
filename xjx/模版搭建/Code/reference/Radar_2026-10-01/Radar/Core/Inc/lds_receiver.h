#ifndef LDS_RECEIVER_H
#define LDS_RECEIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "lds_parser.h"
#include "lds_result_queue.h"

typedef enum
{
  LDS_RECEIVER_OK = 0,
  LDS_RECEIVER_DMA_START_FAILED,
  LDS_RECEIVER_COMMAND_FAILED
} LdsReceiverStatus;

extern LdsParser g_lds_parser;
extern volatile LdsReceiverStatus g_lds_receiver_status;

void LdsReceiver_Init(void);
void LdsReceiver_Poll(void);
void LdsReceiver_ConfigureAndStart(void);
void LdsReceiver_Stop(void);
uint8_t LdsReceiver_ReadEvent(LdsReceiverEvent *event);
LdsReceiverStatus LdsReceiver_GetStatus(void);
void LdsReceiver_GetStats(LdsParserStats *stats);

#ifdef __cplusplus
}
#endif

#endif
