#ifndef LDS_RESULT_QUEUE_H
#define LDS_RESULT_QUEUE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "lds_parser.h"

#define LDS_RESULT_QUEUE_CAPACITY 2U

typedef enum
{
  LDS_RECEIVER_EVENT_NONE = 0,
  LDS_RECEIVER_EVENT_MEASUREMENT,
  LDS_RECEIVER_EVENT_STATUS
} LdsReceiverEventType;

typedef struct
{
  LdsReceiverEventType type;
  union
  {
    LdsParserResult measurement;
    LdsParserStatus status;
  } payload;
} LdsReceiverEvent;

typedef struct
{
  LdsReceiverEvent items[LDS_RESULT_QUEUE_CAPACITY];
  uint8_t head;
  uint8_t tail;
  uint8_t count;
} LdsResultQueue;

void LdsResultQueue_Init(LdsResultQueue *queue);
uint8_t LdsResultQueue_Push(LdsResultQueue *queue,
                            const LdsReceiverEvent *event);
uint8_t LdsResultQueue_Pop(LdsResultQueue *queue, LdsReceiverEvent *event);
uint8_t LdsResultQueue_IsFull(const LdsResultQueue *queue);

#ifdef __cplusplus
}
#endif

#endif
