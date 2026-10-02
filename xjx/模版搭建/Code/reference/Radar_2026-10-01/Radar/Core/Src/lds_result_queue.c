#include "lds_result_queue.h"

void LdsResultQueue_Init(LdsResultQueue *queue)
{
  queue->head = 0U;
  queue->tail = 0U;
  queue->count = 0U;
}

uint8_t LdsResultQueue_Push(LdsResultQueue *queue,
                            const LdsReceiverEvent *event)
{
  if (queue == 0 || event == 0 || queue->count >= LDS_RESULT_QUEUE_CAPACITY)
  {
    return 0U;
  }

  queue->items[queue->tail] = *event;
  queue->tail = (uint8_t)((queue->tail + 1U) % LDS_RESULT_QUEUE_CAPACITY);
  ++queue->count;
  return 1U;
}

uint8_t LdsResultQueue_Pop(LdsResultQueue *queue, LdsReceiverEvent *event)
{
  if (queue == 0 || event == 0 || queue->count == 0U)
  {
    return 0U;
  }

  *event = queue->items[queue->head];
  queue->head = (uint8_t)((queue->head + 1U) % LDS_RESULT_QUEUE_CAPACITY);
  --queue->count;
  return 1U;
}

uint8_t LdsResultQueue_IsFull(const LdsResultQueue *queue)
{
  return queue != 0 && queue->count >= LDS_RESULT_QUEUE_CAPACITY ? 1U : 0U;
}
