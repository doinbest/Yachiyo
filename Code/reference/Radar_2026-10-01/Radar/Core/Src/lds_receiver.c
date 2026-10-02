#include "lds_receiver.h"

#include "lds_dma_cursor.h"
#include "lds_result_queue.h"
#include "lds_startup_commands.h"
#include "main.h"
#include "usart.h"

#define LDS_COMMAND_TIMEOUT_MS 100U
#define LDS_COMMAND_GAP_MS     20U

LdsParser g_lds_parser;
volatile LdsReceiverStatus g_lds_receiver_status = LDS_RECEIVER_OK;

static uint8_t dma_rx_buffer[LDS_DMA_RX_BUFFER_SIZE];
static uint16_t dma_read_position;
static LdsResultQueue receiver_event_queue;
static LdsReceiverEvent parser_event_scratch;

static void begin_measurement_window(void)
{
  LdsParser_Init(&g_lds_parser);
  LdsResultQueue_Init(&receiver_event_queue);
  dma_read_position = LdsDmaCursor_WritePosition(
      (uint16_t)__HAL_DMA_GET_COUNTER(huart2.hdmarx));
}

static HAL_StatusTypeDef send_command(const char *command, uint16_t length)
{
  HAL_StatusTypeDef status;

  status = HAL_UART_Transmit(&huart2,
                             (uint8_t *)command,
                             length,
                             LDS_COMMAND_TIMEOUT_MS);
  HAL_Delay(LDS_COMMAND_GAP_MS);
  return status;
}

void LdsReceiver_Init(void)
{
  LdsParser_Init(&g_lds_parser);
  LdsResultQueue_Init(&receiver_event_queue);
  dma_read_position = 0U;
  g_lds_receiver_status = LDS_RECEIVER_OK;

  if (HAL_UART_Receive_DMA(&huart2,
                           dma_rx_buffer,
                           LDS_DMA_RX_BUFFER_SIZE) != HAL_OK)
  {
    g_lds_receiver_status = LDS_RECEIVER_DMA_START_FAILED;
    return;
  }

  /* Polling the circular DMA position avoids a 50 kHz byte interrupt load. */
  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT | DMA_IT_TC);
}

void LdsReceiver_Poll(void)
{
  uint16_t write_position;

  if (g_lds_receiver_status != LDS_RECEIVER_OK ||
      huart2.hdmarx == 0)
  {
    return;
  }

  write_position = LdsDmaCursor_WritePosition(
      (uint16_t)__HAL_DMA_GET_COUNTER(huart2.hdmarx));

  while (write_position != dma_read_position)
  {
    if (LdsResultQueue_IsFull(&receiver_event_queue) != 0U)
    {
      return;
    }

    LdsParser_Feed(&g_lds_parser,
                   &dma_rx_buffer[dma_read_position],
                   1U);
    ++dma_read_position;
    if (dma_read_position >= LDS_DMA_RX_BUFFER_SIZE)
    {
      dma_read_position = 0U;
    }
    if (LdsParser_ReadResult(
            &g_lds_parser,
            &parser_event_scratch.payload.measurement) != 0U)
    {
      parser_event_scratch.type = LDS_RECEIVER_EVENT_MEASUREMENT;
      (void)LdsResultQueue_Push(&receiver_event_queue, &parser_event_scratch);
    }
    else if (LdsParser_ReadStatus(
                 &g_lds_parser,
                 &parser_event_scratch.payload.status) != 0U)
    {
      parser_event_scratch.type = LDS_RECEIVER_EVENT_STATUS;
      (void)LdsResultQueue_Push(&receiver_event_queue, &parser_event_scratch);
    }
  }
}

void LdsReceiver_ConfigureAndStart(void)
{
  uint8_t index;

  if (g_lds_receiver_status == LDS_RECEIVER_DMA_START_FAILED)
  {
    return;
  }
  g_lds_receiver_status = LDS_RECEIVER_OK;
  if (send_command("LSTOPH", 6U) != HAL_OK)
  {
    g_lds_receiver_status = LDS_RECEIVER_COMMAND_FAILED;
    return;
  }

  for (index = 0U; index < LdsStartupCommand_Count(); ++index)
  {
    if (send_command(LdsStartupCommand_Get(index),
                     LDS_STARTUP_COMMAND_LENGTH) != HAL_OK)
    {
      g_lds_receiver_status = LDS_RECEIVER_COMMAND_FAILED;
      return;
    }
  }

  /* Exclude preflight replies and stale circular-DMA bytes from the scan. */
  begin_measurement_window();
}

void LdsReceiver_Stop(void)
{
  if (send_command("LSTOPH", 6U) != HAL_OK)
  {
    g_lds_receiver_status = LDS_RECEIVER_COMMAND_FAILED;
  }
}

uint8_t LdsReceiver_ReadEvent(LdsReceiverEvent *event)
{
  return LdsResultQueue_Pop(&receiver_event_queue, event);
}

LdsReceiverStatus LdsReceiver_GetStatus(void)
{
  return g_lds_receiver_status;
}

void LdsReceiver_GetStats(LdsParserStats *stats)
{
  if (stats != 0)
  {
    *stats = g_lds_parser.stats;
  }
}
