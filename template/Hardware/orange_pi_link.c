/**
 * @file    orange_pi_link.c
 * @brief   香橙派USB CDC可靠帧接收实现。
 */
#include "orange_pi_link.h"

#include <string.h>

#define ORANGE_PI_SOF_0                    0xA5U
#define ORANGE_PI_SOF_1                    0x5AU
#define ORANGE_PI_FIXED_FIELD_LENGTH       10U
#define ORANGE_PI_FRAME_MIN_LENGTH         14U
#define ORANGE_PI_FRAME_MAX_LENGTH         \
  (ORANGE_PI_FRAME_MIN_LENGTH + ORANGE_PI_PAYLOAD_MAX_LENGTH)
#define ORANGE_PI_FRAME_QUEUE_LENGTH       4U
#define ORANGE_PI_MAX_BYTES_PER_PROCESS    32U
#define ORANGE_PI_PARTIAL_FRAME_TIMEOUT_MS 50U
#define ORANGE_PI_RX_RING_LENGTH           128U

static uint8_t orange_pi_rx_ring[ORANGE_PI_RX_RING_LENGTH];
static volatile uint16_t orange_pi_rx_write = 0U;
static volatile uint16_t orange_pi_rx_read = 0U;
static volatile bool orange_pi_parser_reset_requested = false;
static uint8_t orange_pi_parser_buffer[ORANGE_PI_FRAME_MAX_LENGTH];
static uint16_t orange_pi_parser_count = 0U;
static uint16_t orange_pi_expected_length = 0U;
static uint32_t orange_pi_last_byte_ms = 0U;
static OrangePiFrame_t orange_pi_frame_queue[ORANGE_PI_FRAME_QUEUE_LENGTH];
static uint8_t orange_pi_queue_read = 0U;
static uint8_t orange_pi_queue_write = 0U;
static uint8_t orange_pi_queue_count = 0U;
static OrangePiLinkStatus_t orange_pi_status;

static uint16_t OrangePi_Link_U16_Read(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t OrangePi_Link_U32_Read(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) |
         ((uint32_t)data[3] << 24U);
}

static void OrangePi_Link_Parser_Reset(void)
{
  orange_pi_parser_count = 0U;
  orange_pi_expected_length = 0U;
}

static void OrangePi_Link_Frame_Queue_Push(const OrangePiFrame_t *frame)
{
  if (frame == NULL)
  {
    return;
  }

  if (orange_pi_queue_count >= ORANGE_PI_FRAME_QUEUE_LENGTH)
  {
    ++orange_pi_status.queue_overflow_count;
    return;
  }

  orange_pi_frame_queue[orange_pi_queue_write] = *frame;
  orange_pi_queue_write = (uint8_t)((orange_pi_queue_write + 1U) %
                                    ORANGE_PI_FRAME_QUEUE_LENGTH);
  ++orange_pi_queue_count;
}

static void OrangePi_Link_Complete_Frame_Process(uint32_t now_ms)
{
  OrangePiFrame_t frame;
  uint16_t payload_length;
  uint16_t crc_received;
  uint16_t crc_calculated;

  payload_length = OrangePi_Link_U16_Read(&orange_pi_parser_buffer[6]);
  crc_received = OrangePi_Link_U16_Read(
    &orange_pi_parser_buffer[12U + payload_length]);
  crc_calculated = OrangePi_Link_Crc16_Calc(
    &orange_pi_parser_buffer[2],
    (uint16_t)(ORANGE_PI_FIXED_FIELD_LENGTH + payload_length));

  if (orange_pi_parser_buffer[2] != ORANGE_PI_PROTOCOL_VERSION)
  {
    ++orange_pi_status.version_error_count;
    return;
  }
  if (crc_received != crc_calculated)
  {
    ++orange_pi_status.crc_error_count;
    return;
  }

  (void)memset(&frame, 0, sizeof(frame));
  frame.type = orange_pi_parser_buffer[3];
  frame.seq = OrangePi_Link_U16_Read(&orange_pi_parser_buffer[4]);
  frame.payload_length = payload_length;
  frame.remote_timestamp_ms = OrangePi_Link_U32_Read(
    &orange_pi_parser_buffer[8]);
  if (payload_length > 0U)
  {
    (void)memcpy(frame.payload, &orange_pi_parser_buffer[12],
                 payload_length);
  }

  OrangePi_Link_Frame_Queue_Push(&frame);
  ++orange_pi_status.valid_frame_count;
  orange_pi_status.last_valid_frame_ms = now_ms;
  orange_pi_status.last_remote_timestamp_ms = frame.remote_timestamp_ms;
  orange_pi_status.has_valid_frame = true;
}

static void OrangePi_Link_Byte_Process(uint8_t data, uint32_t now_ms)
{
  uint16_t payload_length;

  orange_pi_last_byte_ms = now_ms;

  if (orange_pi_parser_count == 0U)
  {
    if (data == ORANGE_PI_SOF_0)
    {
      orange_pi_parser_buffer[0] = data;
      orange_pi_parser_count = 1U;
    }
    return;
  }

  if (orange_pi_parser_count == 1U)
  {
    if (data == ORANGE_PI_SOF_1)
    {
      orange_pi_parser_buffer[1] = data;
      orange_pi_parser_count = 2U;
    }
    else if (data != ORANGE_PI_SOF_0)
    {
      OrangePi_Link_Parser_Reset();
    }
    return;
  }

  if (orange_pi_parser_count >= ORANGE_PI_FRAME_MAX_LENGTH)
  {
    ++orange_pi_status.length_error_count;
    OrangePi_Link_Parser_Reset();
    return;
  }

  orange_pi_parser_buffer[orange_pi_parser_count] = data;
  ++orange_pi_parser_count;

  if (orange_pi_parser_count == 8U)
  {
    payload_length = OrangePi_Link_U16_Read(&orange_pi_parser_buffer[6]);
    if (payload_length > ORANGE_PI_PAYLOAD_MAX_LENGTH)
    {
      ++orange_pi_status.length_error_count;
      OrangePi_Link_Parser_Reset();
      return;
    }
    orange_pi_expected_length = (uint16_t)(ORANGE_PI_FRAME_MIN_LENGTH +
                                            payload_length);
  }

  if ((orange_pi_expected_length > 0U) &&
      (orange_pi_parser_count >= orange_pi_expected_length))
  {
    OrangePi_Link_Complete_Frame_Process(now_ms);
    OrangePi_Link_Parser_Reset();
  }
}

bool OrangePi_Link_Init(void)
{
  OrangePi_Link_Parser_Reset();
  orange_pi_queue_read = 0U;
  orange_pi_queue_write = 0U;
  orange_pi_queue_count = 0U;
  orange_pi_rx_write = 0U;
  orange_pi_rx_read = 0U;
  orange_pi_parser_reset_requested = false;
  orange_pi_last_byte_ms = HAL_GetTick();
  (void)memset(&orange_pi_status, 0, sizeof(orange_pi_status));
  return true;
}

void OrangePi_Link_Process(void)
{
  uint8_t data;
  uint8_t processed = 0U;
  uint32_t now_ms;

  now_ms = HAL_GetTick();
  if (orange_pi_parser_reset_requested)
  {
    orange_pi_parser_reset_requested = false;
    OrangePi_Link_Parser_Reset();
  }
  if ((orange_pi_parser_count > 0U) &&
      ((uint32_t)(now_ms - orange_pi_last_byte_ms) >=
       ORANGE_PI_PARTIAL_FRAME_TIMEOUT_MS))
  {
    ++orange_pi_status.frame_timeout_count;
    OrangePi_Link_Parser_Reset();
  }

  while (processed < ORANGE_PI_MAX_BYTES_PER_PROCESS)
  {
    if (orange_pi_rx_read == orange_pi_rx_write)
    {
      break;
    }

    data = orange_pi_rx_ring[orange_pi_rx_read];
    orange_pi_rx_read = (uint16_t)((orange_pi_rx_read + 1U) %
                                   ORANGE_PI_RX_RING_LENGTH);
    OrangePi_Link_Byte_Process(data, HAL_GetTick());
    ++processed;
  }
}

bool OrangePi_Link_Frame_Get(OrangePiFrame_t *frame)
{
  if ((frame == NULL) || (orange_pi_queue_count == 0U))
  {
    return false;
  }

  *frame = orange_pi_frame_queue[orange_pi_queue_read];
  orange_pi_queue_read = (uint8_t)((orange_pi_queue_read + 1U) %
                                   ORANGE_PI_FRAME_QUEUE_LENGTH);
  --orange_pi_queue_count;
  return true;
}

bool OrangePi_Link_Is_Alive(uint32_t max_age_ms)
{
  if (!orange_pi_status.has_valid_frame)
  {
    return false;
  }
  return ((uint32_t)(HAL_GetTick() - orange_pi_status.last_valid_frame_ms) <=
          max_age_ms);
}

bool OrangePi_Link_Status_Get(OrangePiLinkStatus_t *status)
{
  uint32_t interrupt_state;

  if (status == NULL)
  {
    return false;
  }
  interrupt_state = __get_PRIMASK();
  __disable_irq();
  *status = orange_pi_status;
  if (interrupt_state == 0U)
  {
    __enable_irq();
  }
  return true;
}

uint16_t OrangePi_Link_Crc16_Calc(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  uint16_t index;
  uint8_t bit;

  if ((data == NULL) && (length > 0U))
  {
    return 0U;
  }

  for (index = 0U; index < length; ++index)
  {
    crc ^= (uint16_t)data[index] << 8U;
    for (bit = 0U; bit < 8U; ++bit)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1U) ^ 0x1021U);
      }
      else
      {
        crc <<= 1U;
      }
    }
  }
  return crc;
}

void OrangePi_Link_Usb_Rx_Callback(const uint8_t *data, uint32_t length)
{
  uint32_t index;
  uint16_t next_write;

  if ((data == NULL) || (length == 0U))
  {
    return;
  }

  for (index = 0U; index < length; ++index)
  {
    next_write = (uint16_t)((orange_pi_rx_write + 1U) %
                            ORANGE_PI_RX_RING_LENGTH);
    if (next_write == orange_pi_rx_read)
    {
      ++orange_pi_status.rx_overflow_count;
      orange_pi_parser_reset_requested = true;
      break;
    }
    orange_pi_rx_ring[orange_pi_rx_write] = data[index];
    orange_pi_rx_write = next_write;
  }
}
