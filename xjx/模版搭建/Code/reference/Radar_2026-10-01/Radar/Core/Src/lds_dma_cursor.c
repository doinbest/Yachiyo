#include "lds_dma_cursor.h"

uint16_t LdsDmaCursor_WritePosition(uint16_t remaining_count)
{
  if (remaining_count >= LDS_DMA_RX_BUFFER_SIZE)
  {
    return 0U;
  }

  return remaining_count == 0U
             ? 0U
             : (uint16_t)(LDS_DMA_RX_BUFFER_SIZE - remaining_count);
}
