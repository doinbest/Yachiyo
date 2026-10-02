#ifndef LDS_DMA_CURSOR_H
#define LDS_DMA_CURSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define LDS_DMA_RX_BUFFER_SIZE 256U

uint16_t LdsDmaCursor_WritePosition(uint16_t remaining_count);

#ifdef __cplusplus
}
#endif

#endif
