#include "console_rx.h"
#include "arm_console.h"
#include <string.h>

/* Short half-buffer interval bounds Ctrl+C inspection during a continuous burst. */
#define RX_DMA_SIZE 128U
static uint8_t dma_buffer[RX_DMA_SIZE];
static UART_HandleTypeDef *port;
static uint16_t read_position;
static volatile uint8_t recover;
static uint32_t retry_at;
static ConsoleRx_Stats_t stats;

HAL_StatusTypeDef ConsoleRx_Init(UART_HandleTypeDef *uart)
{
  if (!uart || uart->Instance != USART1 || !uart->hdmarx) return HAL_ERROR;
  port = uart;
  /* NDTR==Size and NDTR==0 both denote the ring boundary represented as Size. */
  read_position = RX_DMA_SIZE;
  recover = 0;
  memset(&stats, 0, sizeof(stats));
  return HAL_UARTEx_ReceiveToIdle_DMA(port, dma_buffer, sizeof(dma_buffer));
}
void ConsoleRx_RxEventCallback(UART_HandleTypeDef *uart)
{
  uint16_t position, i;
  if (uart != port || recover) return;
  /* Read NDTR, not the callback Size: an IDLE IRQ may precede an already
     pending HT/TC IRQ. Replaying that older Size would duplicate bytes. */
  position = RX_DMA_SIZE - (uint16_t)__HAL_DMA_GET_COUNTER(port->hdmarx);
  if (position == 0) position = RX_DMA_SIZE;
  if (position > RX_DMA_SIZE || position == read_position) return;
  if (position < read_position) {
    for (i = read_position; i < RX_DMA_SIZE; i++) {
      ArmConsole_ReceiveData(dma_buffer[i]); stats.bytes_received++;
    }
    read_position = 0;
  }
  for (i = read_position; i < position; i++) {
    ArmConsole_ReceiveData(dma_buffer[i]); stats.bytes_received++;
  }
  read_position = position;
}
void ConsoleRx_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart != port) return;
  stats.uart_errors++;
  if (uart->ErrorCode & HAL_UART_ERROR_ORE) stats.overruns++;
  if (uart->ErrorCode & HAL_UART_ERROR_FE) stats.framing_errors++;
  if (uart->ErrorCode & HAL_UART_ERROR_NE) stats.noise_errors++;
  if (uart->ErrorCode & HAL_UART_ERROR_DMA) stats.dma_errors++;
  recover = 1;
  retry_at = HAL_GetTick() - 100U;
  ArmConsole_ReceiveFault();
}
void ConsoleRx_Process(void)
{
  uint32_t mask;
  HAL_StatusTypeDef result;
  if (!port || !recover || (uint32_t)(HAL_GetTick() - retry_at) < 100U) return;
  retry_at = HAL_GetTick();
  /* HAL abort/restart is done in main, never from the UART error callback.
     AbortReceive leaves a concurrently running TX DMA untouched. */
  (void)HAL_UART_AbortReceive(port);
  mask = __get_PRIMASK();
  __disable_irq();
  read_position = RX_DMA_SIZE;
  result = HAL_UARTEx_ReceiveToIdle_DMA(port, dma_buffer, sizeof(dma_buffer));
  if (result == HAL_OK) { recover = 0; stats.restarts++; }
  else stats.restart_failures++;
  __set_PRIMASK(mask);
}
void ConsoleRx_GetStats(ConsoleRx_Stats_t *out)
{
  uint32_t mask;
  if (!out) return;
  mask = __get_PRIMASK(); __disable_irq();
  *out = stats;
  __set_PRIMASK(mask);
}
