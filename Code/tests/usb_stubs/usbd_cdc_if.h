#include <stdint.h>
#define USBD_OK 0U
#define USBD_BUSY 1U
#define USBD_FAIL 2U
uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t length);
uint8_t CDC_IsConfigured_FS(void);
