"""Compile the production CDC transmitter with host-side USB boundary stubs."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / 'template/USB_DEVICE/App/usbd_cdc_if.c').read_text(encoding='utf-8-sig')
function = re.search(r'uint8_t CDC_Transmit_FS\(.*?\n}\n', source, re.S).group()
configured_function = re.search(r'uint8_t CDC_IsConfigured_FS\(void\).*?\n}\n', source, re.S).group()
prefix = r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#define USBD_OK 0U
#define USBD_BUSY 1U
#define USBD_FAIL 2U
#define USBD_STATE_CONFIGURED 3U
#define APP_TX_DATA_SIZE 16U
typedef struct { uint32_t TxState; } USBD_CDC_HandleTypeDef;
typedef struct { void *pClassData; unsigned int dev_state; } USBD_HandleTypeDef;
static USBD_HandleTypeDef hUsbDeviceFS;
static uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
static uint8_t *submitted;
static unsigned int interrupts_masked;
static uint32_t __get_PRIMASK(void) { return interrupts_masked; }
static void __disable_irq(void) { interrupts_masked = 1; }
static void __set_PRIMASK(uint32_t value) { interrupts_masked = value; }
static void USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *device, uint8_t *data, uint16_t length)
{ (void)device; assert(length == 4); submitted = data; }
static uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *device)
{ ((USBD_CDC_HandleTypeDef*)device->pClassData)->TxState = 1; return USBD_OK; }
'''
fixture = r'''
int main(void)
{
  uint8_t local[4] = {0xff, 0xb2, 3, 0xff};
  USBD_CDC_HandleTypeDef cdc = {0};
  assert(CDC_IsConfigured_FS() == 0U);
  hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
  assert(CDC_IsConfigured_FS() == 0U);
  hUsbDeviceFS.dev_state = 0U;
  assert(CDC_Transmit_FS(local, 4) == USBD_FAIL);
  assert(interrupts_masked == 0);
  hUsbDeviceFS.pClassData = &cdc;
  assert(CDC_IsConfigured_FS() == 0U);
  hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
  assert(CDC_IsConfigured_FS() == 1U);
  assert(CDC_Transmit_FS(NULL, 4) == USBD_FAIL);
  assert(CDC_Transmit_FS(local, 17) == USBD_FAIL);
  assert(CDC_Transmit_FS(local, 4) == USBD_OK);
  assert(submitted != local && memcmp(submitted, local, 4) == 0);
  local[2] = 6; /* caller reuses stack while USB still owns previous bytes */
  assert(submitted[2] == 3);
  assert(CDC_Transmit_FS(local, 4) == USBD_BUSY);
  assert(submitted[2] == 3 && interrupts_masked == 0);
  cdc.TxState = 0; interrupts_masked = 1;
  assert(CDC_Transmit_FS(local, 4) == USBD_OK);
  assert(submitted[2] == 6 && interrupts_masked == 1);
  puts("USB transmit connection, BUSY, buffer lifetime and IRQ-state tests passed");
  return 0;
}
'''
with tempfile.TemporaryDirectory() as folder:
    path = Path(folder)
    (path / 'test.c').write_text(prefix + configured_function + function + fixture, encoding='utf-8')
    subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror', str(path / 'test.c'),
                    '-o', str(path / 'test.exe')], check=True)
    subprocess.run([str(path / 'test.exe')], check=True)
