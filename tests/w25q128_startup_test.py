"""Run the production startup function and Flash drivers with simulated HAL I/O."""
from pathlib import Path
import json
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / '.embeddedskills/tests'
OUT.mkdir(parents=True, exist_ok=True)
source = (ROOT / 'template/Core/Src/main.c').read_text(encoding='utf-8-sig')
startup = re.search(r'static void W25Q128_StartupCheck\(void\)\s*\{.*?\n\}', source, re.S).group()

# Values include invalid responses, arbitrary IDs and failures after valid reads.
cases = [
    [(0, [0, 0, 0]), (0, [0xEF, 0x40, 0x18]), (3, [0xAB]*3),
     (0, [0xEF, 0x17]), (1, [0xAB]*2), (2, [0xAB]*2)],
    [(0, [0, 0, 0]), (0, [255]*3), (0, [0, 1, 0]),
     (0, [0, 0]), (0, [255]*2), (0, [255, 1])],
    [(s, [0xAB]*3) for s in [1, 2, 3, 1, 2, 3]],
]
status_names = ['OK', 'ERROR', 'BUSY', 'TIMEOUT']
expected = []
for case in cases:
    lines = []
    for i, (status, data) in enumerate(case):
        line = f'[FLASH] cmd={"9F" if i < 3 else "90"} try={i % 3 + 1} HAL={status_names[status]}'
        if status == 0:
            line += ' ID=' + ' '.join(f'{x:02X}' for x in data)
            if all(x == 0 for x in data) or all(x == 255 for x in data):
                line += ' INVALID'
        lines.append(line)
    expected.append('\r\n' + '\r\n'.join(lines) + '\r\narm> ')

fixture = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "w25q128.h"
#define CONSOLE_UPLOAD_TIMEOUT_MS 100U
static SPI_HandleTypeDef hspi1;

GPIO_TypeDef test_gpioa;
static GPIO_PinState cs = GPIO_PIN_SET;
static unsigned scenario, transfers, delays, cs_writes;
static char output[1024];
static size_t output_size;
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
  assert(port == GPIOA && pin == GPIO_PIN_4 && state != cs);
  cs = state;
  cs_writes++;
}
void HAL_Delay(uint32_t ms)
{
  assert(ms == 10 && cs == GPIO_PIN_SET);
  assert(transfers > 0 && transfers < 6 && delays == transfers - 1);
  delays++;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *bus,
    uint8_t *tx, uint8_t *rx, uint16_t size, uint32_t timeout)
{
  unsigned i = transfers;
  assert(i < 6 && bus == &hspi1 && cs == GPIO_PIN_RESET);
  assert(delays == i && timeout == 20);
  assert(tx[0] == (i < 3 ? 0x9F : 0x90));
  assert(size == (i < 3 ? 4 : 6));
  memset(rx, 0xA5, size);
  memcpy(rx + (i < 3 ? 1 : 4), replies[scenario][i], i < 3 ? 3 : 2);
  transfers++;
  return results[scenario][i];
}
bool ConsoleTx_Write(const uint8_t *data, uint16_t size)
{
  assert(cs == GPIO_PIN_SET);
  assert(output_size + size < sizeof(output));
  memcpy(output + output_size, data, size);
  output_size += size;
  output[output_size] = 0;
  return HAL_OK;
}
'''
tables = '#include "main.h"\nstatic const HAL_StatusTypeDef results[3][6] = {'
tables += ','.join('{' + ','.join(str(s) for s, _ in c) + '}' for c in cases) + '};\n'
tables += 'static const uint8_t replies[3][6][3] = {'
tables += ','.join('{' + ','.join('{' + ','.join(map(str, d)) + '}' for _, d in c) + '}' for c in cases) + '};\n'
tables += 'static const char *expected[3] = {' + ','.join(json.dumps(x) for x in expected) + '};\n'
main = r'''
int main(void)
{
  for (scenario = 0; scenario < 3; scenario++)
  {
    transfers = delays = cs_writes = 0;
    output_size = 0;
    output[0] = 0;
    W25Q128_StartupCheck();
    assert(transfers == 6 && delays == 5 && cs_writes == 12);
    if (strcmp(output, expected[scenario]) != 0)
    {
      fprintf(stderr, "Unexpected startup log:\n%s", output);
      return 1;
    }
  }
  puts("w25q128_startup_test: six transactions, delays, invalid/error logs and prompt OK");
  return 0;
}
'''
path = OUT / 'w25q128_startup_test.c'
exe = OUT / 'w25q128_startup_test.exe'
path.write_text(tables + fixture + startup + main, encoding='utf-8')
subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                '-Itests/usb_stubs', '-Itemplate/Hardware', str(path),
                'template/Hardware/w25q128.c', '-o', str(exe)], cwd=ROOT, check=True)
subprocess.run([str(exe)], check=True)
