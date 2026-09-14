#include "Camera.h"
#include "usbd_cdc_if.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t tx_result = USBD_OK;
static uint8_t usb_configured = 1U;
static uint8_t tx[4];
static uint32_t tick;

uint32_t HAL_GetTick(void) { return tick; }
uint8_t CDC_IsConfigured_FS(void) { return usb_configured; }
uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t length)
{
  assert(length == 4U);
  if (tx_result == USBD_OK) memcpy(tx, data, length);
  return tx_result;
}

static void make_frame(uint8_t *frame, uint8_t valid)
{
  const uint8_t base[] = {0xff,0xb2,3,1,1,0x4e,1,0x52,0xff,0xef,0,0x1c,0,0xfe};
  unsigned int index;
  memcpy(frame, base, sizeof(base));
  frame[3] = valid;
  for (index = 1U; index <= 11U; ++index) frame[12] += frame[index];
}

int main(void)
{
  Camera_DataTypeDef data;
  Camera_SnapshotTypeDef snapshot;
  uint8_t frame[CAMERA_FRAME_SIZE];
  uint8_t joined[CAMERA_FRAME_SIZE * 2U];
  uint8_t overflow[CAMERA_RX_BUFFER_SIZE + 1U] = {0};
  unsigned int split;

  tick = 100U;
  assert(Camera_Init() == HAL_OK);
  Camera_SnapshotGet(&snapshot);
  assert(!snapshot.RequestActive && snapshot.UsbConfigured);
  assert(Camera_VisualStateGet(&snapshot) == CAMERA_VIS_IDLE);
  usb_configured = 0U;
  Camera_SnapshotGet(&snapshot);
  assert(Camera_VisualStateGet(&snapshot) == CAMERA_VIS_IDLE);
  usb_configured = 1U;

  tx_result = USBD_FAIL;
  assert(Camera_MaterialStart(CAMERA_COLOR_BLUE) == HAL_ERROR);
  tx_result = USBD_BUSY;
  assert(Camera_MaterialStart(CAMERA_COLOR_BLUE) == HAL_BUSY);
  tx_result = USBD_OK;
  assert(Camera_MaterialStart(CAMERA_COLOR_BLUE) == HAL_OK);
  assert(memcmp(tx, "\xff\xb2\x03\xff", 4) == 0);
  Camera_SnapshotGet(&snapshot);
  assert(snapshot.RequestActive && snapshot.RequestFunction == 0xb2 && snapshot.RequestTarget == 3);
  assert(!snapshot.HasFrame && !snapshot.TargetValid && !snapshot.HasValidData);
  assert(Camera_VisualStateGet(&snapshot) == CAMERA_VIS_WAIT);
  usb_configured = 0U;
  Camera_SnapshotGet(&snapshot);
  assert(Camera_VisualStateGet(&snapshot) == CAMERA_VIS_OFF);
  usb_configured = 1U;

  make_frame(frame, 1U);
  for (split = 0; split <= sizeof(frame); ++split)
  {
    assert(Camera_MaterialStart(CAMERA_COLOR_BLUE) == HAL_OK);
    Camera_UsbRxCallback(frame, split);
    Camera_Process();
    Camera_UsbRxCallback(frame + split, sizeof(frame) - split);
    Camera_Process();
    Camera_SnapshotGet(&snapshot);
    assert(snapshot.HasFrame && snapshot.TargetValid && snapshot.HasValidData);
    assert(snapshot.Data.CX == 334 && snapshot.Data.CY == 338);
    assert(snapshot.Data.DX == -17 && snapshot.Data.DY == 28);
    assert(Camera_DataGet(&data) == 1U);
    Camera_SnapshotGet(&snapshot);
    assert(snapshot.HasValidData);
  }

  assert(Camera_MaterialStart(CAMERA_COLOR_BLUE) == HAL_OK);
  Camera_SnapshotGet(&snapshot);
  assert(!snapshot.HasFrame && !snapshot.TargetValid && !snapshot.HasValidData);

  memcpy(joined, frame, sizeof(frame));
  memcpy(joined + sizeof(frame), frame, sizeof(frame));
  joined[12] ^= 1U;
  tick = 200U;
  Camera_UsbRxCallback(joined, sizeof(joined));
  Camera_Process();
  Camera_SnapshotGet(&snapshot);
  assert(snapshot.ChecksumErrorCount == 1U);
  assert(snapshot.RxByteCount == (uint32_t)((sizeof(frame) * 15U) + sizeof(joined)));
  assert(snapshot.HasFrame && snapshot.LastFrameTick == 200U);
  assert(Camera_VisualStateGet(&snapshot) == CAMERA_VIS_OK);

  make_frame(frame, 0U);
  tick = 300U;
  Camera_UsbRxCallback(frame, sizeof(frame));
  Camera_Process();
  Camera_SnapshotGet(&snapshot);
  assert(snapshot.HasFrame && !snapshot.TargetValid && !snapshot.HasValidData);
  assert(snapshot.LastFrameTick == 300U);
  assert(snapshot.Data.CX == 334 && snapshot.Data.Sequence == 16U);
  assert(Camera_DataGet(&data) == 0U);
  assert(Camera_VisualStateGet(&snapshot) == CAMERA_VIS_LOST);
  tick = 1501U;
  assert(Camera_VisualStateGet(&snapshot) == CAMERA_VIS_STALE);

  Camera_UsbRxCallback(overflow, sizeof(overflow));
  Camera_Process();
  Camera_SnapshotGet(&snapshot);
  assert(snapshot.OverflowCount == 1U);
  assert(snapshot.RxByteCount == (uint32_t)((sizeof(frame) * 16U) + sizeof(joined) + sizeof(overflow)));

  assert(strcmp(Camera_VisualStateNameGet(CAMERA_VIS_IDLE), "Idle") == 0);
  assert(strcmp(Camera_VisualStateNameGet(CAMERA_VIS_OFF), "Off") == 0);
  assert(strcmp(Camera_VisualStateNameGet(CAMERA_VIS_WAIT), "Wait") == 0);
  assert(strcmp(Camera_VisualStateNameGet(CAMERA_VIS_OK), "Ok") == 0);
  assert(strcmp(Camera_VisualStateNameGet(CAMERA_VIS_LOST), "Lost") == 0);
  assert(strcmp(Camera_VisualStateNameGet(CAMERA_VIS_STALE), "Stale") == 0);

  Camera_RequestStop();
  Camera_SnapshotGet(&snapshot);
  assert(!snapshot.RequestActive && Camera_VisualStateGet(&snapshot) == CAMERA_VIS_IDLE);
  assert(Camera_RingStart(CAMERA_RING_1) == HAL_ERROR);
  /* A reserved Valid value must not make historical coordinates fresh again. */
  Camera_Init(); Camera_MaterialStart(CAMERA_COLOR_BLUE); tick = 2000;
  make_frame(frame, 1); Camera_UsbRxCallback(frame, sizeof(frame)); Camera_Process();
  tick = 4000; make_frame(frame, 2); Camera_UsbRxCallback(frame, sizeof(frame)); Camera_Process();
  Camera_SnapshotGet(&snapshot);
  assert(snapshot.LastFrameTick == 2000 && Camera_VisualStateGet(&snapshot) == CAMERA_VIS_STALE);
  /* The parser carry-over plus a full input chunk can also overflow. */
  Camera_MaterialStart(CAMERA_COLOR_BLUE);
  Camera_UsbRxCallback(frame, 5); Camera_Process();
  Camera_UsbRxCallback(overflow, CAMERA_RX_BUFFER_SIZE); Camera_Process();
  Camera_SnapshotGet(&snapshot); assert(snapshot.OverflowCount == 1);
  puts("camera USB snapshot, counters and visual-state tests passed");
  return 0;
}
