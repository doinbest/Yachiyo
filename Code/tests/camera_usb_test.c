#include "Camera.h"
#include "usbd_cdc_if.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t tx_result = USBD_OK;
static uint8_t usb_configured = 1U;
static uint8_t tx[6];
static uint16_t tx_length;
static uint32_t tick;

uint32_t HAL_GetTick(void) { return tick; }
uint8_t CDC_IsConfigured_FS(void) { return usb_configured; }
uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t length)
{
  assert(length == 4U || length == 6U);
  tx_length = length;
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

static void request_cache_test(void)
{
  uint8_t frame[14]; Camera_SnapshotTypeDef snapshot;
  Camera_Init(); Camera_MaterialStart(CAMERA_COLOR_BLUE);
  make_frame(frame,1); Camera_UsbRxCallback(frame,14); Camera_Process();
  Camera_MaterialStart(CAMERA_COLOR_BLUE); Camera_SnapshotGet(&snapshot);
  assert(!snapshot.HasValidData && snapshot.Data.Sequence==0 && snapshot.Data.CX==0);
  Camera_UsbRxCallback(frame,14); Camera_Process(); Camera_RequestStop();
  Camera_SnapshotGet(&snapshot); assert(snapshot.Data.Sequence==0 && snapshot.LastFrameTick==0);
}

static void b2_boundaries(void)
{
  uint8_t frames[42], bad[14], b4[23]={0xff,0xb4,3,1,1,0x4e,1,0x52,0xff,0xef,0,0x1c,2,0,0,0,1,0,0,0,1,0,0xfe};
  Camera_SnapshotTypeDef snap; Camera_DataTypeDef data; unsigned i; uint32_t seq;
  Camera_Init(); assert(Camera_MaterialStart(CAMERA_COLOR_BLUE)==HAL_OK);
  assert(tx_length==4 && !memcmp(tx,"\xff\xb2\x03\xff",4));
  make_frame(frames,1); make_frame(frames+14,0); make_frame(frames+28,1);
  Camera_UsbRxCallback(frames,42); Camera_Process(); Camera_SnapshotGet(&snap);
  assert(snap.InvalidCount==1 && snap.HasValidData && snap.Data.Sequence==2);
  seq=snap.Data.Sequence;
  Camera_UsbRxCallback(frames,14); Camera_Process(); Camera_SnapshotGet(&snap);
  assert(snap.Data.Sequence==seq+1); /* Equal coordinates are distinct reports. */
  tx_result=USBD_BUSY; assert(Camera_MaterialStart(CAMERA_COLOR_RED)==HAL_BUSY);
  tx_result=USBD_FAIL; assert(Camera_MaterialStart(CAMERA_COLOR_RED)==HAL_ERROR);
  Camera_SnapshotGet(&snap); assert(snap.RequestTarget==3 && snap.InvalidCount==1 && snap.HasValidData);
  tx_result=USBD_OK;
  memcpy(bad,frames,14); bad[2]=1; bad[12]-=2; /* Valid checksum, wrong colour. */
  Camera_UsbRxCallback(bad,14); Camera_Process(); Camera_SnapshotGet(&snap); assert(snap.Data.Sequence==seq+1);
  for(i=1;i<21;i++) b4[21]+=b4[i];
  for(i=0;i<=23;i++) {
    Camera_MaterialStart(CAMERA_COLOR_BLUE);
    Camera_UsbRxCallback(b4,i); Camera_Process(); Camera_UsbRxCallback(b4+i,23-i); Camera_Process();
    Camera_SnapshotGet(&snap); assert(!snap.HasFrame && !Camera_DataGet(&data));
    Camera_UsbRxCallback(frames,14); Camera_Process(); assert(Camera_DataGet(&data));
  }
  /* Restart discards partial frames. B2 cannot distinguish a FULL delayed same-colour reply. */
  Camera_UsbRxCallback(frames,7); Camera_Process(); Camera_MaterialStart(CAMERA_COLOR_BLUE);
  Camera_UsbRxCallback(frames+7,7); Camera_Process(); assert(!Camera_DataGet(&data));
  Camera_SnapshotGet(&snap); assert(!snap.InvalidCount && !snap.HasFrame);
  tick=0xfffffff0U; Camera_UsbRxCallback(frames,14); Camera_Process(); Camera_SnapshotGet(&snap);
  tick=30; assert(Camera_VisualStateGet(&snap)==CAMERA_VIS_OK);
  tick=1300; assert(Camera_VisualStateGet(&snap)==CAMERA_VIS_STALE);
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
  request_cache_test();
  b2_boundaries();
  puts("camera USB snapshot, counters, B2 protocol and visual-state tests passed");
  return 0;
}
