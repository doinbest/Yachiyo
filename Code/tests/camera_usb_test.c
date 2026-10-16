#include "Camera.h"
#include "usbd_cdc_if.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t tx_result = USBD_OK;
static uint8_t usb_configured = 1U;
static uint8_t tx[6];
static uint16_t tx_length;
static unsigned tx_calls;
static uint32_t tick;
static unsigned observed;
static uint8_t observed_valid[4], observed_match[4];
static uint32_t observed_tick[4];
static void observe_frame(const uint8_t *frame,uint32_t received,uint8_t matched)
{
  assert(observed<4);observed_valid[observed]=frame[3];
  observed_match[observed]=matched;observed_tick[observed++]=received;
}
static void pixel_observer_test(void);

uint32_t HAL_GetTick(void) { return tick; }
uint8_t CDC_IsConfigured_FS(void) { return usb_configured; }
uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t length)
{
  tx_calls++;
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
  Camera_SnapshotTypeDef snap; unsigned i; uint32_t seq;
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
    Camera_SnapshotGet(&snap); assert(!snap.HasFrame && !snap.HasValidData);
    Camera_UsbRxCallback(frames,14); Camera_Process(); Camera_SnapshotGet(&snap);
    assert(snap.HasValidData && snap.TargetValid);
  }
  /* Restart discards partial frames. B2 cannot distinguish a FULL delayed same-colour reply. */
  Camera_UsbRxCallback(frames,7); Camera_Process(); Camera_MaterialStart(CAMERA_COLOR_BLUE);
  Camera_UsbRxCallback(frames+7,7); Camera_Process(); Camera_SnapshotGet(&snap);
  assert(!snap.HasValidData && !snap.InvalidCount && !snap.HasFrame);
  tick=0xfffffff0U; Camera_UsbRxCallback(frames,14); Camera_Process(); Camera_SnapshotGet(&snap);
  tick=30; assert(Camera_VisualStateGet(&snap)==CAMERA_VIS_OK);
  tick=1300; assert(Camera_VisualStateGet(&snap)==CAMERA_VIS_STALE);
}

static void guarded_recovery_restart(void)
{
  uint8_t frame[14],retained;Camera_SnapshotTypeDef s;
  Camera_Init();tx_result=USBD_OK;tick=100;
  assert(Camera_MaterialStart(CAMERA_COLOR_BLUE)==HAL_OK);make_frame(frame,1);
  Camera_UsbRxCallback(frame,14);Camera_Process();unsigned count=tx_calls;
  assert(Camera_RestartIfMissing(3,90,250,&retained)==HAL_OK && retained && tx_calls==count);
  Camera_SnapshotGet(&s);assert(s.HasValidData);
  assert(Camera_RestartIfMissing(2,90,250,&retained)==HAL_ERROR && tx_calls==count);
  /* IRQ bytes not yet parsed belong to the original stream. */
  assert(Camera_MaterialStart(CAMERA_COLOR_BLUE)==HAL_OK);tick=200;
  Camera_UsbRxCallback(frame,14);count=tx_calls;
  assert(Camera_RestartIfMissing(3,190,250,&retained)==HAL_BUSY && tx_calls==count);
  tick=205;Camera_Process();
  assert(Camera_RestartIfMissing(3,190,250,&retained)==HAL_OK && retained && tx_calls==count);
  /* A sustained invalid stream can still request reacquisition. */
  make_frame(frame,0);Camera_UsbRxCallback(frame,14);Camera_Process();
  assert(Camera_RestartIfMissing(3,190,250,&retained)==HAL_OK && !retained && tx_calls==count+1);
  Camera_SnapshotGet(&s);assert(!s.HasFrame && !s.HasValidData);
  /* Parsing a pre-boundary frame late cannot qualify it as recovered. */
  make_frame(frame,1);tick=300;Camera_UsbRxCallback(frame,14);tick=500;Camera_Process();count=tx_calls;
  assert(Camera_RestartIfMissing(3,350,250,&retained)==HAL_OK && !retained && tx_calls==count+1);
  tick=UINT32_MAX-10;Camera_UsbRxCallback(frame,14);Camera_Process();tick=5;count=tx_calls;
  assert(Camera_RestartIfMissing(3,UINT32_MAX-20,250,&retained)==HAL_OK && retained && tx_calls==count);
  Camera_RequestStop();count=tx_calls;
  assert(Camera_RestartIfMissing(3,0,250,&retained)==HAL_ERROR && tx_calls==count);
}
int main(void)
{
  guarded_recovery_restart();
  pixel_observer_test();
  Camera_SnapshotTypeDef snapshot;
  Camera_SnapshotTypeDef consumer;
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
    /* A second consumer reads the same frame without consuming it. */
    Camera_SnapshotGet(&consumer);
    assert(consumer.HasValidData && consumer.TargetValid);
    assert(consumer.Data.Sequence == snapshot.Data.Sequence);
    assert(consumer.Data.CX == snapshot.Data.CX && consumer.Data.CY == snapshot.Data.CY);
    assert(consumer.Data.DX == snapshot.Data.DX && consumer.Data.DY == snapshot.Data.DY);
    Camera_SnapshotGet(&snapshot);
    assert(snapshot.HasValidData && snapshot.Data.Sequence == consumer.Data.Sequence);
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
  Camera_Init(); Camera_MaterialStart(CAMERA_COLOR_BLUE);
  make_frame(frame, 1); tick=100; Camera_UsbRxCallback(frame, 7); Camera_Process();
  tick=110; Camera_UsbRxCallback(frame+7, 7);
  tick=130; Camera_Process(); Camera_SnapshotGet(&snapshot);
  assert(snapshot.Data.Tick==130 && snapshot.Data.HasReceiveTime && snapshot.Data.ReceivedTick==110);
  tick=140; Camera_UsbRxCallback(frame, sizeof(frame));
  tick=150; Camera_UsbRxCallback(frame, sizeof(frame));
  tick=160; Camera_Process(); Camera_SnapshotGet(&snapshot);
  assert(snapshot.Data.Tick==160 && snapshot.Data.ReceivedTick==150);
  for(uint8_t ring=1;ring<=3;ring++) {
    uint8_t reply[14]; Camera_SnapshotTypeDef snapshot;
    assert(Camera_RingStart(ring)==HAL_OK);
    assert(tx_length==4 && tx[0]==0xff && tx[1]==0xb2 && tx[2]==9+ring && tx[3]==0xff);
    make_frame(reply,1); reply[2]=9+ring;reply[12]=0;
    for(unsigned j=1;j<=11;j++)reply[12]+=reply[j];
    Camera_UsbRxCallback(reply,14);Camera_Process();Camera_SnapshotGet(&snapshot);
    assert(snapshot.TargetValid && snapshot.Data.Target==9+ring);
  }
  assert(Camera_RingStart(0)==HAL_ERROR && Camera_RingStart(4)==HAL_ERROR);
  assert(Camera_MaterialStart((Camera_ColorTypeDef)10)==HAL_ERROR);
  /* 0D never publishes angles/STOP as ordinary DX/DY. */
  Camera_Init();
  assert(Camera_PoseStart()==HAL_OK && tx[2]==0x0d);
  uint8_t pose_reply[14]={0xff,0xb2,0x0d,1,0xff,0xec,0,10,0xff,0xce,0,1,0,0xfe};
  for(unsigned j=1;j<=11;j++)pose_reply[12]+=pose_reply[j];
  tick=250;Camera_UsbRxCallback(pose_reply,7);Camera_Process();
  tick=260;Camera_UsbRxCallback(pose_reply+7,7);Camera_Process();
  Camera_PoseSnapshot_t pose;Camera_PoseSnapshotGet(&pose);Camera_SnapshotGet(&snapshot);
  assert(pose.Valid && pose.EX==-20 && pose.EY==10 && pose.AngleCdeg==-50 && pose.Stop==1);
  assert(pose.ReceivedTick==260 && pose.Sequence && !snapshot.HasValidData);
  uint32_t seq=pose.Sequence;
  pose_reply[3]=0;pose_reply[12]--;Camera_UsbRxCallback(pose_reply,14);Camera_Process();
  Camera_PoseSnapshotGet(&pose);assert(!pose.Valid && pose.Sequence!=seq);
  assert(Camera_RingStart(1)==HAL_OK);Camera_PoseSnapshotGet(&pose);assert(!pose.Valid);
  /* A black-line reply cannot satisfy a ring request, or leak into DX/DY. */
  assert(Camera_PoseStartSource(0x0e)==HAL_OK && tx[2]==0x0e);
  Camera_PoseSnapshotGet(&pose);assert(pose.Active && pose.Target==0x0e && !pose.HasFrame);
  pose_reply[3]=1;pose_reply[12]++;
  Camera_UsbRxCallback(pose_reply,14);Camera_Process();
  Camera_PoseSnapshotGet(&pose);assert(!pose.HasFrame);
  pose_reply[2]=0x0e;pose_reply[12]++;
  tick=400;Camera_UsbRxCallback(pose_reply,5);Camera_Process();
  tick=410;Camera_UsbRxCallback(pose_reply+5,9);Camera_Process();
  Camera_PoseSnapshotGet(&pose);Camera_SnapshotGet(&snapshot);
  assert(pose.Valid && pose.Target==0x0e && pose.AngleCdeg==-50 && pose.ReceivedTick==410);
  assert(!snapshot.HasValidData);
  assert(Camera_PoseStartSource(0x01)==HAL_ERROR);
  assert(Camera_PoseStart()==HAL_OK && tx[2]==0x0d);
  Camera_UsbRxCallback(pose_reply,14);Camera_Process();
  Camera_PoseSnapshotGet(&pose);assert(!pose.HasFrame && pose.Target==0x0d);
  request_cache_test();
  b2_boundaries();
  puts("camera USB snapshot, counters, B2 protocol and visual-state tests passed");
  return 0;
}

static void pixel_observer_test(void)
{
  uint8_t frames[42];
  Camera_Init();Camera_FrameObserverSet(observe_frame);Camera_MaterialStart(CAMERA_COLOR_BLUE);
  observed=0;tick=12;
  make_frame(frames,1);make_frame(frames+14,0);make_frame(frames+28,1);
  frames[30]=1;frames[40]-=2; /* A valid reply for a different target still belongs in the raw log. */
  Camera_UsbRxCallback(frames,42);tick=20;Camera_Process();
  assert(observed==3 && observed_valid[0]==1 && observed_valid[1]==0);
  assert(observed_match[0]==1 && observed_match[1]==1 && observed_match[2]==0);
  assert(observed_tick[0]==12 && observed_tick[2]==12);
  frames[12]++;Camera_UsbRxCallback(frames,14);Camera_Process();assert(observed==3);
  Camera_FrameObserverSet(NULL);Camera_UsbRxCallback(frames+14,14);Camera_Process();assert(observed==3);
}
