#include "QR.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }
static void receive(const char *text) { while (*text) QR_ReceiveData((uint8_t)*text++); }
int main(void)
{
  QR_SnapshotTypeDef snapshot;
  char code[QR_TASK_CODE_BUFFER_SIZE];
  QR_Init(); QR_SnapshotGet(&snapshot);
  assert(!snapshot.Valid && !snapshot.Sequence && !snapshot.Received);
  QR_SnapshotGet(NULL); assert(!QR_TaskCodeGet(NULL));
  receive("123+231+"); QR_SnapshotGet(&snapshot); assert(!snapshot.Valid);
  tick = UINT32_MAX - 5U;
  receive("312+321\r\n");
  QR_SnapshotGet(&snapshot);
  assert(snapshot.Valid && snapshot.Sequence == 1 && snapshot.Accepted == 1);
  assert(snapshot.Received == 17 && snapshot.ReceivedTick == tick);
  assert(!strcmp(snapshot.Code,"123+231+312+321"));
  assert(QR_TaskCodeGet(code) && !strcmp(code,snapshot.Code));
  assert(!QR_TaskCodeGet(code));
  {
    QR_SnapshotTypeDef before;
    QR_SnapshotGet(&before);
    assert(QR_SimulatedSet("156+123+516+231"));
    QR_SnapshotGet(&snapshot);
    assert(snapshot.Source == QR_SOURCE_SIMULATED && snapshot.Sequence == before.Sequence + 1);
    assert(snapshot.Received == before.Received && snapshot.Accepted == before.Accepted);
    assert(snapshot.Rejected == before.Rejected);
    assert(QR_TaskCodeGet(code) && !strcmp(code, "156+123+516+231"));
    assert(QR_ColorGet(0, 0) == 1 && QR_ColorGet(0, 1) == 5 && QR_ColorGet(1, 0) == 5);
    assert(!QR_ColorGet(2, 0) && !QR_SimulatedSet("156+123+516+23x"));
    receive("123+231+312+321\r");
    QR_SnapshotGet(&snapshot); assert(snapshot.Source == QR_SOURCE_UART);
  }
  QR_Init(); receive("123+231+312+321\r\n"); QR_TaskCodeGet(code);
  QR_SnapshotGet(&snapshot); assert(snapshot.Valid && snapshot.Sequence == 1);
  tick = 4; assert((uint32_t)(tick - snapshot.ReceivedTick) == 10);
  receive("123+231+312+321\r");
  QR_SnapshotGet(&snapshot); assert(snapshot.Sequence == 2 && snapshot.Accepted == 2);
  receive("bad123+231+312+321\r123\r123+231+312+3210\r");
  QR_SnapshotGet(&snapshot); assert(snapshot.Rejected == 3 && snapshot.Accepted == 2);
  receive("123+"); QR_ErrorCallback(); receive("231+312+321\r");
  QR_SnapshotGet(&snapshot); assert(snapshot.UartErrors == 1 && snapshot.Rejected == 4);
  receive("111+222+333+444\r");
  QR_SnapshotGet(&snapshot); assert(snapshot.Sequence == 3);
  assert(QR_TaskCodeGet(code) && !strcmp(code,"111+222+333+444"));
  assert(!QR_TaskCodeGet(code));
  puts("qr: split frames, CRLF, snapshots, consumption, malformed frames, UART error, tick wrap PASS");
  return 0;
}
