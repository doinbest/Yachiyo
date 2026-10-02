#include "contest_screen.h"
#include "tjc_screen.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t now_ms;
static char sent[16][96];
static unsigned sent_count;
static TJC_Status_t tx;
static int defer_tx;
bool TJC_Status_Get(TJC_Status_t *out) { *out = tx; return true; }

uint32_t HAL_GetTick(void) { return now_ms; }

HAL_StatusTypeDef TJC_Command_Send(const char *command)
{
  assert(sent_count < 16U);
  (void)snprintf(sent[sent_count++], sizeof(sent[0]), "%s", command);
  if (defer_tx) tx.tx_busy = true;
  else tx.tx_count++;
  return HAL_OK;
}

HAL_StatusTypeDef TJC_Text_Set(const char *object_name, const char *value)
{
  char command[96];
  (void)snprintf(command, sizeof(command), "%s.txt=\"%s\"", object_name, value);
  return TJC_Command_Send(command);
}

static void drain_screen(void)
{
  unsigned index;
  for (index = 0U; index < 16U; ++index)
  {
    now_ms += 100U;
    ContestScreen_Process();
  }
}

static int contains(const char *expected)
{
  unsigned index;
  for (index = 0U; index < sent_count; ++index)
  {
    if (strcmp(sent[index], expected) == 0) return 1;
  }
  return 0;
}

int main(void)
{
  unsigned index;
  ContestScreen_Init();
  drain_screen();
  assert(contains("main.tTaskTop.txt=\"---+---+\""));
  assert(contains("main.tTaskBottom.txt=\"---+---\""));
  assert(contains("main.tGrab.txt=\"正确抓取数：0\""));
  assert(contains("main.tPlace.txt=\"正确放置数：0\""));
  for (index = 0U; index < sent_count; ++index)
    assert(strstr(sent[index], "tState") == NULL);

  sent_count = 0U;
  assert(ContestScreen_TaskCodeSet("123+456+789+012") == 1U);
  drain_screen();
  assert(contains("main.tTaskTop.txt=\"123+456+\""));
  assert(contains("main.tTaskBottom.txt=\"789+012\""));

  sent_count = 0U;
  assert(ContestScreen_ProgressSet(2U, 1U) == 1U);
  drain_screen();
  assert(contains("main.tGrab.txt=\"正确抓取数：2\""));
  assert(contains("main.tPlace.txt=\"正确放置数：1\""));

  sent_count = 0U;
  assert(ContestScreen_TaskCodeSet("123+456+789+012") == 1U);
  drain_screen();
  assert(sent_count == 0U);
  assert(ContestScreen_TaskCodeSent());
  assert(ContestScreen_TaskCodeRefresh("156+123+516+231"));
  assert(!ContestScreen_TaskCodeSent());
  defer_tx = 1; now_ms += 100; ContestScreen_Process();
  assert(!ContestScreen_TaskCodeSent());
  /* A submitted DMA is not a completed field. */
  now_ms += 100; ContestScreen_Process(); assert(sent_count == 1);
  tx.tx_busy = false; tx.tx_count++; defer_tx = 0;
  drain_screen(); assert(ContestScreen_TaskCodeSent());
  /* Replacing a field while its previous DMA is in flight must resend it. */
  sent_count=0;assert(ContestScreen_TaskCodeRefresh("123+231+312+321"));
  defer_tx=1;now_ms+=100;ContestScreen_Process();
  assert(ContestScreen_TaskCodeSet("156+123+516+231"));
  tx.tx_busy=false;tx.tx_count++;defer_tx=0;
  drain_screen();
  assert(contains("main.tTaskTop.txt=\"156+123+\"") && ContestScreen_TaskCodeSent());
  sent_count = 0;
  assert(ContestScreen_TaskCodeRefresh("156+123+516+231"));
  defer_tx = 1; now_ms += 100; ContestScreen_Process();
  tx.tx_busy = false; tx.uart_error_count++;
  now_ms += 100; ContestScreen_Process();
  assert(ContestScreen_HasError() && !ContestScreen_TaskCodeSent());
  puts("contest_screen_test: PASS");
  return 0;
}
