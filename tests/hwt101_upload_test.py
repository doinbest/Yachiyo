"""Compile actual uploader/timing code against controlled calibration snapshots."""
from pathlib import Path
import re
import subprocess
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / '.embeddedskills/tests'
source = (ROOT / 'template/Core/Src/main.c').read_text(encoding='utf-8-sig')
upload = re.search(r'static void HWT101_Upload_Process\(void\)\s*\{.*?\n\}', source, re.S).group()
timing = source.split('/* IMU_TIMING_BEGIN:', 1)[1].split('/* IMU_TIMING_END */', 1)[0].split('*/', 1)[1]
fixture = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "hwt101_calibration.h"
#define CONSOLE_UPLOAD_TIMEOUT_MS 100U
#define HWT101_UPLOAD_INTERVAL_MS 200U

static HWT101_Angle_t HWT101_CurrentAngle, angle;
static uint8_t HWT101_AngleValid, stream, available;
static uint32_t HWT101_UploadLastTick, HWT101_ProgressLastTick;
static uint32_t HWT101_UploadRunId, HWT101_UploadFinalRunId;
static HWT101_CalState_t HWT101_UploadFinalState;
static HWT101_CalStatus_t status;
static HWT101_Status_t comm;
static uint32_t tick;
static char output[16000];
static unsigned progress_count, final_count, yaw_count, timing_lines;
uint32_t HAL_GetTick(void) {return tick;}
uint8_t ArmConsole_ImuStreamEnabled(void) {return stream;}
bool HWT101_Cal_GetStatus(HWT101_CalStatus_t *out) {*out=status;return true;}
bool HWT101_Status_Get(HWT101_Status_t *out) {*out=comm;return true;}
bool HWT101_Angle_Get(HWT101_Angle_t *out) {if(!available)return false;*out=angle;return true;}
bool HWT101_Angle_Is_Fresh(const HWT101_Angle_t *a,uint32_t max_age) {return tick-a->last_update_ms<=max_age;}
bool ConsoleTx_Write(const uint8_t *data,uint16_t size)
{
  char line[512];assert(size<sizeof(line));
  memcpy(line,data,size);line[size]=0;
  assert(strlen(output)+size<sizeof(output));strcat(output,line);
  if(strstr(line,"keep_still=1"))progress_count++;
  if(strstr(line,"[IMU CAL] result="))final_count++;
  if(strstr(line,"HWT101 yaw_deg="))yaw_count++;
  if(strstr(line,"[IMU TIME]"))timing_lines++;
  return HAL_OK;
}
'''
main = r'''
static void reset_output(void) {output[0]=0;}
static void state(HWT101_CalState_t value,const char *name,bool busy)
{status.state=value;status.state_name=name;status.busy=busy;}
int main(void)
{
  status.reason="none";status.result="NONE";status.save_state="NOT_REQUESTED";
  state(HWT101_CAL_IDLE,"IDLE",false);tick=200;HWT101_Upload_Process();assert(output[0]==0);
  stream=1;HWT101_Upload_Process();assert(strstr(output,"yaw_deg=NA") && !strstr(output,"corrected"));
  stream=0;reset_output();status.run_id=1;status.total_ms=50000;status.verify_ms=30000;status.result="PENDING";
  state(HWT101_CAL_CALIBRATING,"CALIBRATING",true);HWT101_Upload_Process();
  for(unsigned ms=5000;ms<=15000;ms+=5000){tick=200+ms;status.elapsed_ms=ms;HWT101_Upload_Process();}
  assert(progress_count==3 && final_count==0);
  tick=20200;status.elapsed_ms=20000;state(HWT101_CAL_VERIFYING,"VERIFYING",true);HWT101_Upload_Process();
  assert(final_count==0 && !strstr(output,"result=PASS"));
  tick=25200;HWT101_Upload_Process();assert(progress_count==5);
  stream=1;HWT101_Upload_Process();assert(yaw_count==1); /* busy suppresses stream even if enabled */
  stream=0;reset_output();state(HWT101_CAL_DONE,"DONE",false);status.result="PASS";status.verified=true;
  status.save_state="REQUESTED";status.save_requested=true;status.save_readback_ok=true;HWT101_Upload_Process();
  assert(final_count==1 && strstr(output,"result=PASS") && strstr(output,"save_state=REQUESTED"));
  assert(strstr(output,"save_requested=1") && strstr(output,"save_readback_ok=1") && strstr(output,"persistent=UNTESTED"));
  assert(strstr(output,"relative_deg=") && strstr(output,"verify_elapsed_ms="));
  assert(strstr(output,"arm> "));reset_output();HWT101_Upload_Process();assert(output[0]==0 && final_count==1);
  /* Same state, different run must report even when uploader missed the busy state. */
  status.run_id=2;HWT101_Upload_Process();assert(final_count==2);
  /* A new terminal failure on the same run is visible exactly once. */
  reset_output();state(HWT101_CAL_FAILED,"FAILED",false);status.result="FAIL";status.reason="gap";
  status.gap_ms=353;status.max_gap_ms=353;HWT101_Upload_Process();assert(final_count==3 && strstr(output,"gap_ms=353"));
  HWT101_Upload_Process();assert(final_count==3);
  stream=1;available=1;angle.yaw=-12.5f;angle.last_update_ms=tick;angle.update_count=42;reset_output();tick+=200;
  HWT101_Upload_Process();assert(strstr(output,"yaw_deg=-12.50") && strstr(output,"fresh=1"));
  assert(!strstr(output,"corrected") && !strstr(output,"bias_dps"));
  reset_output();tick+=400;HWT101_Upload_Process();assert(strstr(output,"fresh=0"));stream=0;
  /* Freeze prior-loop timing and preserve unsigned wrap arithmetic. */
  reset_output();status.run_id=3;state(HWT101_CAL_VERIFYING,"VERIFYING",true);tick=UINT32_MAX-10;
  ImuTiming_Begin();HWT101_Upload_Process();tick+=350;ImuTiming_Mark(IMU_TIME_OLED);ImuTiming_End();
  assert(ImuTiming.previous_loop_ms==350 && ImuTiming.maximum[IMU_TIME_OLED]==350);
  ImuTiming_Begin();tick+=2;ImuTiming_Mark(IMU_TIME_READ);tick++;ImuTiming_Mark(IMU_TIME_CAL);
  state(HWT101_CAL_RESTORING,"RESTORING",true);unsigned before=timing_lines;HWT101_Upload_Process();
  assert(timing_lines==before);tick+=100;ImuTiming_Mark(IMU_TIME_LOG);ImuTiming_End();
  ImuTiming_Begin();tick+=100;ImuTiming_Mark(IMU_TIME_CAL);ImuTiming_End();
  assert(ImuTiming.previous_loop_ms==350);
  state(HWT101_CAL_FAILED,"FAILED",false);HWT101_Upload_Process();
  assert(strstr(output,"loop_prev_ms=350") && strstr(output,"loop_partial_ms=3") && timing_lines==before+IMU_TIME_COUNT);
  tick+=100;ImuTiming_Mark(IMU_TIME_LOG);ImuTiming_End();assert(ImuTiming.previous_loop_ms==350);
  status.run_id=4;state(HWT101_CAL_VERIFYING,"VERIFYING",true);ImuTiming_Begin();
  assert(ImuTiming.max_loop_ms==0 && ImuTiming.maximum[IMU_TIME_OLED]==0);
  reset_output();state(HWT101_CAL_CANCELLED,"CANCELLED",false);status.result="CANCELLED";HWT101_Upload_Process();
  assert(strstr(output,"result=CANCELLED") && strstr(output,"arm> "));
  reset_output();status.run_id=0;unsigned finals_before=final_count;HWT101_Upload_Process();
  assert(final_count==finals_before+1 && strstr(output,"result=CANCELLED"));
  HWT101_Upload_Process();assert(final_count==finals_before+1);
  puts("hwt101_upload_test: native progress, final deduplication, raw telemetry and frozen timing OK");return 0;
}
'''
# Main integration ordering and all motion entry guards are part of the contract.
assert 'HWT101_Cal_Process();' in source
assert source.index('HWT101_Process();') < source.index('HWT101_Cal_Process();') < source.index('Mecanum_Velocity_Process();')
assert re.search(r'if\s*\(!HWT101_Cal_IsBusy\(\)\s*&&\s*!ChassisMotion_IsBusy\(\)\s*&&\s*!ChassisRoute_IsBusy\(\)\)\s*OledUi_Process\(\)', source)
color_start = re.search(r'static void ArmVision_ColorAlignStart\(void\).*?\n\}', source, re.S).group()
assert 'HWT101_Cal_IsBusy()' in color_start
assert 'HWT101_Drift_' not in source and 'HWT101_Store_' not in source
OUT.mkdir(parents=True, exist_ok=True)
path=OUT/'hwt101_upload_test.c';exe=OUT/'hwt101_upload_test.exe'
path.write_text(fixture+timing+upload+main,encoding='utf-8')
subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Itests/usb_stubs','-Itemplate/Hardware','-Itemplate/App',str(path),'-o',str(exe)],cwd=ROOT,check=True)
subprocess.run([str(exe)],check=True)
