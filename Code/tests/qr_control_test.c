#define main grab_original_main
#include "grab_task_test.c"
#undef main
static void scanning_setup(void)
{
  reset();configure(true);setting("z_min","-100");setting("qr_z_min","-50");
  setting("qr_z_speed","5");RearReferenceValid=true;
  memset(&qr_snapshot,0,sizeof qr_snapshot);qr_snapshot.Sequence=10;
  Operation=OP_STORE;InventoryTouched=true;Status.slot=2;
  assert(GrabTask_QrScanStart(10));
  assert(!InventoryTouched && Operation==OP_ALIGN && Status.slot==0);
}
int main(void)
{
  reset();configure(true);RearReferenceValid=true;setting("z_min","-20");
  assert(!GrabTask_QrScanStart(0) && !positions);
  scanning_setup();
  for(unsigned i=0;i<15;i++)step(false,0,0,0);
  assert(!positions && !base_moves && !closures && !openings);
  for(unsigned i=0;i<50 && !positions;i++)step(false,0,0,0);
  assert(positions==1 && z_position_pulses==-5000 && z_position_rpm==9);
  assert(!base_moves && destination[0]==0 && !closures && !openings);
  for(unsigned i=0;i<80 && positions<2;i++)step(false,0,0,0);
  assert(positions==2 && z_position_pulses==0 && z_position_rpm==9);
  for(unsigned i=0;i<80;i++)step(false,0,0,0);
  assert(positions==2 && fabsf(physical[1])<.1f && !base_moves);
  assert(status().state==GRAB_QR_SCAN && !strcmp(Status.reason,"qr_not_found"));
  unsigned before=positions;
  qr_snapshot.Valid=1;qr_snapshot.Source=QR_SOURCE_SIMULATED;qr_snapshot.Sequence=11;
  step(false,0,0,0);assert(status().state==GRAB_QR_SCAN && positions==before);
  qr_snapshot.Source=QR_SOURCE_UART;strcpy(qr_snapshot.Code,"156+123+516+231");
  step(false,0,0,0);run_to(GRAB_IDLE,100,0);
  assert(Status.stop_confirmed && !strcmp(Status.result,"qr_scanned"));
  scanning_setup();Config[C_Z_MAX]=0;physical[1]=.05f;
  for(unsigned i=0;i<100 && positions<2;i++)step(false,0,0,0);
  assert(positions==2 && z_position_pulses==0);
  GrabTask_Stop();run_to(GRAB_IDLE,100,0);
  scanning_setup();
  physical[1]=-10;
  for(unsigned i=0;i<100 && positions<2;i++)step(false,0,0,0);
  assert(positions==2 && z_position_pulses==-1000);
  qr_snapshot.Valid=1;qr_snapshot.Source=QR_SOURCE_UART;qr_snapshot.Sequence=11;
  step(false,0,0,0);run_to(GRAB_IDLE,100,0);
  assert(Status.stop_confirmed && !strcmp(Status.result,"qr_scanned"));
  scanning_setup();qr_snapshot.Valid=1;qr_snapshot.Source=QR_SOURCE_UART;
  /* Cached pre-run UART data must not finish the new scan. */
  for(unsigned i=0;i<10;i++)step(false,0,0,0);
  assert(status().state==GRAB_QR_SCAN && !positions);
  qr_snapshot.Sequence=12;step(false,0,0,0);run_to(GRAB_IDLE,100,0);
  assert(!positions && Status.stop_confirmed);
  scanning_setup();move_arrives=false;
  for(unsigned i=0;i<60 && !positions;i++)step(false,0,0,0);
  assert(positions==1 && physical[1]==0 && !strcmp(Status.reason,"qr_descending"));
  qr_snapshot.Valid=1;qr_snapshot.Source=QR_SOURCE_UART;qr_snapshot.Sequence=11;
  step(false,0,0,0);assert(status().state==GRAB_STOPPING);
  run_to(GRAB_IDLE,100,0);assert(Status.stop_confirmed && !strcmp(Status.result,"qr_scanned"));
  scanning_setup();GrabTask_Stop();run_to(GRAB_IDLE,100,0);
  before=positions;for(unsigned i=0;i<50;i++)step(false,0,0,0);
  assert(positions==before && !strcmp(Status.result,"cancelled"));
  puts("qr_control_test: PASS (wait, Z-only descent, lower limit, new UART code, stop)");
}
