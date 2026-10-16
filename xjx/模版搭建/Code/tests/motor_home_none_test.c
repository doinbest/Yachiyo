/* Receive/Both on cold drives: no enable/HOME may precede verified RAM None. */
#define main old_lazy_main
#include "motor_bus_lazy_config_test.c"
#undef main
#include <stdlib.h>
static uint8_t mode[8],saved[8][33];
static unsigned reads[8],writes[8],enables,homes,handled;
static bool verified[8],drop;
static void step(void)
{
  if(sends!=handled){
    unsigned id=frame[0]-1;uint8_t r[33]={frame[0],frame[1],2,0x6b};unsigned n=4;
    assert(id>=4 && id<8);handled=sends;
    if(frame[1]==0x42){
      memcpy(r,saved[id],33);r[22]=mode[id];reads[id]++;n=drop?0:33;
      if(mode[id]==0 && writes[id])verified[id]=true;
    }else if(frame[1]==0x48){
      assert(frame[2]==0xd1 && frame[3]==0 && frame[22]==0);
      for(unsigned i=4;i<32;i++)if(i!=22)assert(frame[i]==saved[id][i]);
      mode[id]=0;writes[id]++;
    }else if(frame[1]==0xf3 || frame[1]==0x9a){
      assert(mode[id]==0 && verified[id] && reads[id]>=2);
      if(frame[1]==0xf3)enables++;else homes++;n=0;
    }else if(frame[1]==0x35){n=6;r[2]=r[3]=r[4]=0;r[5]=0x6b;}
    else if(frame[1]==0x36){memset(r+2,0,5);n=8;r[7]=0x6b;}
    else if(frame[1]==0x3a || frame[1]==0x3b){r[2]=3;}
    else {assert(frame[1]==0xfe || frame[1]==0x9c);n=0;}
    MotorBus_TxCpltCallback(&uart);if(n)MotorBus_RxBytes(r,n);
  }
  tick+=3;MotorBus_Process();MechanicalArm_Process();Turntable_Process();
}
int main(int argc,char **argv)
{
  const char *kind=argc>1?argv[1]:"home6";
  Turntable_Init();assert(MotorBus_Init(&uart)==HAL_OK && MechanicalArm_Init(&uart)==HAL_OK);
  for(unsigned id=4;id<8;id++){
    uint8_t *c=saved[id];c[0]=id+1;c[1]=0x42;c[2]=33;c[3]=21;
    for(unsigned i=4;i<32;i++)c[i]=(uint8_t)(i+id);
    c[22]=mode[id]=(id&1)?3:1;c[32]=0x6b;
  }
  for(unsigned i=0;i<20;i++)step();
  assert(!sends);
  drop=strstr(kind,"fail")!=NULL;
  if(!strcmp(kind,"parallel")){
    assert(Turntable_Home());unsigned phase=0;
    for(unsigned i=0;i<4000 && (phase<4 || Turntable_IsBusy());i++){
      MechanicalArm_EventTypeDef e;
      if(!phase && MechanicalArm_Enable(MECHANICAL_ARM_AXIS_Z)==MECHANICAL_ARM_RESULT_NONE)phase=1;
      else if(phase==1 && MechanicalArm_ResultGet(&e)){assert(e.Result==MECHANICAL_ARM_RESULT_OK);phase=2;}
      else if(phase==2 && MechanicalArm_Home(MECHANICAL_ARM_AXIS_Z,2)==MECHANICAL_ARM_RESULT_NONE)phase=3;
      else if(phase==3 && MechanicalArm_ResultGet(&e)){assert(e.Result==MECHANICAL_ARM_RESULT_OK);phase=4;}
      step();
    }
    assert(phase==4 && !Turntable_IsBusy() && !MotorBus_IsQuarantined());
    assert(enables==2 && homes==2 && writes[5]==1 && writes[7]==1);
    puts("parallel Z enable/HOME and ID8 HOME release configuration ownership PASS");return 0;
  }
  if(!strcmp(kind,"stop") || !strcmp(kind,"disable") || !strcmp(kind,"abort")){
    MechanicalArm_ResultTypeDef r=!strcmp(kind,"stop")?MechanicalArm_Stop(MECHANICAL_ARM_AXIS_Z):
      !strcmp(kind,"disable")?MechanicalArm_Disable(MECHANICAL_ARM_AXIS_Z):MechanicalArm_HomeAbort(MECHANICAL_ARM_AXIS_Z);
    assert(r==MECHANICAL_ARM_RESULT_NONE);MotorBus_Process();
    assert(sends==1 && frame[0]==6 && frame[1]==(!strcmp(kind,"stop")?0xfe:!strcmp(kind,"disable")?0xf3:0x9c));
    assert(!writes[5] && !reads[5]);puts("stop/disable/HOME_ABORT do not wait for configuration PASS");return 0;
  }
  if(!strncmp(kind,"tray",4)){
    assert(!strcmp(kind,"tray_enable")?Turntable_Enable(true):Turntable_Home());
    for(unsigned i=0;i<3000 && Turntable_IsBusy();i++)step();
    assert(!Turntable_IsBusy());
    if(drop)assert(!enables && !homes && MotorBus_IsQuarantined());
    else assert(enables==1 && (!strcmp(kind,"tray_enable") || homes==1) && writes[7]==1);
  }else{
    bool enable=!strncmp(kind,"enable",6),all=strstr(kind,"all")!=NULL;
    unsigned id=all?5:(unsigned)atoi(kind+(enable?6:4));
    MechanicalArm_AxisTypeDef axis=all?MECHANICAL_ARM_AXIS_ALL:(MechanicalArm_AxisTypeDef)(id-5);
    MechanicalArm_ResultTypeDef result=MECHANICAL_ARM_RESULT_BUSY;
    for(unsigned i=0;i<3000 && result==MECHANICAL_ARM_RESULT_BUSY;i++){
      result=enable?MechanicalArm_Enable(axis):MechanicalArm_Home(axis,2);step();
    }
    if(drop){assert(result==MECHANICAL_ARM_RESULT_TX_ERROR && !enables && !homes);}
    else{
      assert(result==MECHANICAL_ARM_RESULT_NONE);
      MechanicalArm_EventTypeDef e;bool done=false;
      for(unsigned i=0;i<3000 && !done;i++){step();done=MechanicalArm_ResultGet(&e);}
      assert(done && e.Result==MECHANICAL_ARM_RESULT_OK);
      assert((enable?enables:homes)==(all?3U:1U));
    }
  }
  puts("motor_home_none_test: verified RAM None precedes enable/HOME PASS");
}
