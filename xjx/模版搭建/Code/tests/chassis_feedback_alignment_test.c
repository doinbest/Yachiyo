#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "mecanum_chassis.h"
#include "motor_bus.h"
#include "chassis_motion.h"
#include "chassis_localization.h"
#include "hwt101_calibration.h"
static uint32_t now_ms,tx_due,reply_due,applied_ms,last_plan_ms;
static UART_HandleTypeDef uart;
static uint8_t pending_frame[40],pending_len,reply[8],reply_size;
static bool transmitting,reply_pending;
static int cached[4],actual_rpm;
static double true_position;
static unsigned batches,writes,failures,queries;
static float planned_v;
static unsigned ack_delay=2;
static unsigned profile_mode;
static int queried_rpm[4];
static uint32_t speed_tx[4],position_tx[4];
static unsigned valid_targets;
static ChassisMotion_Status_t heading;
uint32_t HAL_GetTick(void){return now_ms;}
bool ChassisRoute_IsBusy(void){return false;}
bool ChassisMotion_IsBusy(void){return false;}
bool HWT101_Cal_IsBusy(void){return false;}
bool HWT101_Cal_ControlAngleGet(HWT101_Angle_t *a){(void)a;return false;}
void ChassisMotion_StatusGet(ChassisMotion_Status_t *out){*out=heading;out->heading_sample_tick=now_ms;}
bool ChassisMotion_AnchorSet(float yaw){heading.heading_valid=heading.map_anchor_valid=true;heading.map_yaw_rad=yaw;return true;}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n){(void)u;(void)p;(void)n;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u){(void)u;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n){
 (void)u;assert(!transmitting);memcpy(pending_frame,p,n);pending_len=(uint8_t)n;transmitting=true;tx_due=now_ms+1;return HAL_OK;
}
static void complete_wire(void){
 const uint8_t *p=pending_frame;unsigned id=p[0];transmitting=false;MotorBus_TxCpltCallback(&uart);
 reply[0]=id?id:1;reply[1]=p[1];reply_size=4;reply[2]=2;reply[3]=0x6b;
 if(p[1]==0xf6){
  assert(pending_len==8&&p[6]==1&&id>=1&&id<=4);assert(id==writes%4+1);
  cached[id-1]=(p[2]!=(id>=3))?-(int)((p[3]<<8)|p[4]):(int)((p[3]<<8)|p[4]);writes++;
 }else if(p[1]==0xff){
  assert(pending_len==4&&id==0&&writes%4==0);
  for(unsigned w=1;w<4;w++)if(cached[w]!=cached[0])failures++;
  actual_rpm=cached[0];applied_ms=now_ms;batches++;
 }else if(pending_len==3){
  assert(id>=1&&id<=4);queries++;
  if(p[1]==0x35){
   queried_rpm[id-1]=actual_rpm;speed_tx[id-1]=now_ms;
   int raw=id>=3?-actual_rpm:actual_rpm;unsigned value=(unsigned)abs(raw);
   reply[2]=raw<0;reply[3]=(uint8_t)(value>>8);reply[4]=(uint8_t)value;reply[5]=0x6b;reply_size=6;
  }else if(p[1]==0x36){
   position_tx[id-1]=now_ms;
   int64_t units=(int64_t)llround(true_position*65536/(3.141592653589793*80));
   if(id>=3)units=-units;
   uint32_t magnitude=(uint32_t)llabs(units);
   reply[2]=units<0;reply[3]=magnitude>>24;reply[4]=magnitude>>16;reply[5]=magnitude>>8;reply[6]=magnitude;reply[7]=0x6b;reply_size=8;
  }else{assert(p[1]==0x3a);reply[2]=3;}
 }else assert(0);
 reply_pending=true;reply_due=now_ms+ack_delay;
}
static void tick(void){
 now_ms++;true_position+=actual_rpm*(3.141592653589793*80/60)/1000;
 if(transmitting&&now_ms>=tx_due)complete_wire();
 if(reply_pending&&now_ms>=reply_due){reply_pending=false;MotorBus_RxBytes(reply,reply_size);}
 MotorBus_Process();ChassisLocalization_Process();Mecanum_Process();MotorBus_Process();
}
static float profile(unsigned t){
 if(profile_mode==1)return 1000;
 if(profile_mode==2){
  if(t<3334)return fminf(1000,t*.3f);
  if(t<4000)return 1000;
  return fmaxf(0,1000-(t-4000)*.3f);
 }
 if(t<400)return 500*(1-cosf(3.141592653589793f*t/400));
 if(t<800)return 1000;
 if(t<1200)return 500*(1+cosf(3.141592653589793f*(t-800)/400));
 return 0;
}
int main(int argc,char **argv){
 if(argc>1)ack_delay=(unsigned)atoi(argv[1]);
 if(argc>2)profile_mode=(unsigned)atoi(argv[2]);
 assert(MotorBus_Init(&uart)==HAL_OK);assert(Mecanum_Feedback_Select(0));Mecanum_Feedback_Enable(true);
 ChassisLocalization_Init();assert(ChassisLocalization_Origin(0,0,0));
 if(profile_mode==1)assert(Mecanum_Velocity_Request(1000,0,0,0));
 for(unsigned i=0;i<1000;i++)tick();
 uint32_t start=now_ms,seen[4]={0},position_seen[4]={0},group_seen[4]={0};
 for(unsigned w=0;w<4;w++) {
  Mecanum_Feedback_t f;assert(Mecanum_FeedbackGet(w,&f));
  position_seen[w]=f.position_ms;seen[w]=group_seen[w]=f.speed_ms;
 }
 float worst_vy=0,worst_group=0,max_y=0,max_plan_vs_sent=0;
 uint32_t max_span=0,max_group_span=0,max_batch_lag=0;
 unsigned duration=profile_mode==2?8000:1800;
 uint32_t initial_group=0;
 {ChassisLocalization_Status_t loc;ChassisLocalization_Get(&loc);initial_group=loc.feedback_sequence;}
 for(unsigned t=0;t<duration;t++){
  if(t%20==0){planned_v=profile(t);last_plan_ms=now_ms;assert(Mecanum_Velocity_Request(planned_v,0,0,0));}
  tick();Mecanum_Status_t s;Mecanum_StatusGet(&s);assert(!s.locked);
  Mecanum_Feedback_t f[4];bool changed=false,full=true,new_group=true;float logical[4];uint32_t oldest=UINT32_MAX,newest=0;
  for(unsigned w=0;w<4;w++){
   assert(Mecanum_FeedbackGet(w,&f[w]));
   if(f[w].speed_valid && f[w].speed_ms!=seen[w]) {
    assert(f[w].speed_tx_ms==speed_tx[w]);
    if(f[w].speed_target_valid) {
     assert(f[w].speed_target_rpm==queried_rpm[w]);
     assert((int32_t)(f[w].speed_tx_ms-f[w].speed_target_ms)>=0);
     valid_targets++;
    }
   }
   if(f[w].position_valid && f[w].position_ms!=position_seen[w])
    assert(f[w].position_tx_ms==position_tx[w]);
   position_seen[w]=f[w].position_ms;
   changed|=f[w].speed_ms!=seen[w];seen[w]=f[w].speed_ms;
   full&=f[w].speed_valid;new_group&=f[w].speed_ms!=group_seen[w];
   logical[w]=(w>=2?-1:1)*f[w].speed_rpm;
   if(f[w].speed_ms<oldest)oldest=f[w].speed_ms;
   if(f[w].speed_ms>newest)newest=f[w].speed_ms;
  }
  ChassisLocalization_Status_t loc;ChassisLocalization_Get(&loc);
  if(fabsf(loc.observer.feedback.y_mm)>max_y)max_y=fabsf(loc.observer.feedback.y_mm);
  float vy=(-logical[0]+logical[1]-logical[2]+logical[3])*(3.141592653589793f*80/60)/4;
  if(full&&changed){
   if(fabsf(vy)>worst_vy)worst_vy=fabsf(vy);
   if(newest-oldest>max_span)max_span=newest-oldest;
   if(new_group){
    for(unsigned w=0;w<4;w++)group_seen[w]=f[w].speed_ms;
    if(fabsf(vy)>worst_group)worst_group=fabsf(vy);
    if(newest-oldest>max_group_span)max_group_span=newest-oldest;
   }
  }
  float gap=fabsf(planned_v-s.sent_rpm[0]*(3.141592653589793f*80/60));if(gap>max_plan_vs_sent)max_plan_vs_sent=gap;
  if(s.sent_ms&&now_ms-s.sent_ms>max_batch_lag&&t>100&&t<1200)max_batch_lag=now_ms-s.sent_ms;
 }
 assert(failures==0);assert(batches>5);assert(last_plan_ms>start);
 if(profile_mode==1){assert(worst_vy<.01f);assert(max_y<.1f);}
 else{assert(worst_vy>1);}
 if(profile_mode==0) assert(max_y<(ack_delay==2?2:4));
 if(profile_mode==2) assert(max_y<.3f);
 {ChassisLocalization_Status_t loc;ChassisLocalization_Get(&loc);
  printf("alignment integration: groups=%lu x=%.3f Y=%.3f query/target matches=%u\n",
      (unsigned long)(loc.feedback_sequence-initial_group),loc.observer.feedback.x_mm,
      loc.observer.feedback.y_mm,valid_targets);
  assert(loc.feedback_sequence>initial_group && valid_targets>10);
  if(profile_mode==0) assert(loc.observer.feedback.x_mm>500);}
 printf("SUMMARY reply_delay=%u profile=%u true_lateral_mm_s=0 true_lateral_position_mm=0 batches=%u partial_or_mixed_batches=%u queries=%u apparent_vy_max=%.3f complete_speed_group_apparent_vy_max=%.3f estimated_y_max=%.3f speed_sample_span_max_ms=%lu complete_speed_group_span_max_ms=%lu planned_vs_last_sent_max_mm_s=%.3f last_sent_age_max_ms=%lu\n",ack_delay,profile_mode,batches,failures,queries,worst_vy,worst_group,max_y,(unsigned long)max_span,(unsigned long)max_group_span,max_plan_vs_sent,(unsigned long)max_batch_lag);
 return 0;
}
