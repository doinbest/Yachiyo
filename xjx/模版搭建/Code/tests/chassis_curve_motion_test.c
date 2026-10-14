#define CHASSIS_DISTANCE_TEST_HELPERS_ONLY
#include "chassis_distance_test.c"
static void run_path(unsigned kind,float speed,float turn,unsigned pause_at,unsigned delay)
{
  ChassisPath path;
  float sx=kind==1?2250:kind==2?2100:1200,sy=kind==1?150:kind==2?2100:1200;
  float tx=kind==1?2100:1200,ty=kind==1?1250:2100;
  setup();
  float yaw=kind==1?180:90,x=sx,y=sy,vx=0,vy=0,w=0;
  assert(ChassisMotion_AnchorSet(yaw*CHASSIS_MODEL_PI/180));
  location.observer.feedback.x_mm=x;location.observer.feedback.y_mm=y;
  assert(ChassisPath_Build(&path,kind,sx,sy,tx,ty,speed));
  assert(ChassisMotion_PathStart(&path,speed,turn,200000,0));
  unsigned action=status().action_id;bool paused=false,resumed=false;float saved=0;
  float peak_w=0,min_moving=10000;unsigned moving_samples=0;
  float xs[16],ys[16];for(unsigned i=0;i<16;i++){xs[i]=x;ys[i]=y;}
  unsigned sequence=0;
  FILE *trace=fopen("../.embeddedskills/tests/curve-plant.csv","a");assert(trace);
  for(unsigned now=120;now<180000;now+=20){
    ChassisMotion_Status_t m=status();
    float rad=yaw*CHASSIS_MODEL_PI/180;
    float wantx=cosf(rad)*m.body_target.vx_mm_s-sinf(rad)*m.body_target.vy_mm_s;
    float wanty=sinf(rad)*m.body_target.vx_mm_s+cosf(rad)*m.body_target.vy_mm_s;
    vx+=(wantx-vx)*.2f;vy+=(wanty-vy)*.2f;
    w+=(m.body_target.omega_rad_s*180/CHASSIS_MODEL_PI-w)*.2f;
    x+=vx*.02f;y+=vy*.02f;yaw+=w*.02f;
    sample(now);reading(now,30+yaw-(kind==1?180:90));
    xs[sequence%16]=x;ys[sequence%16]=y;
    unsigned old=sequence>=delay?(sequence-delay)%16:0;
    location.observer.feedback.x_mm=xs[old];location.observer.feedback.y_mm=ys[old];sequence++;
    location.feedback_tick=now-delay*20;
    location.observer.feedback.yaw_rad=yaw*CHASSIS_MODEL_PI/180;
    /* Quantized encoder positions still move during brake settling. */
    for(unsigned i=0;i<4;i++){
      location.wheels[i].position_raw=(int64_t)((x+y+yaw)*65536/251.327f);
      location.wheels[i].speed_rpm=(int16_t)lroundf(hypotf(vx,vy)*60/251.327f);
    }
    if(m.state==CHASSIS_MOTION_STOPPING){
      if(bus.stop_pending || bus.stage==MECANUM_STAGE_STOPPED){
        if(bus.stop_pending)bus.sent_ms=now;
        bus.stage=MECANUM_STAGE_STOPPED;bus.stop_pending=false;bus.tx_complete=true;
      }else{
        bus.stage=MECANUM_STAGE_SENT;bus.tx_complete=true;bus.sent_velocity_valid=true;
        bus.sent_ms=now;memset(bus.sent_rpm,0,sizeof bus.sent_rpm);
      }
    }else bus.stage=MECANUM_STAGE_SENT;
    ChassisMotion_Process();
    fprintf(trace,"%u,%u,%.4f,%.4f,%.4f\n",kind,now,x,y,yaw);
    peak_w=fmaxf(peak_w,fabsf(status().body_target.omega_rad_s)*180/CHASSIS_MODEL_PI);
    if(now>1000 && ChassisMotion_PathProgress()<path.turn_end && !paused){
      min_moving=fminf(min_moving,hypotf(vx,vy));moving_samples++;
      assert(status().state==CHASSIS_MOTION_RUNNING && status().action_id==action && stops==0);
    }
    if(pause_at && !paused && ChassisMotion_PathProgress()>path.turn_end-pause_at){
      saved=ChassisMotion_PathProgress();assert(ChassisMotion_Stop(0));paused=true;
    }
    if(!ChassisMotion_IsBusy()){
      if(paused && !resumed){
        if(hypotf(vx,vy)>.5f || fabsf(w)>.1f)continue; /* route's independent stop monitor */
        assert(ChassisMotion_PathProgress()>=saved);
        if(!ChassisMotion_PathStart(&path,speed,turn,200000,saved)){
          printf("resume rejected: %s rpm=%d saved=%.2f\n",status().reason,location.wheels[0].speed_rpm,saved);fflush(stdout);assert(0);
        }resumed=true;continue;
      }
      break;
    }
  }
  fclose(trace);
  printf("curve kind=%u speed=%.0f turn=%.0f final=%.2f,%.2f,%.2f state=%d reason=%s min_v=%.2f\n",
    kind,speed,turn,x,y,yaw,status().state,status().reason,min_moving);
  fflush(stdout);
  assert(status().state==CHASSIS_MOTION_DONE && status().stop_confirmed);
  assert(hypotf(x-tx,y-ty)<12 && fabsf(yaw-(kind==1?90:0))<2.1f);
  assert(peak_w<=turn+.001f && moving_samples && min_moving>1);
  if(pause_at)assert(resumed);
}
int main(void)
{
  FILE *trace=fopen("../.embeddedskills/tests/curve-plant.csv","w");assert(trace);fclose(trace);
  (void)zero_dispatched;
  for(unsigned kind=1;kind<=3;kind++)run_path(kind,1000,100,0,0);
  run_path(1,200,20,0,0);run_path(2,200,50,50,0);
  for(unsigned kind=1;kind<=3;kind++)run_path(kind,1000,100,0,6);
  for(unsigned kind=1;kind<=3;kind++)run_path(kind,1000,100,0,11);
  /* Stop/resume on the new entry, mid-turn and near the straight exit. */
  run_path(1,1000,100,700,11);
  run_path(1,1000,100,350,11);
  run_path(1,1000,100,30,11);
  puts("chassis_curve_motion_test: PASS");
}
