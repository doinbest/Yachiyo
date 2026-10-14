#include "station_heading.h"
#include "Camera.h"
#include "GrabTask.h"
#include "chassis_heading.h"
#include "chassis_motion.h"
#include "chassis_localization.h"
#include "chassis_config.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
static enum { H_REQUEST,H_SAMPLE,H_COMMIT,H_TURN,H_MOVING,H_DONE } state;
static unsigned station,count;
static uint32_t begun,window,sequence,last_rx;
static bool have_seq,have_rx,debt,calibrated,clock_started;
static bool observe_only;
static float nominal,speed,sum,low,high,first_yaw,image_angle,body_error,observation_z;
static float first_image;
static StationHeading_Config_t cfg;
static const char *reason="heading_idle";
void StationHeading_Reset(void){state=H_DONE;reason="heading_idle";debt=false;clock_started=false;image_angle=body_error=observation_z=NAN;}
static void fresh_window(void){count=0;have_seq=have_rx=false;window=HAL_GetTick();}
void StationHeading_Begin(unsigned at,float yaw,float max_speed)
{
  observe_only=false;
  station=at;nominal=yaw;speed=max_speed;debt=false;clock_started=false;image_angle=body_error=observation_z=NAN;
  begun=HAL_GetTick();state=H_REQUEST;fresh_window();reason="heading_observing";
}
void StationHeading_ObserveBegin(unsigned at,float yaw,float max_speed)
{StationHeading_Begin(at,yaw,max_speed);observe_only=true;}
void StationHeading_Pause(void)
{
  if(state==H_DONE)return;
  Camera_RequestStop();state=H_REQUEST;fresh_window();clock_started=false;
}
const char *StationHeading_Reason(void){return reason;}
float StationHeading_ImageAngle(void){return image_angle;}
float StationHeading_BodyError(void){return body_error;}
float StationHeading_ObservationZ(void){return observation_z;}
static bool finish(const char *why){Camera_RequestStop();reason=why;state=H_DONE;return true;}
static bool stationary(ChassisMotion_Status_t *motion)
{
  ChassisLocalization_Status_t p;
  ChassisMotion_StatusGet(motion);ChassisLocalization_Get(&p);
  /* StopRequest confirms physical rest independently of distance-task arrival.
   * Its getter revokes old proof after a new motion/action. */
  ChassisStop_Status_t stop;ChassisMotion_StopStatusGet(&stop);
  bool stopped=motion->stop_confirmed || (stop.tx_complete && stop.wheels_stopped);
  uint32_t now=HAL_GetTick();
  if(ChassisMotion_IsBusy() || !motion->heading_valid || !motion->map_anchor_valid ||
     !stopped || !p.origin_valid || !p.feedback_valid || !p.speed_valid ||
     now-p.feedback_tick>600U || !GrabTask_HeadingReady())return false;
  for(unsigned i=0;i<4;i++)if(!p.wheels[i].state_valid || now-p.wheels[i].state_ms>600U ||
     (p.wheels[i].state_flags&0x0dU)!=1U || abs(p.wheels[i].speed_rpm)>1)return false;
  return true;
}
bool StationHeading_Process(void)
{
  ChassisMotion_Status_t motion;
  uint32_t now=HAL_GetTick();
  if(state==H_DONE)return true;
  Mecanum_Feedback_Enable(true);
  if(state==H_MOVING){
    ChassisMotion_StatusGet(&motion);
    if(motion.state==CHASSIS_MOTION_ERROR){reason=motion.reason;return false;}
    if(motion.state!=CHASSIS_MOTION_DONE || !motion.stop_confirmed)return false;
    state=H_REQUEST;fresh_window();clock_started=false;
  }
  /* Missing frames/readiness and USB retries share one bounded opportunity.
   * Once a correction is applied, finish its motion and verification instead. */
  if(clock_started && !debt && now-begun>=cfg.wait_ms)
    return finish("heading_no_observation");
  if(!stationary(&motion)){
    if(state==H_SAMPLE || state==H_COMMIT){Camera_RequestStop();state=H_REQUEST;}
    count=0;window=now;reason="heading_pose_wait";return false;
  }
  if(state==H_REQUEST){
    /* Start once after readiness/resume; USB retries share this bounded window. */
    if(!clock_started){begun=now;clock_started=true;}
    GrabTask_HeadingConfigGet(station,&cfg);
    if(cfg.wait_ms>STATION_HEADING_WAIT_MS)cfg.wait_ms=STATION_HEADING_WAIT_MS;
    observation_z=cfg.observation_z_mm;
    calibrated=!observe_only && cfg.height_matches && isfinite(cfg.zero_deg) && isfinite(cfg.gain) && fabsf(cfg.gain)>0 &&
      isfinite(cfg.tolerance_deg) && cfg.tolerance_deg>0 && cfg.tolerance_deg<=2 &&
      isfinite(cfg.spread_deg) && cfg.spread_deg>0 && isfinite(cfg.settle_ms) && cfg.settle_ms>=0;
    if(Camera_PoseStart()!=HAL_OK){reason="heading_usb_wait";if(!debt && now-begun>=cfg.wait_ms)return finish("heading_no_observation");return false;}
    state=H_SAMPLE;fresh_window();reason="heading_observing";return false;
  }
  if(state==H_TURN){
    if(!isfinite(cfg.turn_limit_deg) || fabsf(body_error)>cfg.turn_limit_deg ||
       fabsf(remainderf(nominal-motion.map_yaw_rad*180/CHASSIS_MODEL_PI,360))>cfg.turn_limit_deg){reason="heading_turn_unverified";return false;}
    if(!ChassisMotion_TurnToPrecision(nominal,speed,cfg.tolerance_deg,CHASSIS_DISTANCE_TIMEOUT_MS)){
      ChassisMotion_StatusGet(&motion);reason=motion.reason;return false;
    }
    Camera_RequestStop();state=H_MOVING;reason="heading_turning";return false;
  }
  Camera_PoseSnapshot_t p;Camera_PoseSnapshotGet(&p);
  if(state==H_COMMIT){
    /* A busy transport/late stop may postpone commit: never retain a stale ready result. */
    if(!p.Active || !p.Valid || now-p.ReceivedTick>CAMERA_DATA_STALE_MS){state=H_SAMPLE;fresh_window();return false;}
    if(p.Sequence!=sequence || fabsf(remainderf(motion.map_yaw_rad-first_yaw,2*CHASSIS_MODEL_PI))*180/CHASSIS_MODEL_PI>cfg.tolerance_deg){state=H_SAMPLE;fresh_window();return false;}
    float observed=remainderf(nominal+body_error,360);
    /* Spread qualifies samples, not a permission to retain a wrong map when
     * a physical correction is needed. Check the actual remaining turn above. */
    if((fabsf(body_error)>cfg.tolerance_deg ||
        fabsf(remainderf(observed-motion.map_yaw_rad*180/CHASSIS_MODEL_PI,360))>fminf(cfg.spread_deg,cfg.tolerance_deg)) &&
       !ChassisHeading_Apply(observed*CHASSIS_MODEL_PI/180))return false;
    if(fabsf(body_error)<=cfg.tolerance_deg){debt=false;return finish("heading_verified");}
    debt=true;state=H_TURN;reason="heading_corrected";return false;
  }
  if(!debt && now-begun>=cfg.wait_ms)return finish("heading_no_observation");
  if(!p.Active || !p.HasFrame || !p.Valid || now-p.ReceivedTick>CAMERA_DATA_STALE_MS){
    /* Retain samples that were fresh when received. Missing/invalid reports
     * add no evidence; only new valid samples can complete the group. */
    if(!debt && now-begun>=cfg.wait_ms)return finish("heading_no_observation");
    reason=debt?"heading_reacquire":"heading_observing";return false;
  }
  if((have_seq && (int32_t)(p.Sequence-sequence)<=0) ||
     (have_rx && (int32_t)(p.ReceivedTick-last_rx)<=0))return false;
  sequence=p.Sequence;have_seq=true;last_rx=p.ReceivedTick;have_rx=true;
  if((uint32_t)(p.ReceivedTick-window)>=0x80000000U ||
     (calibrated && p.ReceivedTick-window<(uint32_t)cfg.settle_ms))return false;
  image_angle=p.AngleCdeg*.01f; /* STOP deliberately never consulted. */
  if(!calibrated && debt){reason="heading_calibration_changed";return false;}
  if(!calibrated){
    /* Image-only observations have no calibrated gain. Compare in image
     * degrees, unwrapped around the first sample at the +/-90 boundary. */
    float e=count?first_image+remainderf(image_angle-first_image,180):image_angle;
    if(!count || fabsf(remainderf(motion.map_yaw_rad-first_yaw,2*CHASSIS_MODEL_PI))*180/CHASSIS_MODEL_PI>STATION_HEADING_IMAGE_SPREAD_DEG ||
       fmaxf(high,e)-fminf(low,e)>STATION_HEADING_IMAGE_SPREAD_DEG){
      count=0;sum=0;first_image=e=image_angle;low=high=e;first_yaw=motion.map_yaw_rad;
    }
    low=fminf(low,e);high=fmaxf(high,e);sum+=e;
    if(++count>=3){
      image_angle=remainderf(sum/count,180);
      return finish(observe_only?"heading_observed":cfg.height_matches?"heading_uncalibrated":"heading_height_unmatched");
    }
    return false;
  }
  float e=remainderf(image_angle-cfg.zero_deg,180)*cfg.gain;
  if(!count || fabsf(remainderf(motion.map_yaw_rad-first_yaw,2*CHASSIS_MODEL_PI))*180/CHASSIS_MODEL_PI>cfg.tolerance_deg ||
     fmaxf(high,e)-fminf(low,e)>cfg.spread_deg){count=0;sum=0;low=high=e;first_yaw=motion.map_yaw_rad;}
  low=fminf(low,e);high=fmaxf(high,e);sum+=e;
  if(++count<3)return false;
  body_error=sum/count;
  /* Reject a gross branch mismatch, without revoking a previously valid map. */
  if(fabsf(body_error)>=90){count=0;reason="heading_angle_mismatch";return false;}
  state=H_COMMIT;return StationHeading_Process();
}
