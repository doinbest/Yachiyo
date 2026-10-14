/** @file radar_start.c
 * @brief 两个名义起点下的流式计数与前方禁障格分类；不改车辆参考或运动。
 */
#include "radar_start.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static const RadarStart_Pose_t StartPoses[2]={{2250.0f,2250.0f,270.0f},{2250.0f,150.0f,90.0f}};

static int start_index(RadarStart_Id_t id)
{
  return id==RADAR_START_UPPER?0:(id==RADAR_START_LOWER?1:-1);
}

bool RadarStart_ParamsForStart(const RadarMap_Params_t *lower,RadarStart_Id_t id,
                               RadarMap_Params_t *out)
{
  if(lower==NULL || out==NULL || start_index(id)<0)return false;
  *out=*lower;
  out->start_id=(uint8_t)id;
  if(id==RADAR_START_UPPER) {
    if(lower->grid_version>=4U){
      /* Upper installation is an independent physical calibration. */
      out->lidar_x_mm=lower->upper_lidar_x_mm;out->lidar_y_mm=lower->upper_lidar_y_mm;out->zero_deg=270.0f;
      RadarGeometry_Default(&out->geometry,1U);out->geometry.zero_tenths=2700U;
      out->geometry.origin_q16[0]=(int32_t)roundf(out->lidar_x_mm*65536.0f);
      out->geometry.origin_q16[1]=(int32_t)roundf(out->lidar_y_mm*65536.0f);
      out->situation=1U;return true;
    }
    /* Static corner contract; this is independent of the vehicle's yaw change. */
    out->lidar_x_mm=4500.0f-out->lidar_x_mm;
    out->lidar_y_mm=2400.0f-out->lidar_y_mm;
    out->zero_deg=fmodf(out->zero_deg+90.0f,360.0f);
    if(out->zero_deg<0.0f)out->zero_deg+=360.0f;
  }
  return true;
}

void RadarStart_Begin(RadarStart_Context_t *context,const RadarMap_Params_t *lower)
{
  RadarMap_Params_t reference;
  if(context==NULL)return;
  if(lower!=NULL)reference=*lower;
  else RadarMap_Defaults(&reference);
  RadarStart_ParamsForStart(&reference,RADAR_START_LOWER,&context->params[1]);
  RadarStart_ParamsForStart(&reference,RADAR_START_UPPER,&context->params[0]);
  RadarMap_Reset(&context->maps[0]);RadarMap_Reset(&context->maps[1]);
}

void RadarStart_Add(RadarStart_Context_t *context,const RadarSample_t *sample)
{
  if(context==NULL)return;
  RadarMap_Add(&context->maps[0],&context->params[0],sample);
  RadarMap_Add(&context->maps[1],&context->params[1],sample);
}

void RadarStart_Finish(RadarStart_Context_t *context,RadarStart_Result_t *result)
{
  unsigned i;
  if(result==NULL)return;
  memset(result,0,sizeof(*result));result->reason="ambiguous";
  if(context==NULL)return;
  for(i=0U;i<2U;i++) {
    RadarMap_FinishRaw(&context->maps[i],&context->params[i]);
    result->conflict_mask[i]=context->maps[i].mask&RADAR_MAP_REQUIRED_CLEAR_MASK;
    result->qr_counts[i]=context->maps[i].counts[14];
    result->raw_counts[i]=context->maps[i].counts[22];
  }
  if(result->conflict_mask[0]!=0U && result->conflict_mask[1]!=0U) {
    result->reason="both_conflict";
  } else if((result->conflict_mask[0]==0U)!=(result->conflict_mask[1]==0U)) {
    i=result->conflict_mask[0]==0U?0U:1U;
    result->start_id=(uint8_t)(i==0U?RADAR_START_UPPER:RADAR_START_LOWER);
    result->reason="clear_regions";
    RadarMap_Finish(&context->maps[i],&context->params[i]);
  }
}

const RadarMap_t *RadarStart_Map(const RadarStart_Context_t *context,RadarStart_Id_t id)
{
  int index=start_index(id);
  return context!=NULL && index>=0?&context->maps[index]:NULL;
}

const RadarMap_Params_t *RadarStart_Params(const RadarStart_Context_t *context,RadarStart_Id_t id)
{
  int index=start_index(id);
  return context!=NULL && index>=0?&context->params[index]:NULL;
}

const RadarStart_Pose_t *RadarStart_Pose(RadarStart_Id_t id)
{
  int index=start_index(id);
  return index>=0?&StartPoses[index]:NULL;
}
