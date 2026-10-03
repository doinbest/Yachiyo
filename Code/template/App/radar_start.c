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

void RadarStart_Begin(RadarStart_Context_t *context,const RadarMap_Params_t *lower)
{
  float zero;
  if(context==NULL)return;
  if(lower!=NULL)context->params[1]=*lower;
  else RadarMap_Defaults(&context->params[1]);
  context->params[0]=context->params[1];
  /* Rotate the lower-start sensor offset by 180deg about the vehicle center. */
  context->params[0].lidar_x_mm=4500.0f-context->params[1].lidar_x_mm;
  context->params[0].lidar_y_mm=2400.0f-context->params[1].lidar_y_mm;
  zero=fmodf(context->params[1].zero_deg+180.0f,360.0f);
  if(zero<0.0f)zero+=360.0f;
  context->params[0].zero_deg=zero;
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
