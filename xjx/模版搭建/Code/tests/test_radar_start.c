#include "radar_start.h"
#include <math.h>
#include <stdio.h>
#include <string.h>


static void legacy_defaults(RadarMap_Params_t *p) { RadarMap_Defaults(p); p->situation=0; p->grid_version=3; p->lidar_x_mm=2250; p->lidar_y_mm=150; p->zero_deg=180; }
static unsigned checks,failures;
#define CHECK(c) do { ++checks; if(!(c)){++failures; \
  printf("FAIL line %d: %s\n",__LINE__,#c);} } while(0)
static int near(float a,float b){return fabsf(a-b)<0.03f;}

static void test_shared_corner_params(void)
{
  RadarMap_Params_t lower,out,saved;
  legacy_defaults(&lower);lower.lidar_x_mm=2180;lower.lidar_y_mm=220;lower.zero_deg=350;
  lower.threshold=7;lower.angle_max_tenths=700;saved=lower;
  CHECK(RadarStart_ParamsForStart(&lower,RADAR_START_UPPER,&out));
  CHECK(near(out.lidar_x_mm,2320) && near(out.lidar_y_mm,2180) && near(out.zero_deg,80));
  CHECK(out.threshold==7 && out.angle_max_tenths==700 && !memcmp(&lower,&saved,sizeof lower));
  CHECK(RadarStart_ParamsForStart(&lower,RADAR_START_LOWER,&out));
  CHECK(!memcmp(&out,&lower,sizeof out));
  CHECK(RadarStart_ParamsForStart(&out,RADAR_START_UPPER,&out));
  CHECK(near(out.lidar_x_mm,2320) && near(out.lidar_y_mm,2180) && near(out.zero_deg,80));
  CHECK(!RadarStart_ParamsForStart(&lower,RADAR_START_UNKNOWN,&out));
  CHECK(!RadarStart_ParamsForStart(NULL,RADAR_START_LOWER,&out));
  CHECK(!RadarStart_ParamsForStart(&lower,RADAR_START_LOWER,NULL));
}

/* Independent synthetic LDS50C samples: physical 10mm range resolution, 0.1deg. */
static RadarSample_t polar(float lx,float ly,float zero,float x,float y,uint8_t energy)
{
  RadarSample_t sample;
  float dx=x-lx,dy=y-ly;
  float local=fmodf(zero-atan2f(dy,dx)*57.29577951308232f+720.0f,360.0f);
  sample.angle_tenths=(uint16_t)lroundf(local*10.0f);
  sample.distance_mm=(uint16_t)(10.0f*roundf(hypotf(dx,dy)/10.0f));
  sample.energy=energy;return sample;
}

/* Synthetic returns in a non-mandatory cell under the physical corner.
 * The wrong corner projects lower cell2 to QR14, or upper cell10 to RAW22. */
static void board(RadarStart_Context_t *context,RadarStart_Id_t physical_start,unsigned count)
{
  float lx=2250.0f;
  float ly=physical_start==RADAR_START_LOWER?150.0f:2250.0f;
  float zero=physical_start==RADAR_START_LOWER?180.0f:270.0f;
  for(unsigned i=0;i<count;i++) {
    float x=physical_start==RADAR_START_LOWER?1200.0f:350.0f+20.0f*(float)(i%3U);
    float y=physical_start==RADAR_START_LOWER?350.0f+20.0f*(float)(i%3U):1200.0f;
    RadarSample_t sample=polar(lx,ly,zero,x,y,80);
    CHECK(sample.angle_tenths<=900);
    RadarStart_Add(context,&sample);
  }
}

/* Static corner headings differ by 90deg; vehicle yaw stays independently fixed. */
static void test_reference_and_poses(void)
{
  RadarMap_Params_t reference;
  RadarStart_Context_t context;
  const RadarMap_Params_t *upper,*lower;
  const RadarStart_Pose_t *pose;
  legacy_defaults(&reference);RadarStart_Begin(&context,&reference);
  upper=RadarStart_Params(&context,RADAR_START_UPPER);
  lower=RadarStart_Params(&context,RADAR_START_LOWER);
  CHECK(upper!=NULL && lower!=NULL);
  if(upper && lower) {
    CHECK(near(upper->lidar_x_mm,2250) && near(upper->lidar_y_mm,2250) && near(upper->zero_deg,270));
    CHECK(near(lower->lidar_x_mm,2250) && near(lower->lidar_y_mm,150) && near(lower->zero_deg,180));
    CHECK(upper->threshold==1 && lower->threshold==1 && lower->angle_max_tenths==3600);
  }
  CHECK(reference.lidar_x_mm==2250 && reference.lidar_y_mm==150 && reference.zero_deg==180);
  reference.lidar_x_mm=2100;reference.lidar_y_mm=300;reference.zero_deg=-90;
  reference.energy_min=17;reference.energy_max=100;reference.threshold=4;
  reference.angle_min_tenths=3500;reference.angle_max_tenths=100;
  reference.distance_min_mm=300;reference.distance_max_mm=12000;
  RadarStart_Begin(&context,&reference);upper=RadarStart_Params(&context,RADAR_START_UPPER);
  if(upper) {
    CHECK(near(upper->lidar_x_mm,2400) && near(upper->lidar_y_mm,2100) && near(upper->zero_deg,0));
    CHECK(upper->energy_min==17 && upper->energy_max==100 && upper->threshold==4);
    CHECK(upper->distance_min_mm==300 && upper->distance_max_mm==12000);
    CHECK(upper->angle_min_tenths==3500 && upper->angle_max_tenths==100);
  }
  pose=RadarStart_Pose(RADAR_START_UPPER);
  CHECK(pose && near(pose->x_mm,2250) && near(pose->y_mm,2250) && near(pose->yaw_deg,270));
  pose=RadarStart_Pose(RADAR_START_LOWER);
  CHECK(pose && near(pose->x_mm,2250) && near(pose->y_mm,150) && near(pose->yaw_deg,90));
  CHECK(RadarStart_Pose(RADAR_START_UNKNOWN)==NULL);
  CHECK(RadarStart_Map(&context,RADAR_START_UNKNOWN)==NULL);
  CHECK(RadarStart_Params(&context,RADAR_START_UNKNOWN)==NULL);
}

/* Catch a forced default on no evidence, symmetric evidence or contradictory scans. */
static void test_ambiguity_and_conflicts(void)
{
  RadarMap_Params_t params;RadarStart_Context_t context;RadarStart_Result_t result;
  legacy_defaults(&params);RadarStart_Begin(&context,&params);
  RadarStart_Finish(&context,&result);
  CHECK(result.start_id==RADAR_START_UNKNOWN && !strcmp(result.reason,"ambiguous"));
  CHECK(result.conflict_mask[0]==0 && result.conflict_mask[1]==0);
  /* Inward ordinary evidence need not identify a corner; never force a default. */
  for(unsigned i=0;i<2;i++) {
    RadarSample_t sample=polar(2250,150,180,1200,1200,80);
    RadarStart_Add(&context,&sample);
  }
  RadarStart_Finish(&context,&result);
  CHECK(result.start_id==RADAR_START_UNKNOWN && !strcmp(result.reason,"ambiguous"));
  CHECK(context.maps[0].counts[12]==2 && context.maps[1].counts[12]==2);
  RadarStart_Begin(&context,&params);
  board(&context,RADAR_START_LOWER,1);RadarStart_Finish(&context,&result);
  CHECK(result.start_id==RADAR_START_LOWER && result.qr_counts[0]==1 &&
    !strcmp(result.reason,"clear_regions"));
  board(&context,RADAR_START_UPPER,2);board(&context,RADAR_START_LOWER,1);
  RadarStart_Finish(&context,&result);
  CHECK(result.start_id==RADAR_START_UNKNOWN && !strcmp(result.reason,"both_conflict"));
  CHECK(result.conflict_mask[0]==(1UL<<14) && result.conflict_mask[1]==(1UL<<22));
  CHECK(result.qr_counts[0]==2 && result.raw_counts[1]==2);
}

/* Both physical corner fixtures must select their own world frame, not the sensor. */
static void test_board_classification_both_starts(void)
{
  RadarMap_Params_t params;RadarStart_Context_t context;RadarStart_Result_t result;
  legacy_defaults(&params);
  for(unsigned id=RADAR_START_UPPER;id<=RADAR_START_LOWER;id++) {
    RadarStart_Id_t start=(RadarStart_Id_t)id;
    RadarStart_Begin(&context,&params);board(&context,start,3);
    /* A small obstacle wholly inside cell17 must survive map protection. */
    for(unsigned i=0;i<3;i++) {
      RadarSample_t sample=start==RADAR_START_LOWER?
        polar(2250,150,180,1200,1590.0f+10.0f*(float)i,80):
        polar(2250,2250,270,1200,1590.0f+10.0f*(float)i,80);
      RadarStart_Add(&context,&sample);
    }
    RadarStart_Finish(&context,&result);
    CHECK(result.start_id==id && !strcmp(result.reason,"clear_regions"));
    CHECK(result.conflict_mask[id-1U]==0);
    CHECK(result.conflict_mask[2U-id]==(1UL<<(id==RADAR_START_LOWER?14:22)));
    CHECK((id==RADAR_START_LOWER?result.qr_counts[0]:result.raw_counts[1])==3);
    const RadarMap_t *map=RadarStart_Map(&context,start);
    CHECK(map && map->mask==(RADAR_MAP_FIXED_MASK|(1UL<<17)) && map->counts[17]==3 &&
          map->counts[id==RADAR_START_LOWER?2:10]==3);
    /* Classification evidence can occupy a business cell; plan an independent clear map. */
    RadarMap_t clear;RadarPlan_t plan;RadarMap_Reset(&clear);
    CHECK(RadarPlan_BuildFromStart(&clear,start,&plan));
    CHECK(plan.valid && plan.start_id==id && plan.points[0].x_mm==2250 &&
      plan.points[0].y_mm==(id==RADAR_START_UPPER?2250:150));
    CHECK(plan.points[plan.count-1U].x_mm==2250 &&
      plan.points[plan.count-1U].y_mm==(id==RADAR_START_UPPER?2250:150));
  }
}

/* Raw-front evidence also participates; finishing must not erase it before deciding. */
static void test_raw_conflict_and_count_retention(void)
{
  RadarMap_Params_t params;RadarStart_Context_t context;RadarStart_Result_t result;
  legacy_defaults(&params);RadarStart_Begin(&context,&params);
  for(unsigned i=0;i<3;i++) {
    RadarSample_t sample=polar(2250,150,180,1200,2100,80);
    RadarStart_Add(&context,&sample);
  }
  RadarStart_Finish(&context,&result);
  CHECK(result.start_id==RADAR_START_UPPER && result.conflict_mask[1]==(1UL<<22));
  CHECK(result.raw_counts[1]==3 && context.maps[1].counts[22]==3);
  CHECK(context.maps[1].mask&(1UL<<22));
  RadarStart_Finish(&context,&result);
  CHECK(result.start_id==RADAR_START_UPPER && result.raw_counts[1]==3);
}

/* Catch ignored filters and cumulative display-buffer limits in the streaming core. */
static void test_filters_streaming_and_saturation(void)
{
  RadarMap_Params_t params;RadarStart_Context_t context;RadarStart_Result_t result;
  RadarSample_t sample=polar(2250,150,180,1200,350,80);
  legacy_defaults(&params);params.energy_min=81;RadarStart_Begin(&context,&params);
  board(&context,RADAR_START_LOWER,3);RadarStart_Finish(&context,&result);
  CHECK(result.start_id==RADAR_START_UNKNOWN && result.qr_counts[0]==0);
  legacy_defaults(&params);params.distance_max_mm=900;RadarStart_Begin(&context,&params);
  board(&context,RADAR_START_LOWER,3);RadarStart_Finish(&context,&result);
  CHECK(result.start_id==RADAR_START_UNKNOWN && result.qr_counts[0]==0);
  legacy_defaults(&params);params.angle_max_tenths=100;RadarStart_Begin(&context,&params);
  board(&context,RADAR_START_LOWER,3);RadarStart_Finish(&context,&result);
  CHECK(result.start_id==RADAR_START_UNKNOWN && result.qr_counts[0]==0);
  legacy_defaults(&params);RadarStart_Begin(&context,&params);
  for(unsigned i=0;i<5000;i++)RadarStart_Add(&context,&sample);
  RadarStart_Finish(&context,&result);
  CHECK(result.start_id==RADAR_START_LOWER && result.qr_counts[0]==5000);
  for(unsigned i=0;i<65000;i++)RadarStart_Add(&context,&sample);
  RadarStart_Finish(&context,&result);
  CHECK(result.start_id==RADAR_START_LOWER && result.qr_counts[0]==65535);
  RadarStart_Begin(&context,&params);RadarStart_Finish(&context,&result);
  CHECK(result.start_id==RADAR_START_UNKNOWN && result.qr_counts[0]==0 && result.qr_counts[1]==0);
}

int main(void)
{
  test_shared_corner_params();test_reference_and_poses();test_ambiguity_and_conflicts();
  test_board_classification_both_starts();test_raw_conflict_and_count_retention();
  test_filters_streaming_and_saturation();
  printf("Radar dual start: %u checks, %u failures\n",checks,failures);
  return failures?1:0;
}
