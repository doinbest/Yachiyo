/**
 * @file radar_plan_cli.c
 * @brief Same C map/planner as the MCU, with a streaming host-side text adapter.
 *
 * stdin (whitespace separated, decimal integers):
 * RADAR1 manual_mask sample_count
 * lidar_x_mm lidar_y_mm zero_deg distance_min_mm distance_max_mm
 * angle_min_tenths angle_max_tenths threshold energy_min energy_max
 * Then sample_count triples: angle_tenths distance_mm energy.
 * N=0 plans a manual mask; N>0 classifies the full cloud, then ORs manual_mask.
 * stdout: one JSON object with algorithm, valid, failed_leg, mask, counts,
 * points[[x_mm,y_mm,station,visit],...] and route_profiles from the shared route
 * phase core. Optional --speed mm/s defaults to the firmware route speed.
 * --start manual|auto|upper|lower defaults to the existing manual reference.
 * Upper-start output is a geometric plan; driving profiles remain lower-start.
 * No-path is a normal result (exit 0).
 * Malformed input/invalid params give {"error":"..."} and exit 2.
 *
 * Build from repository root:
 * gcc -std=c99 -O2 -Wall -Wextra -Werror -ICode/template/App
 *   Code/tools/radar_plan_cli.c Code/template/App/radar_map.c
 *   Code/template/App/radar_start.c Code/template/App/chassis_route_profile.c
 *   -lm -o radar_plan_cli
 */
#include "radar_map.h"
#include "radar_start.h"
#include "chassis_route_profile.h"
#include "chassis_path.h"
#include "chassis_config.h"

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Host-only storage, matching the MCU's existing full-cloud capacity. */
static RadarSample_t samples[4096];
static union { RadarStableContext rolling; RadarStableBatchContext automatic[2]; } stable;
static const RadarStablePoint *stable_points;
static uint16_t stable_count;
static RadarStableStatus stable_status;
static bool stable_input;

static int read_word(char word[96])
{
  return scanf("%95s",word)==1;
}

static int read_uint(uint32_t max,uint32_t *value)
{
  char word[96],*end;
  unsigned long number;
  if(!read_word(word) || word[0]=='-') return 0;
  errno=0;number=strtoul(word,&end,10);
  if(errno || end==word || *end || number>max) return 0;
  *value=(uint32_t)number;return 1;
}

static int read_float(float *value)
{
  char word[96],*end;
  if(!read_word(word)) return 0;
  errno=0;*value=strtof(word,&end);
  return !errno && end!=word && !*end;
}
static int read_int32(int32_t *value)
{
  char word[96],*end;long long number;
  if(!read_word(word))return 0;
  errno=0;number=strtoll(word,&end,10);
  if(errno || end==word || *end || number<INT32_MIN || number>INT32_MAX)return 0;
  *value=(int32_t)number;return 1;
}

static int input_error(const char *error)
{
  printf("{\"error\":\"%s\"}\n",error);return 2;
}

static int read_params(RadarMap_Params_t *p)
{
  uint32_t dmin,dmax,amin,amax,threshold,emin,emax;
  if(!read_float(&p->lidar_x_mm) || !read_float(&p->lidar_y_mm) || !read_float(&p->zero_deg) ||
     !read_uint(UINT16_MAX,&dmin) || !read_uint(UINT16_MAX,&dmax) ||
     !read_uint(UINT16_MAX,&amin) || !read_uint(UINT16_MAX,&amax) ||
     !read_uint(UINT16_MAX,&threshold) || !read_uint(UINT8_MAX,&emin) || !read_uint(UINT8_MAX,&emax)) return 0;
  p->distance_min_mm=(uint16_t)dmin;p->distance_max_mm=(uint16_t)dmax;
  p->angle_min_tenths=(uint16_t)amin;p->angle_max_tenths=(uint16_t)amax;
  p->threshold=(uint16_t)threshold;p->energy_min=(uint8_t)emin;p->energy_max=(uint8_t)emax;
  return 1;
}

static void print_points(const RadarPlan_Point_t *points,unsigned count)
{
  unsigned i;
  putchar('[');
  for(i=0U;i<count;i++) {
    const RadarPlan_Point_t *point=&points[i];
    printf("%s[%d,%d,%u,%u]",i?",":"",(int)point->x_mm,(int)point->y_mm,
           (unsigned)point->station,(unsigned)point->visit);
  }
  putchar(']');
}

static void print_profile(const RadarPlan_Point_t *points,unsigned count,float speed,bool task,bool continuous_entry,bool smooth_entry)
{
  unsigned segment,index,emitted=0U;
  float yaw=180.0f;
  bool first_scan_leg=true;
  ChassisRoute_Profile_t profile;
  printf("{\"points\":");print_points(points,count);printf(",\"initial_yaw_deg\":180,\"phases\":[");
  for(segment=1U;segment<count;segment++) {
    const RadarPlan_Point_t *from=&points[segment-1U],*to=&points[segment];
    /* Match the live WorkResume: discard only the obsolete same-column
     * pre-exit neutral connectors, then show the mission-owned retreat425. */
    if(task && from->station==2U)
      while(!to->station && to->x_mm==from->x_mm &&
            to->y_mm<=CHASSIS_PROCESSING_EXIT_Y_MM && segment+1U<count)
        to=&points[++segment];
    if(!(task?ChassisRouteProfile_BuildTaskEntry(from,to,yaw,speed,first_scan_leg,continuous_entry || smooth_entry,&profile):
        ChassisRouteProfile_Build(from,to,yaw,speed,first_scan_leg,&profile))) break;
    float accels[CHASSIS_ROUTE_PROFILE_MAX_PHASES];
    ChassisRouteProfile_ApplyTaskLinear(from,to,&profile,task && !continuous_entry,accels);
    for(index=0U;index<profile.count;index++) {
      const ChassisRoute_Phase_t *phase=&profile.phases[index];
      printf("%s{\"kind\":\"%s\",\"phase\":\"%s\",\"direction\":\"%s\",\"segment\":%u,"
             "\"station\":%u,\"visit\":%u,\"target\":[%.6g,%.6g,%.6g],\"speed_mm_s\":%.6g",
             emitted++?",":"",!strcmp(phase->direction,"curve")?"curve":phase->turn?"turn":"move",phase->phase,phase->direction,segment,
             (unsigned)to->station,(unsigned)to->visit,(double)phase->x_mm,(double)phase->y_mm,
             (double)phase->yaw_deg,(double)phase->speed_mm_s);
      if(!strcmp(phase->direction,"curve")){
        ChassisPath path={0};
        unsigned kind=!strcmp(phase->phase,"entry_curve")?1:from->x_mm==2100?2:3;
        ChassisPath_Build(&path,kind,from->x_mm,from->y_mm,to->x_mm,to->y_mm,phase->speed_mm_s);
        printf(",\"path\":[");
        for(unsigned j=0;j<path.count;j++){
          ChassisPath_Point p=path.point[j];
          printf("%s[%.6f,%.6f,%.6f,%.6f,%.6f]",j?",":"",(double)p.x,(double)p.y,(double)p.yaw,(double)p.s,(double)p.limit);
        }
        printf("]");
      }
      if(task) printf(",\"arm_pose\":\"%s\"",
        !strncmp(phase->phase,"work_",5) && strcmp(phase->phase,"work_approach") && strcmp(phase->phase,"work_curve")?"work":"rear");
      putchar('}');
    }
    yaw=profile.final_yaw_deg;
    if(to->station==5U && to->visit==1U) first_scan_leg=false;
  }
  printf("],\"speed_mm_s\":%.6g,\"turn_speed_deg_s\":%.6g}",(double)speed,(double)CHASSIS_MOTION_MAX_OMEGA_DEG_S);
}

static void print_result(const RadarMap_t *map,const RadarPlan_t *plan,float speed,
                         bool automatic,const RadarStart_Result_t *start,
                         const RadarMap_Params_t *params)
{
  unsigned i;
  RadarPlan_Point_t fixed[CHASSIS_ROUTE_TARGET_COUNT+1U];
  printf("{\"algorithm\":\"%s\",\"grid_version\":%u,\"valid\":%s,\"failed_leg\":%u,\"mask\":%" PRIu32 ",\"counts\":[",
         stable_input?"radar_stable_c_v2":"radar_map_c_v1",(unsigned)map->grid_version,
         plan->valid?"true":"false",(unsigned)plan->failed_leg,map->mask);
  for(i=0U;i<RADAR_MAP_CELL_COUNT;i++) printf("%s%u",i?",":"",(unsigned)map->counts[i]);
  printf("],\"accepted_points\":%u,\"outside_points\":%u,\"evidence_mask\":%" PRIu32 ",\"points\":",
         (unsigned)map->accepted_points,(unsigned)map->outside_points,map->evidence_mask);
  print_points(plan->points,plan->count);
  printf(",\"start\":{\"auto\":%s,\"selected\":%u,\"reason\":\"%s\","
         "\"qr\":[%u,%u],\"raw\":[%u,%u],\"conflicts\":[%" PRIu32 ",%" PRIu32 "]}",
         automatic?"true":"false",(unsigned)start->start_id,start->reason,
         (unsigned)start->qr_counts[0],(unsigned)start->qr_counts[1],
         (unsigned)start->raw_counts[0],(unsigned)start->raw_counts[1],
         start->conflict_mask[0],start->conflict_mask[1]);
  printf(",\"plan_only\":%s,\"pose\":",start->start_id==RADAR_START_UPPER?"true":"false");
  if(params) printf("[%.6g,%.6g,%.6g]",(double)params->lidar_x_mm,
                    (double)params->lidar_y_mm,(double)params->zero_deg);
  else printf("null");
  if(stable_input){
    printf(",\"window\":{\"first\":%" PRIu32 ",\"last\":%" PRIu32 ",\"circles\":%u},"
      "\"raw_points\":%" PRIu32 ",\"filtered_points\":%" PRIu32 ",\"stable_points\":[",
      stable_status.first_circle,stable_status.last_circle,(unsigned)stable_status.window_circles,
      stable_status.raw_points,stable_status.filtered_points);
    for(i=0;i<stable_count;i++)printf("%s[%u,%u,%u,%u,%u]",i?",":"",stable_points[i].x_mm,
      stable_points[i].y_mm,stable_points[i].votes,stable_points[i].angle_tenths,stable_points[i].distance_mm);
    printf("],\"geometry\":{\"matrix_q16\":[%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 "],"
      "\"origin_q16\":[%" PRId32 ",%" PRId32 "],\"zero_tenths\":%u}",
      params->geometry.matrix_q16[0],params->geometry.matrix_q16[1],params->geometry.matrix_q16[2],
      params->geometry.matrix_q16[3],params->geometry.origin_q16[0],params->geometry.origin_q16[1],
      params->geometry.zero_tenths);
  }
  for(i=0U;i<=CHASSIS_ROUTE_TARGET_COUNT;i++) (void)ChassisRouteProfile_FixedPoint(i,&fixed[i]);
  printf(",\"route_profiles\":{\"fixed\":");
  print_profile(fixed,CHASSIS_ROUTE_TARGET_COUNT+1U,speed,false,false,false);
  printf(",\"planned\":");print_profile(plan->points,
      plan->valid && start->start_id==RADAR_START_LOWER?plan->count:0U,speed,false,false,false);
  printf(",\"task_fixed\":");print_profile(fixed,CHASSIS_ROUTE_TARGET_COUNT+1U,speed,true,false,false);
  printf(",\"task_planned\":");print_profile(plan->points,
      plan->valid && start->start_id==RADAR_START_LOWER?plan->count:0U,speed,true,false,false);
  printf(",\"task_fixed_cosine\":");print_profile(fixed,CHASSIS_ROUTE_TARGET_COUNT+1U,speed,true,true,false);
  printf(",\"task_planned_cosine\":");print_profile(plan->points,
      plan->valid && start->start_id==RADAR_START_LOWER?plan->count:0U,speed,true,true,false);
  printf(",\"task_fixed_smooth\":");print_profile(fixed,CHASSIS_ROUTE_TARGET_COUNT+1U,speed,true,false,true);
  printf(",\"task_planned_smooth\":");print_profile(plan->points,
      plan->valid && start->start_id==RADAR_START_LOWER?plan->count:0U,speed,true,false,true);
  puts("}}");
}

int main(int argc,char *argv[])
{
  char word[96];
  float speed=CHASSIS_ROUTE_DEFAULT_SPEED_MM_S;
  uint32_t manual_mask,sample_count,i,angle,distance,energy,circle_count=0U,situation;
  RadarMap_Params_t params;
  RadarMap_t map;
  RadarPlan_t plan;
  RadarStart_Context_t candidates;
  RadarStart_Result_t start;
  const RadarMap_Params_t *selected_params=&params;
  const char *mode="manual";
  int arg;
  bool automatic=false;
  memset(&start,0,sizeof(start));memset(&plan,0,sizeof(plan));
  for(arg=1;arg<argc;arg+=2) {
    char *end;
    if(arg+1>=argc) return input_error("option_value");
    if(!strcmp(argv[arg],"--speed")) {
      errno=0;speed=strtof(argv[arg+1],&end);
      if(errno || end==argv[arg+1] || *end || !isfinite(speed) || speed<10.0f || speed>5000.0f)
        return input_error("speed_range");
    } else if(!strcmp(argv[arg],"--start")) {
      mode=argv[arg+1];
      if(strcmp(mode,"manual") && strcmp(mode,"auto") && strcmp(mode,"upper") && strcmp(mode,"lower"))
        return input_error("start_option");
    } else return input_error("unknown_option");
  }
  RadarMap_Defaults(&params);
  if(!read_word(word) || (strcmp(word,"RADAR1") && strcmp(word,"RADAR2")))return input_error("input_format");
  stable_input=!strcmp(word,"RADAR2");
  if(!stable_input){params.situation=0U;params.grid_version=3U;}
  if(
     !read_uint(RADAR_MAP_ALL_CELLS,&manual_mask) || !read_uint(4096U,&sample_count) ||
     !read_params(&params)) return input_error("input_format");
  if(stable_input){
    circle_count=sample_count;if(circle_count<15U || circle_count>256U)return input_error("circle_count");
    if(!read_uint(2U,&situation) || !situation)return input_error("geometry");
    params.situation=(uint8_t)situation;
    for(i=0U;i<4U;i++)if(!read_int32(&params.geometry.matrix_q16[i]))return input_error("geometry");
    for(i=0U;i<2U;i++)if(!read_int32(&params.geometry.origin_q16[i]))return input_error("geometry");
    if(!read_uint(3599U,&situation))return input_error("geometry");
    params.geometry.zero_tenths=(uint16_t)situation;
  }
  if(!RadarMap_ParamsValid(&params)) return input_error("invalid_params");
  if(stable_input)RadarMap_FreezeParams(&params,&params);
  RadarMap_Reset(&map);
  automatic=strcmp(mode,"manual")!=0;
  if(automatic) RadarStart_Begin(&candidates,&params);
  if(stable_input){
    if(automatic && !strcmp(mode,"auto")){
      if(circle_count!=15U)return input_error("auto_requires_15");
      for(i=0U;i<2U;i++)RadarStableBatch_Init(&stable.automatic[i]);
    }else{
      if(automatic){
        start.start_id=!strcmp(mode,"upper")?RADAR_START_UPPER:RADAR_START_LOWER;
        params=*RadarStart_Params(&candidates,(RadarStart_Id_t)start.start_id);
      }
      RadarStable_Init(&stable.rolling,circle_count>15U?1U:0U);
    }
    for(uint32_t circle=1U;circle<=circle_count;circle++){
      if(!read_uint(65535U,&sample_count))return input_error("input_format");
      bool dual=automatic && !strcmp(mode,"auto");
      if(dual)for(i=0U;i<2U;i++)(void)RadarStableBatch_BeginCircle(&stable.automatic[i],circle);
      else (void)RadarStable_BeginCircle(&stable.rolling,circle);
      for(uint32_t point=0U;point<sample_count;point++){
        RadarSample_t sample;
        if(!read_uint(3599U,&angle) || !read_uint(65535U,&distance) || !read_uint(255U,&energy))return input_error("input_format");
        sample.angle_tenths=(uint16_t)angle;sample.distance_mm=(uint16_t)distance;sample.energy=(uint8_t)energy;
        for(i=0U;i<(dual?2U:1U);i++){
          float x=0.0f,y=0.0f;const RadarMap_Params_t *p=dual?&candidates.params[i]:&params;
          bool allowed=RadarMap_SampleAllowed(p,&sample);
          bool inside=allowed && RadarMap_Project(p,&sample,&x,&y) && x>=0 && y>=0 && x<2400 && y<2400;
          if(dual){
            RadarStableBatch_RecordStats(&stable.automatic[i],1U,allowed?1U:0U,allowed && !inside?1U:0U);
            if(inside)RadarStableBatch_Add(&stable.automatic[i],sample.angle_tenths,sample.distance_mm,(uint16_t)x,(uint16_t)y);
          }else{
            RadarStable_RecordStats(&stable.rolling,1U,allowed?1U:0U,allowed && !inside?1U:0U);
            if(inside)RadarStable_Add(&stable.rolling,sample.angle_tenths,sample.distance_mm,(uint16_t)x,(uint16_t)y);
          }
        }
      }
      if(dual)for(i=0U;i<2U;i++)(void)RadarStableBatch_CommitCircle(&stable.automatic[i]);
      else (void)RadarStable_CommitCircle(&stable.rolling);
    }
    if(read_word(word))return input_error("input_format");
    if(automatic && !strcmp(mode,"auto")){
      for(i=0U;i<2U;i++){
        (void)RadarStableBatch_FinalizeBegin(&stable.automatic[i]);
        while(!RadarStableBatch_FinalizeStep(&stable.automatic[i],2048U)){}
        stable_points=RadarStableBatch_Points(&stable.automatic[i],&stable_count);
        RadarMap_BuildStable(&candidates.maps[i],&candidates.params[i],stable_points,stable_count);
      }
      RadarStart_Finish(&candidates,&start);selected_params=RadarStart_Params(&candidates,(RadarStart_Id_t)start.start_id);
      if(selected_params){
        i=start.start_id==RADAR_START_UPPER?0U:1U;map=candidates.maps[i];
        stable_points=RadarStableBatch_Points(&stable.automatic[i],&stable_count);
        stable_status=*RadarStableBatch_GetStatus(&stable.automatic[i]);
      }else {stable_count=0U;stable_status=*RadarStableBatch_GetStatus(&stable.automatic[0]);}
    }else{
      (void)RadarStable_FinalizeBegin(&stable.rolling);while(!RadarStable_FinalizeStep(&stable.rolling,2048U)){}
      stable_points=RadarStable_Points(&stable.rolling,&stable_count);stable_status=*RadarStable_GetStatus(&stable.rolling);
      RadarMap_BuildStable(&map,&params,stable_points,stable_count);RadarMap_Finish(&map,&params);
      if(!automatic)start.start_id=RADAR_START_LOWER;
      start.reason="stable_reference";
    }
    map.mask|=manual_mask;(void)RadarPlan_BuildFromStart(&map,(RadarStart_Id_t)start.start_id,&plan);
    print_result(&map,&plan,speed,automatic,&start,selected_params?selected_params:&params);return 0;
  }
  for(i=0U;i<sample_count;i++) {
    if(!read_uint(UINT16_MAX,&angle) || !read_uint(UINT16_MAX,&distance) || !read_uint(UINT8_MAX,&energy))
      return input_error("input_format");
    samples[i].angle_tenths=(uint16_t)angle;samples[i].distance_mm=(uint16_t)distance;
    samples[i].energy=(uint8_t)energy;
  }
  if(read_word(word)) return input_error("input_format");
  if(automatic) {
    for(i=0U;i<2U;i++)
      RadarMap_BuildCloud(&candidates.maps[i],&candidates.params[i],samples,(uint16_t)sample_count);
    RadarStart_Finish(&candidates,&start);
    if(strcmp(mode,"auto")) {
      start.start_id=!strcmp(mode,"upper")?RADAR_START_UPPER:RADAR_START_LOWER;
      start.reason="manual_selection";
    }
    selected_params=RadarStart_Params(&candidates,(RadarStart_Id_t)start.start_id);
    if(selected_params) {
      map=*RadarStart_Map(&candidates,(RadarStart_Id_t)start.start_id);
      RadarMap_Finish(&map,selected_params);
    }
  } else {
    RadarMap_BuildCloud(&map,&params,samples,(uint16_t)sample_count);
    RadarMap_Finish(&map,&params);
    start.start_id=RADAR_START_LOWER;start.reason="manual_reference";
  }
  map.mask|=manual_mask;
  (void)RadarPlan_BuildFromStart(&map,(RadarStart_Id_t)start.start_id,&plan);
  print_result(&map,&plan,speed,automatic,&start,selected_params);return 0;
}
