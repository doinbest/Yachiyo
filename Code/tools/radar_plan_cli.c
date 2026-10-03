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
#include "chassis_config.h"

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Host-only storage, matching the MCU's existing full-cloud capacity. */
static RadarSample_t samples[4096];

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

static void print_profile(const RadarPlan_Point_t *points,unsigned count,float speed)
{
  unsigned segment,index,emitted=0U;
  float yaw=90.0f;
  bool first_scan_leg=true;
  ChassisRoute_Profile_t profile;
  printf("{\"points\":");print_points(points,count);printf(",\"phases\":[");
  for(segment=1U;segment<count;segment++) {
    const RadarPlan_Point_t *to=&points[segment];
    if(!ChassisRouteProfile_Build(&points[segment-1U],to,yaw,speed,first_scan_leg,&profile)) break;
    for(index=0U;index<profile.count;index++) {
      const ChassisRoute_Phase_t *phase=&profile.phases[index];
      printf("%s{\"kind\":\"%s\",\"phase\":\"%s\",\"direction\":\"%s\",\"segment\":%u,"
             "\"station\":%u,\"visit\":%u,\"target\":[%.6g,%.6g,%.6g],\"speed_mm_s\":%.6g}",
             emitted++?",":"",phase->turn?"turn":"move",phase->phase,phase->direction,segment,
             (unsigned)to->station,(unsigned)to->visit,(double)phase->x_mm,(double)phase->y_mm,
             (double)phase->yaw_deg,(double)phase->speed_mm_s);
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
  printf("{\"algorithm\":\"radar_map_c_v1\",\"valid\":%s,\"failed_leg\":%u,\"mask\":%" PRIu32 ",\"counts\":[",
         plan->valid?"true":"false",(unsigned)plan->failed_leg,map->mask);
  for(i=0U;i<RADAR_MAP_CELL_COUNT;i++) printf("%s%u",i?",":"",(unsigned)map->counts[i]);
  printf("],\"points\":");print_points(plan->points,plan->count);
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
  for(i=0U;i<=CHASSIS_ROUTE_TARGET_COUNT;i++) (void)ChassisRouteProfile_FixedPoint(i,&fixed[i]);
  printf(",\"route_profiles\":{\"fixed\":");
  print_profile(fixed,CHASSIS_ROUTE_TARGET_COUNT+1U,speed);
  printf(",\"planned\":");print_profile(plan->points,
      plan->valid && start->start_id==RADAR_START_LOWER?plan->count:0U,speed);
  puts("}}");
}

int main(int argc,char *argv[])
{
  char word[96];
  float speed=CHASSIS_ROUTE_DEFAULT_SPEED_MM_S;
  uint32_t manual_mask,sample_count,i,angle,distance,energy;
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
  if(!read_word(word) || strcmp(word,"RADAR1") ||
     !read_uint(RADAR_MAP_ALL_CELLS,&manual_mask) || !read_uint(4096U,&sample_count) ||
     !read_params(&params)) return input_error("input_format");
  if(!RadarMap_ParamsValid(&params)) return input_error("invalid_params");
  RadarMap_Reset(&map);
  automatic=strcmp(mode,"manual")!=0;
  if(automatic) RadarStart_Begin(&candidates,&params);
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
