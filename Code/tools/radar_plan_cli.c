/**
 * @file radar_plan_cli.c
 * @brief Same C map/planner as the MCU, with a streaming host-side text adapter.
 *
 * stdin (whitespace separated, decimal integers):
 * RADAR1 manual_mask sample_count
 * lidar_x_mm lidar_y_mm zero_deg distance_min_mm distance_max_mm
 * angle_min_tenths angle_max_tenths threshold energy_min energy_max
 * Then sample_count triples: angle_tenths distance_mm energy.
 * N=0 plans a manual mask; N>0 counts the cloud, then ORs manual_mask.
 * stdout: one JSON object with algorithm, valid, failed_leg, mask, counts,
 * points[[x_mm,y_mm,station,visit],...]. No-path is a normal result (exit 0).
 * Malformed input/invalid params give {"error":"..."} and exit 2.
 *
 * Build from repository root:
 * gcc -std=c99 -O2 -Wall -Wextra -Werror -ICode/template/App
 *   Code/tools/radar_plan_cli.c Code/template/App/radar_map.c -lm -o radar_plan_cli
 */
#include "radar_map.h"

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

static void print_result(const RadarMap_t *map,const RadarPlan_t *plan)
{
  unsigned i;
  printf("{\"algorithm\":\"radar_map_c_v1\",\"valid\":%s,\"failed_leg\":%u,\"mask\":%" PRIu32 ",\"counts\":[",
         plan->valid?"true":"false",(unsigned)plan->failed_leg,map->mask);
  for(i=0U;i<RADAR_MAP_CELL_COUNT;i++) printf("%s%u",i?",":"",(unsigned)map->counts[i]);
  printf("],\"points\":[");
  for(i=0U;i<plan->count;i++) {
    const RadarPlan_Point_t *point=&plan->points[i];
    printf("%s[%d,%d,%u,%u]",i?",":"",(int)point->x_mm,(int)point->y_mm,
           (unsigned)point->station,(unsigned)point->visit);
  }
  puts("]}");
}

int main(void)
{
  char word[96];
  uint32_t manual_mask,sample_count,i,angle,distance,energy;
  RadarMap_Params_t params;
  RadarMap_t map;
  RadarPlan_t plan;
  RadarSample_t sample;
  if(!read_word(word) || strcmp(word,"RADAR1") ||
     !read_uint(RADAR_MAP_ALL_CELLS,&manual_mask) || !read_uint(UINT32_MAX,&sample_count) ||
     !read_params(&params)) return input_error("input_format");
  if(!RadarMap_ParamsValid(&params)) return input_error("invalid_params");
  RadarMap_Reset(&map);
  for(i=0U;i<sample_count;i++) {
    if(!read_uint(UINT16_MAX,&angle) || !read_uint(UINT16_MAX,&distance) || !read_uint(UINT8_MAX,&energy))
      return input_error("input_format");
    sample.angle_tenths=(uint16_t)angle;sample.distance_mm=(uint16_t)distance;sample.energy=(uint8_t)energy;
    (void)RadarMap_Add(&map,&params,&sample);
  }
  if(read_word(word)) return input_error("input_format");
  RadarMap_Finish(&map,&params);map.mask|=manual_mask;
  (void)RadarPlan_Build(&map,&plan);
  print_result(&map,&plan);return 0;
}
