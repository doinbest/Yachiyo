#include "radar_map.h"
#include "radar_host_map.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Defaults must accept rays outside the formerly selected first quadrant. */
static void default_full_circle(void)
{
  RadarMap_Params_t p;RadarMap_t map;
  const RadarSample_t samples[]={{0,500,80},{900,500,80},{1800,500,80},{2700,500,80}};
  RadarMap_Defaults(&p);p.lidar_x_mm=p.lidar_y_mm=1200;
  RadarMap_BuildCloud(&map,&p,samples,4);
  assert(map.accepted_points==4);
  assert(map.counts[11]==1 && map.counts[17]==1 && map.counts[13]==1 && map.counts[7]==1);
  assert(map.mask==(RADAR_MAP_FIXED_MASK|(1UL<<11)|(1UL<<17)|(1UL<<13)|(1UL<<7)));
}

/* Field-exterior neighbours must not erase a valid field-interior return. */
static void keep_connected_inside_points(void)
{
  RadarMap_Params_t p;RadarMap_t map;
  const RadarSample_t samples[]={{0,1250,80},{0,1300,80},{0,1350,80},
    {0,1400,80},{0,1450,80},{0,1500,80},{0,1550,80},{0,1600,80},
    {0,1650,80},{0,1700,80},{0,1750,80},{0,1800,80},{0,1850,80},
    {0,1900,80},{0,1950,80},{0,2000,80},{0,2050,80}};
  RadarMap_Defaults(&p);p.lidar_x_mm=1900;p.lidar_y_mm=1200;
  RadarMap_BuildCloud(&map,&p,samples,sizeof(samples)/sizeof(samples[0]));
  assert(map.counts[11]==2 && map.counts[10]==8);
  assert(map.outside_points==7 && map.background_points==0);
  assert(map.accepted_points==10);
}

/* Independently separated points in one cell count toward the same threshold;
 * neighbouring cells cannot borrow support from one another. */
static void threshold_per_cell(void)
{
  RadarMap_Params_t p;RadarMap_t map;
  const RadarSample_t samples[]={{0,500,80},{0,800,80},{0,1000,80},{900,500,80}};
  RadarMap_Defaults(&p);p.lidar_x_mm=2250;p.lidar_y_mm=1200;p.threshold=2;
  RadarMap_BuildCloud(&map,&p,samples,4);
  assert(map.counts[13]==2 && map.counts[12]==1 && map.counts[19]==1);
  assert(map.mask==(RADAR_MAP_FIXED_MASK|(1UL<<13)));
}

/* Within the common grid, the same raw samples must reproduce the F1 counts
 * after the independently hand-specified 90 degree coordinate transform. */
static void rotated_f1_counts(void)
{
  RadarHostMap_Config_t host;RadarHostMap_Result_t result;
  RadarMap_Params_t p;RadarMap_t map;
  const RadarSample_t samples[]={{0,1800,80},{100,1700,80},{400,1500,80},
    {900,1000,80},{2700,1000,80},{1800,1000,80},{500,1500,0}};
  RadarHostMap_Defaults(&host);RadarMap_Defaults(&p);
  p.lidar_x_mm=2170;p.lidar_y_mm=230;p.zero_deg=180;
  const uint16_t ranges[][2]={{0,3600},{0,900},{3500,400},{0,0}};
  for(unsigned threshold=1;threshold<=2;threshold++)for(unsigned range=0;range<4;range++) {
    host.threshold=p.threshold=(uint16_t)threshold;
    host.angle_min_tenths=p.angle_min_tenths=ranges[range][0];
    host.angle_max_tenths=p.angle_max_tenths=ranges[range][1];
    host.energy_min=p.energy_min=range==3?1:0;
    assert(RadarHostMap_Build(&host,samples,7,&result));
    RadarMap_BuildCloud(&map,&p,samples,7);RadarMap_Finish(&map,&p);
    for(unsigned row=0;row<5;row++)for(unsigned col=0;col<5;col++)
      assert(map.counts[col*5+4-row]==result.counts[row*5+col]);
    assert(map.mask==RadarMap_LegacyMask(result.effective_mask));
  }
}

static void f1_angle_endpoint_semantics(void)
{
  RadarHostMap_Config_t host;RadarHostMap_Result_t h;
  RadarMap_Params_t p;RadarMap_t m;
  const RadarSample_t samples[]={{0,600,80},{900,600,80},{3500,600,80},{100,600,80}};
  RadarHostMap_Defaults(&host);RadarMap_Defaults(&p);
  host.radar_x_mm=host.radar_y_mm=1200;p.lidar_x_mm=p.lidar_y_mm=1200;
  host.angle_min_tenths=p.angle_min_tenths=3500;
  host.angle_max_tenths=p.angle_max_tenths=3600;
  assert(RadarHostMap_Build(&host,samples,4,&h));
  RadarMap_BuildCloud(&m,&p,samples,4);
  assert(h.filtered_points==0 && m.accepted_points==0);
  host.angle_max_tenths=p.angle_max_tenths=100;
  assert(RadarHostMap_Build(&host,samples,4,&h));
  RadarMap_BuildCloud(&m,&p,samples,4);
  assert(h.filtered_points==3 && m.accepted_points==3);
}

static void rotated_task_protection(void)
{
  RadarMap_Params_t p;RadarMap_t m;
  RadarMap_Defaults(&p);RadarMap_Reset(&m);
  for(unsigned i=0;i<25;i++)m.counts[i]=1;
  RadarMap_Finish(&m,&p);
  assert(m.evidence_mask==0x1FFFFFFUL);
  assert(m.mask==(0x1FFFFFFUL&~0x404414UL));
  p.lidar_y_mm=2250;RadarMap_Finish(&m,&p);
  assert(m.mask==(0x1FFFFFFUL&~0x1404404UL));
  assert(m.counts[2]==1 && m.counts[10]==1 && m.counts[24]==1);
}

static void rotated_boundary_ownership(void)
{
  RadarMap_Params_t p;RadarMap_t m;
  RadarSample_t s={0,500,80};
  RadarMap_Defaults(&p);p.zero_deg=0;p.lidar_x_mm=50;p.lidar_y_mm=1200;
  RadarMap_Reset(&m);assert(RadarMap_Add(&m,&p,&s));
  assert(m.counts[10]==1 && m.counts[11]==0); /* X=550 belongs to left. */
  p.lidar_x_mm=1750;RadarMap_Reset(&m);assert(RadarMap_Add(&m,&p,&s));
  assert(m.counts[14]==1); /* X=2250 included after reference rotation. */
  p.lidar_x_mm=-350;RadarMap_Reset(&m);assert(!RadarMap_Add(&m,&p,&s));
  const unsigned edges[]={550,1000,1400,1850};
  for(unsigned i=0;i<4;i++) {
    p.lidar_x_mm=50;p.lidar_y_mm=1200;p.zero_deg=0;RadarMap_Reset(&m);
    for(int delta=-1;delta<=1;delta++){s.distance_mm=(uint16_t)(edges[i]-50+delta);assert(RadarMap_Add(&m,&p,&s));}
    assert(m.counts[10+i]==2 && m.counts[11+i]==1);
    p.lidar_x_mm=1200;p.lidar_y_mm=50;p.zero_deg=90;RadarMap_Reset(&m);
    for(int delta=-1;delta<=1;delta++){s.distance_mm=(uint16_t)(edges[i]-50+delta);assert(RadarMap_Add(&m,&p,&s));}
    assert(m.counts[i*5+2]==1 && m.counts[(i+1)*5+2]==2);
  }
}

/* Compare complete ordered cell chains after expanding collinear waypoints.
 * The expected chain comes from the unchanged F1 search, not the formal wrapper. */
static void all_rotated_grid_routes(void)
{
  const RadarGrid_t reference={{150,550,1000,1400,1850,2250},{150,550,1000,1400,1850,2250}};
  const int16_t centers[]={350,775,1200,1625,2050};
  for(unsigned extra=0;extra<=25;extra++) {
    RadarMap_t m;RadarMap_Reset(&m);if(extra<25)m.mask|=1UL<<extra;
    uint32_t inverse=0;for(unsigned cell=0;cell<25;cell++)if(m.mask&(1UL<<cell))inverse|=1UL<<((4-cell%5)*5+cell/5);
    for(unsigned start=0;start<25;start++)for(unsigned goal=0;goal<25;goal++) {
      uint8_t expected[25],count;RadarPlan_t p;
      bool wanted=RadarGrid_Path(&reference,inverse,(uint8_t)((4-start%5)*5+start/5),(uint8_t)((4-goal%5)*5+goal/5),expected,&count);
      bool actual=RadarPlan_BuildLeg(&m,centers[start%5],centers[start/5],centers[goal%5],centers[goal/5],&p);
      assert(actual==wanted);if(!actual)continue;
      unsigned at=0,cell=start;assert(expected[at++]==(4-cell%5)*5+cell/5);
      for(unsigned point=1;point<p.count;point++) {
        unsigned row=0,col=0;while(row<5&&centers[row]!=p.points[point].y_mm)row++;
        while(col<5&&centers[col]!=p.points[point].x_mm)col++;
        assert(row<5&&col<5);unsigned to=row*5+col;
        assert(cell/5==to/5||cell%5==to%5);
        int step=cell/5==to/5?(to>cell?1:-1):(to>cell?5:-5);
        while(cell!=to){cell=(unsigned)((int)cell+step);assert(at<count&&expected[at++]==(4-cell%5)*5+cell/5);}
      }
      assert(at==count);
    }
  }
}

static void inset_outer_belt_matches_f1(void)
{
  RadarHostMap_Config_t host;RadarHostMap_Result_t h;
  RadarMap_Params_t p;RadarMap_t m;
  const RadarSample_t s[]={{0,1125,80},{900,1125,80},{1800,1125,80},{2700,1125,80}};
  RadarHostMap_Defaults(&host);host.radar_x_mm=host.radar_y_mm=1200;
  RadarMap_Defaults(&p);p.lidar_x_mm=p.lidar_y_mm=1200;
  assert(RadarHostMap_Build(&host,s,4,&h));RadarMap_BuildCloud(&m,&p,s,4);
  assert(h.outside_points==4 && m.outside_points==4 && m.accepted_points==0);
  assert(m.counts[10]==0 && m.counts[22]==0 && m.counts[14]==0 && m.counts[2]==0);
}

static void cardinal_directions(void)
{
  const RadarSample_t s[]={{0,600,80},{900,600,80},{1800,600,80},{2700,600,80}};
  const float expected[4][2]={{600,1200},{1200,1800},{1800,1200},{1200,600}};
  RadarMap_Params_t p;RadarMap_Defaults(&p);p.lidar_x_mm=p.lidar_y_mm=1200;
  for(unsigned i=0;i<4;i++) {
    float x,y;assert(RadarMap_Project(&p,&s[i],&x,&y));
    assert(fabsf(x-expected[i][0])<0.001f && fabsf(y-expected[i][1])<0.001f);
  }
  assert(RadarMap_LegacyMask(1UL<<17)==(1UL<<11)); /* R4C3→R3C2 */
  assert(RadarMap_LegacyMask(1UL<<24)==(1UL<<20)); /* R5C5→R5C1 */
  p.zero_deg=270;
  const float upper_expected[4][2]={{1200,600},{600,1200},{1200,1800},{1800,1200}};
  for(unsigned i=0;i<4;i++) {
    float x,y;assert(RadarMap_Project(&p,&s[i],&x,&y));
    assert(fabsf(x-upper_expected[i][0])<0.001f && fabsf(y-upper_expected[i][1])<0.001f);
  }
}

/* The rotated F1 tie order chooses west before south for this equal-cost L. */
static void rotated_equal_cost_grid_route(void)
{
  RadarMap_t m;RadarPlan_t p;
  RadarMap_Reset(&m);
  assert(RadarPlan_BuildLeg(&m,1200,1200,350,350,&p));
  assert(p.count==3);
  assert(p.points[1].x_mm==350 && p.points[1].y_mm==1200);
  /* A grid-center route must not be replaced by a different cross-grid L. */
  assert(RadarPlan_BuildLeg(&m,2100,1200,1200,2100,&p));
  assert(p.count==3);
  assert(p.points[1].x_mm==1200 && p.points[1].y_mm==1200);
}

/* The physical-wall belt is excluded; an independent point inside the inset
 * must still block its own cell even in the same circle as the wall. */
static RadarSample_t world_sample(float x,float y)
{
  RadarSample_t s;float dx=x-1200,dy=y-1200;
  float raw=180.0f-atan2f(dy,dx)*180.0f/3.14159265358979323846f;
  s.angle_tenths=(uint16_t)lroundf(fmodf(raw+360.0f,360.0f)*10.0f);
  s.distance_mm=(uint16_t)lroundf(hypotf(dx,dy));s.energy=80;return s;
}
static void supported_field_edge_only(void)
{
  RadarMap_Params_t p;RadarMap_t cloud_map,stream_map;
  RadarSample_t samples[14];
  RadarMap_Defaults(&p);p.lidar_x_mm=p.lidar_y_mm=1200;
  for(unsigned i=0;i<10;i++)samples[i]=world_sample(1400+100*i,i%2?2407:2393);
  samples[10]=world_sample(300,75); /* outer strip no longer participates */
  samples[11]=world_sample(800,1200); /* independent interior obstacle */
  samples[12]=world_sample(1500,2370); /* outside the inset, regardless of wall tolerance */
  samples[13]=world_sample(2000,2240); /* independent in-grid object */
  RadarMap_BuildCloud(&cloud_map,&p,samples,14);
  assert(cloud_map.counts[23]==0 && !(cloud_map.evidence_mask&(1UL<<23)));
  RadarMap_Finish(&cloud_map,&p);
  assert(cloud_map.mask==(RADAR_MAP_FIXED_MASK|(1UL<<11)|(1UL<<24)));
  assert(cloud_map.background_points==0 && cloud_map.accepted_points==2 && cloud_map.outside_points==12);
  /* The object at y=2240 survives; y=2370 is outside the selected grid. */
  RadarMap_Reset(&stream_map);
  for(unsigned i=0;i<14;i++)RadarMap_Add(&stream_map,&p,&samples[i]);
  RadarMap_Finish(&stream_map,&p);
  assert(stream_map.mask==cloud_map.mask && stream_map.background_points==0);
  assert(!memcmp(stream_map.counts,cloud_map.counts,sizeof stream_map.counts));
  RadarMap_BuildCloud(&cloud_map,&p,samples,12);RadarMap_Finish(&cloud_map,&p);
  assert(cloud_map.mask==(RADAR_MAP_FIXED_MASK|(1UL<<11)));
  /* Even an isolated wall-belt return is outside the restored grid. */
  RadarMap_BuildCloud(&cloud_map,&p,&samples[4],1);RadarMap_Finish(&cloud_map,&p);
  assert(cloud_map.mask==RADAR_MAP_FIXED_MASK && cloud_map.outside_points==1);
}

/* Actual scan 9 contains a continuous top edge as far in as Y=2372 mm.
 * The chosen inset excludes that wall, retaining detached in-grid objects. */
static void noisy_continuous_edge_keeps_detached_object(void)
{
  RadarMap_Params_t p;RadarMap_t m,stream;
  RadarSample_t samples[14];
  RadarMap_Defaults(&p);p.lidar_x_mm=p.lidar_y_mm=1200;
  for(unsigned i=0;i<12;i++)samples[i]=world_sample(600+100*i,i%2?2407:2374);
  samples[12]=world_sample(800,1200);
  samples[13]=world_sample(2000,2240);
  RadarMap_BuildCloud(&m,&p,samples,14);RadarMap_Finish(&m,&p);
  assert(m.mask==(RADAR_MAP_FIXED_MASK|(1UL<<11)|(1UL<<24)));
  assert(m.background_points==0);
  RadarMap_Reset(&stream);
  for(unsigned i=0;i<14;i++)RadarMap_Add(&stream,&p,&samples[i]);
  RadarMap_Finish(&stream,&p);
  assert(stream.mask==m.mask && stream.background_points==0);
  assert(!memcmp(stream.counts,m.counts,sizeof m.counts));
}

/* A complete circle may start in the middle of a wall, splitting its support
 * between the beginning/end of the cache. The two endpoints still join. */
static void edge_support_across_circle_seam(void)
{
  RadarMap_Params_t p;RadarMap_t m;RadarSample_t samples[11];
  RadarMap_Defaults(&p);p.lidar_x_mm=p.lidar_y_mm=1200;
  for(unsigned i=0;i<5;i++)samples[i]=world_sample(1100+100*i,i%2?2407:2374);
  samples[5]=world_sample(800,1200);
  for(unsigned i=0;i<5;i++)samples[6+i]=world_sample(600+100*i,i%2?2374:2407);
  RadarMap_BuildCloud(&m,&p,samples,11);RadarMap_Finish(&m,&p);
  assert(m.mask==(RADAR_MAP_FIXED_MASK|(1UL<<11)));
  assert(m.background_points==0 && m.accepted_points==1 && m.outside_points==10);
}
int main(int argc,char **argv)
{
  if(argc>1) {
    if(!strcmp(argv[1],"angle"))f1_angle_endpoint_semantics();
    else if(!strcmp(argv[1],"protect"))rotated_task_protection();
    else if(!strcmp(argv[1],"boundary"))rotated_boundary_ownership();
    else if(!strcmp(argv[1],"route"))rotated_equal_cost_grid_route();
    else if(!strcmp(argv[1],"wall")){supported_field_edge_only();noisy_continuous_edge_keeps_detached_object();edge_support_across_circle_seam();}
    else assert(0);
  } else {
    default_full_circle();keep_connected_inside_points();threshold_per_cell();rotated_f1_counts();
    cardinal_directions();inset_outer_belt_matches_f1();
    f1_angle_endpoint_semantics();rotated_task_protection();rotated_boundary_ownership();
    rotated_equal_cost_grid_route();
    all_rotated_grid_routes();
    supported_field_edge_only();
    noisy_continuous_edge_keeps_detached_object();edge_support_across_circle_seam();
  }
  puts("Radar point-count mapping: PASS");return 0;
}
