/* Source prepared first; no C compiler/executable is run in this round. */
#define main legacy_station_heading_main
#include "station_heading_test.c"
#undef main
static void no_mapping(void)
{
 config.zero_deg=config.gain=config.tolerance_deg=config.spread_deg=config.settle_ms=config.turn_limit_deg=NAN;
 config.wait_ms=0;config.height_matches=config.direct_eligible=true;
}
static void direct_turn_and_recheck(unsigned at,float nominal_yaw,int image,float limit)
{
 reset();no_mapping();motion.map_yaw_rad=nominal_yaw*CHASSIS_MODEL_PI/180;
 StationHeading_CompetitionBeginSource(at,nominal_yaw,limit,STATION_HEADING_RING);
 assert(!strcmp(StationHeading_Method(),"direct_ring"));
 assert(StationHeading_TurnSpeed()==fminf(limit,10));assert(!StationHeading_Process());
 assert(!sample(image,0));assert(!sample(image,1));assert(!sample(image,0));
 assert(commits==1 && !turns);
 float observed=motion.map_yaw_rad*180/CHASSIS_MODEL_PI;
 assert(fabsf(remainderf(observed-(nominal_yaw+image*.01f),360))<.001f);
 turn_accept=false;for(unsigned i=0;i<6;i++)assert(!StationHeading_Process());
 assert(commits==1 && !turns); /* Busy retries cannot add the same visual angle. */
 turn_accept=true;assert(!StationHeading_Process());
 assert(turns==1 && turn_speed==fminf(limit,10) && turn_tolerance==.1f);
 assert(fabsf(turn_delta+image*.01f)<.001f && turn_yaw==nominal_yaw);
 unsigned old_requests=requests;
 tick+=6000;assert(!StationHeading_Process()); /* Never skip a moving correction at 5 s. */
 motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=false;
 assert(!StationHeading_Process() && requests==old_requests);
 motion.stop_confirmed=true;motion.map_yaw_rad=nominal_yaw*CHASSIS_MODEL_PI/180;
 assert(!StationHeading_Process() && requests==old_requests+1);
 assert(!StationHeading_Process()); /* Old image is not post-turn evidence. */
 assert(!sample(19,1));assert(!sample(19,0));assert(sample(19,1));
 assert(!strcmp(StationHeading_Reason(),"heading_verified") && turns==1);
 assert(fabsf(StationHeading_BodyError()-.19f)<.001f);
 assert(fabsf(remainderf(motion.map_yaw_rad*180/CHASSIS_MODEL_PI-nominal_yaw,360))<.001f);
}
int main(void)
{
 for(unsigned station_id=2;station_id<=3;station_id++)for(int sign=-1;sign<=1;sign+=2)
  direct_turn_and_recheck(station_id,station_id==2?180:90,sign*1092,100);
 direct_turn_and_recheck(2,180,45,5); /* Latest real image0.45, independent5deg/s cap. */
 reset();no_mapping();StationHeading_CompetitionBeginSource(2,180,100,STATION_HEADING_RING);
 StationHeading_Process();sample(10,0);sample(20,1);assert(sample(15,0));
 assert(!turns && !commits && !strcmp(StationHeading_Reason(),"heading_verified"));
 assert(fabsf(StationHeading_ImageAngle()-.15f)<.001f);
 /* Independent observation/mapped paths and black-line qualification remain separate. */
 reset();no_mapping();config.wait_ms=5000;StationHeading_ObserveBeginSource(2,180,5,STATION_HEADING_RING);
 StationHeading_Process();sample(1092,0);sample(1092,1);assert(sample(1092,0));
 assert(!turns && !commits && !strcmp(StationHeading_Reason(),"heading_observed"));
 reset();no_mapping();config.wait_ms=5000;StationHeading_CompetitionBeginSource(3,90,100,STATION_HEADING_LINE);
 StationHeading_Process();sample(50,0);sample(50,1);assert(sample(50,0));
 assert(!turns && !commits && !strcmp(StationHeading_Reason(),"heading_uncalibrated"));
 for(unsigned invalid=0;invalid<2;invalid++){
  reset();no_mapping();if(invalid)config.direct_eligible=false;else config.height_matches=false;
  StationHeading_CompetitionBeginSource(2,180,100,STATION_HEADING_RING);
  StationHeading_Process();sample(50,0);sample(50,1);assert(sample(50,0));
  assert(!turns && !commits);
  assert(!strcmp(StationHeading_Reason(),invalid?"heading_reference_unverified":"heading_height_unmatched"));
 }
 reset();no_mapping();StationHeading_CompetitionBeginSource(2,180,100,STATION_HEADING_RING);
 StationHeading_Process();sample(45,0);sample(45,1);
 StationHeading_Pause();tick+=20000;StationHeading_Process();
 assert(!sample(45,0));assert(!sample(45,0));assert(!sample(45,0));
 assert(commits==1 && !turns);StationHeading_Process();assert(turns==1);
 reset();no_mapping();StationHeading_CompetitionBeginSource(2,180,100,STATION_HEADING_RING);
 StationHeading_Process();tick=5000;assert(StationHeading_Process());
 assert(!turns && !commits && !strcmp(StationHeading_Reason(),"heading_no_observation"));
 puts("competition heading: verified inverse1:1, defaults,10deg/s cap, fresh post-stop evidence and legacy boundaries PASS");
 return 0;
}
