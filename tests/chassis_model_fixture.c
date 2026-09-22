/* JSON fixture from the production model and configuration; no board I/O. */
#include "chassis_model.h"
#include "chassis_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void scenario(const char *name,float vx,float vy,float omega)
{
  const ChassisModel_Geometry_t g={CHASSIS_WHEELBASE_MM,CHASSIS_TRACK_WIDTH_MM,CHASSIS_WHEEL_DIAMETER_MM,CHASSIS_MOTOR_TO_WHEEL_RATIO};
  const ChassisModel_Velocity_t input={vx,vy,omega};
  ChassisModel_Wheels_t wheels;
  ChassisModel_Velocity_t back,quantized;
  float commands[4];unsigned i;
  assert(ChassisModel_Inverse(&g,&input,CHASSIS_MOTION_MAX_WHEEL_RPM,&wheels));
  assert(ChassisModel_Forward(&g,wheels.rpm,&back));
  for(i=0;i<4;i++)commands[i]=(float)wheels.command[i];
  assert(ChassisModel_Forward(&g,commands,&quantized));
  printf("{\"kind\":\"scenario\",\"name\":\"%s\",\"body\":[%.9g,%.9g,%.9g],\"max_rpm\":%.9g,\"scale\":%.9g,",name,(double)vx,(double)vy,(double)omega,(double)CHASSIS_MOTION_MAX_WHEEL_RPM,(double)wheels.scale);
  printf("\"rpm\":[%.9g,%.9g,%.9g,%.9g],\"command\":[%d,%d,%d,%d],",(double)wheels.rpm[0],(double)wheels.rpm[1],(double)wheels.rpm[2],(double)wheels.rpm[3],wheels.command[0],wheels.command[1],wheels.command[2],wheels.command[3]);
  printf("\"forward\":[%.9g,%.9g,%.9g],\"quantized_forward\":[%.9g,%.9g,%.9g]}\n",(double)back.vx_mm_s,(double)back.vy_mm_s,(double)back.omega_rad_s,(double)quantized.vx_mm_s,(double)quantized.vy_mm_s,(double)quantized.omega_rad_s);
}
int main(void)
{
  float x,y,bx,by;
  const float rotation_distance=(CHASSIS_WHEELBASE_MM+CHASSIS_TRACK_WIDTH_MM)*.5f*CHASSIS_MODEL_PI*.5f;
  const float pulses_per_mm=CHASSIS_MOTOR_TO_WHEEL_RATIO*CHASSIS_COMMAND_PULSES_PER_REV/(CHASSIS_MODEL_PI*CHASSIS_WHEEL_DIAMETER_MM);
  scenario("forward",100,0,0);scenario("left",0,100,0);scenario("yaw",0,0,.1f);
  scenario("combined_scaled",1600,1200,.8f);scenario("low_8",8,0,0);scenario("low_15",15,0,0);scenario("negative_15",-15,0,0);
  assert(ChassisModel_BodyToMap(100,0,CHASSIS_MODEL_PI*.5f,&x,&y));
  assert(ChassisModel_MapToBody(100,0,CHASSIS_MODEL_PI*.5f,&bx,&by));
  printf("{\"kind\":\"rotation90\",\"map\":[%.9g,%.9g],\"body\":[%.9g,%.9g]}\n",(double)x,(double)y,(double)bx,(double)by);
  /* Baseline command-pulse conversion, distinct from 65536 protocol position units. */
  printf("{\"kind\":\"pulses\",\"pulses_per_rev\":%u,\"forward100\":%ld,\"rotation90\":%ld}\n",(unsigned)CHASSIS_COMMAND_PULSES_PER_REV,lroundf(100*pulses_per_mm),lroundf(rotation_distance*pulses_per_mm));
  return 0;
}
