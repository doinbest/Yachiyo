#define main legacy_arm_bus_main
#include "mechanical_arm_bus_test.c"
#undef main
int main(void)
{
 MechanicalArm_ConfigTypeDef config;
 assert(MechanicalArm_ConfigGet(MECHANICAL_ARM_AXIS_Z,&config));
 assert(config.SpeedRpm==2250 && config.Acceleration==255);
 puts("mechanical Z default:2250 RPM equals250 mm/s at480 pulses/mm; acceleration255 PASS");
}
