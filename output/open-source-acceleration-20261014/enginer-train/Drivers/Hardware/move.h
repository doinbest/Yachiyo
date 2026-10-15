#ifndef __move_H
#define __move_H
#include "main.h"
#include "pid.h"
typedef struct {
		float DIR; 
    float accel;     
    float decel;    
		float accel_step;
		float decel_step;
		float constant_speed_steps;
    float max_speed; 
    int total_steps; 
} MotorControlParams;

extern PID_t Position_Pid,Angle_Pid_x,Angle_Pid_y;

void Set_step_params(MotorControlParams *params,uint8_t dir,float acc,float dec,float max_speed,float distance);
void Step_run_acc_dec(MotorControlParams *params);

void Car_Run_Vx_Vy(float Vx, float Vy, float W,uint16_t acc);
void Car_Run_Vx_Vy_W_L(float Vx, float Vy, float W, float L,uint16_t acc);
void move(uint8_t dir,uint32_t step,uint16_t speed,uint16_t acc,uint16_t dec);
void set_allcur_position_zero(void);
void move_py(uint8_t dir,uint32_t step,uint16_t speed,uint16_t acc,uint16_t dec,uint32_t py_distance,float aim_angle,float W);

void material_dingwei(void);
void material_color_dingwei(uint8_t color);
void circle_dingwei(uint8_t color);
void circle_dingwei_cu(uint8_t color);
uint8_t turn_haf(float target_angle);
void home_dingwei(void);
void move_global(float global_x,float global_y);

#endif
