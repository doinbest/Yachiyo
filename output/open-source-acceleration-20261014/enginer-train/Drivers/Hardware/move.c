#include "stm32f4xx.h"                  // Device header
#include "emm_v5.h"
#include "math.h"
#include "move.h"
#include "pid.h"
#include "hwt101.h"
#include "delay.h"
#include "serial_task.h"
#include "pwm.h"
#include "fashion_star_uart_servo.h"
#include "serial.h"

extern Usart_DataTypeDef *servo_usart;

#define ZDT_ID_Addr_1  1
#define ZDT_ID_Addr_2  2
#define ZDT_ID_Addr_3  3
#define ZDT_ID_Addr_4  4

#define ZDT_DIR_CCW    0
#define ZDT_DIR_CW     1

PID_t Position_Pid_x=
{
	.KP=1.0f,
	.KD=0.0f,
	.OUTMAX=20.0,
	.OUTMIN=-20.0,
};

PID_t Position_Pid_y=
{
	.KP=1.0f,
	.KD=0.0f,
	.OUTMAX=20.0,
	.OUTMIN=-20.0,
};

PID_t Angle_Pid=
{
	.KP=-2.3f,
	.KD=-0.03,
	.OUTMAX=10.0,
	.OUTMIN=-10.0,
};

PID_t Blance_Pid=
{
	.KP=-1.0f,
	.KD=-0.05,
	.OUTMAX=10.0,
	.OUTMIN=-10.0,
};

void Car_Run_Vx_Vy_W_L(float Vx, float Vy, float W, float L,uint16_t acc)
{
     float V1 = 0,V2 = 0,V3 = 0,V4 = 0;
     uint32_t clk = 0;

			clk=L;
			
     V1 = Vx + Vy + W;
     V2 = Vy - Vx + W;
     V3 = Vy - Vx - W;
     V4 = Vx + Vy - W;

     if(V1 < 0)
     {
        V1 *= (-1);
        Emm_V5_Pos_Control(ZDT_ID_Addr_1,ZDT_DIR_CW,(uint16_t)V1,acc,clk, 0, 0);
     }
     else if ((V1 > 0) || (V1 == 0))
     {
        Emm_V5_Pos_Control(ZDT_ID_Addr_1,ZDT_DIR_CCW,(uint16_t)V1,acc,clk,0,0);
     }
     HAL_Delay(1); 

     if(V2 < 0)
     {
        V2 *= (-1);
        Emm_V5_Pos_Control(ZDT_ID_Addr_2,ZDT_DIR_CCW,(uint16_t)V2,acc,clk,0,1);
     }
     else if ((V2 > 0) || (V2 == 0))
     {
        Emm_V5_Pos_Control(ZDT_ID_Addr_2,ZDT_DIR_CW,(uint16_t)V2,acc,clk,0,1);
     }
     HAL_Delay(1); 

     if(V3 < 0)
     {
				V3 *= (-1);
        Emm_V5_Pos_Control(ZDT_ID_Addr_3,ZDT_DIR_CCW,(uint16_t)V3,acc,clk,0,1);
     }
     else if ((V3 > 0) || (V3 == 0))
     {
        Emm_V5_Pos_Control(ZDT_ID_Addr_3,ZDT_DIR_CW,(uint16_t)V3,acc,clk,0,1);
     }
     HAL_Delay(1); 

     if(V4 < 0)
     {
        V4 *= (-1);
        Emm_V5_Pos_Control(ZDT_ID_Addr_4,ZDT_DIR_CW,(uint16_t)V4,acc,clk,0,1);
     }
     else if ((V1 > 0) || (V1 == 0))
     {
        Emm_V5_Pos_Control(ZDT_ID_Addr_4,ZDT_DIR_CCW,(uint16_t)V4,acc,clk,0,1);
     }
     HAL_Delay(1); 

		Emm_V5_Synchronous_motion(0);
    HAL_Delay(1); 
}

void Car_Run_Vx_Vy(float Vx, float Vy, float W,uint16_t acc)
{
     float V1 = 0,V2 = 0,V3 = 0,V4 = 0;
	
	   V1 = Vx + Vy + W;
     V2 = Vy - Vx + W;
     V3 = Vy - Vx - W;
     V4 = Vx + Vy - W;
     if(V1 < 0)
     {
        V1 *= (-1);
        Emm_V5_Vel_Control(ZDT_ID_Addr_1,ZDT_DIR_CW,(uint16_t)V1,acc,1);
     }
     else if ((V1 > 0) || (V1 == 0))
     {
        Emm_V5_Vel_Control(ZDT_ID_Addr_1,ZDT_DIR_CCW,(uint16_t)V1,acc,1);
     }
			delay_ms(1); 

     if(V2 < 0)
     {
        V2 *= (-1);
        Emm_V5_Vel_Control(ZDT_ID_Addr_2,ZDT_DIR_CCW,(uint16_t)V2,acc,1);
     }
     else if ((V2 > 0) || (V2 == 0))
     {
        Emm_V5_Vel_Control(ZDT_ID_Addr_2,ZDT_DIR_CW,(uint16_t)V2,acc,1);
     }
			delay_ms(1); 

     if(V3 < 0)
     {
				V3 *= (-1);
        Emm_V5_Vel_Control(ZDT_ID_Addr_3,ZDT_DIR_CCW,(uint16_t)V3,acc,1);
     }
     else if ((V3 > 0) || (V3 == 0))
     {
        Emm_V5_Vel_Control(ZDT_ID_Addr_3,ZDT_DIR_CW,(uint16_t)V3,acc,1);
     }
			delay_ms(1); 

     if(V4 < 0)
     {
        V4 *= (-1);
        Emm_V5_Vel_Control(ZDT_ID_Addr_4,ZDT_DIR_CW,(uint16_t)V4,acc,1);
     }
     else if ((V4 > 0) || (V4 == 0))
     {
        Emm_V5_Vel_Control(ZDT_ID_Addr_4,ZDT_DIR_CCW,(uint16_t)V4,acc,1);
     }
			delay_ms(1); 
			Emm_V5_Synchronous_motion(0);
			delay_ms(1); 
}
float position1,position2,position3,position4;
void set_allcur_position_zero(void)
{
	position1=0;position2=0,position3=0;position4=0;
	set_cur_position_zero(1);
	set_cur_position_zero(2);
	set_cur_position_zero(3);
	set_cur_position_zero(4);
}

void Set_step_params(MotorControlParams *params,uint8_t dir,float acc,float dec,float max_speed,float distance)
{
	params->DIR=dir;
	params->accel=acc;
	params->decel=dec;
	params->total_steps=distance;
	params->max_speed=max_speed;
}

void calculate_step_distance(MotorControlParams *params)
{
	params->accel_step=(params->max_speed*params->max_speed)/(2*params->accel);
	params->decel_step=(params->max_speed*params->max_speed)/(2*params->decel);
	
	float min_distance=params->accel_step+params->decel_step;
	if(params->total_steps<=min_distance)
	{
		params->accel_step=(params->total_steps*params->decel)/(params->accel+params->decel);
		params->decel_step=(params->total_steps*params->accel)/(params->accel+params->decel);
		params->constant_speed_steps=0.0;
	}
	else
		params->constant_speed_steps=params->total_steps-params->accel_step-params->decel_step;
}

float target_speed;
float cur_pos,remain_distance;
void Step_run_acc_dec(MotorControlParams *params)
{
	calculate_step_distance(params);
	float v0=1.0f;
	cur_pos=0.0,remain_distance=0.0;
	while(fabsf(params->total_steps-cur_pos)>=2.0f && cur_pos<=params->total_steps)
	{
		position1=read_cur_position(1);
    delay_ms(5); 
		position2=read_cur_position(2);
    delay_ms(5); 
		position3=read_cur_position(3);
    delay_ms(5); 
		position4=read_cur_position(4);
    delay_ms(5); 
		
//		if(params->DIR==0 || params->DIR==1)
//			cur_pos=fabsf((position1-position2+position3-position4)/4);
//		else if(params->DIR==2 || params->DIR==3)
//			cur_pos=fabsf((position1+position2-position3-position4)/4);
//		else if(params->DIR==4)
//			cur_pos=fabsf((position1+position2-position3-position4)/2);
		cur_pos=fabsf(position1)+fabsf(position2)+fabsf(position3)+fabsf(position4);
		remain_distance=params->total_steps-cur_pos;
		if(cur_pos<=params->accel_step)
		{
			target_speed = sqrtf(v0*v0+ 2*params->accel*cur_pos);
		}
		else if(cur_pos>params->accel_step && cur_pos<(params->accel_step+params->constant_speed_steps))
		{
			target_speed=params->max_speed;
		}
		else if(cur_pos>=(params->accel_step+params->constant_speed_steps))
		{
			target_speed = sqrtf(2*params->decel*remain_distance);
		}
		Angle_Pid.Actual=global_angle_all;
		
		PID_Update(&Angle_Pid);
		if(fabsf(Angle_Pid.Err)<=3.0f)
		{
			Angle_Pid.OUTMAX=10.0;
			Angle_Pid.OUTMIN=-10.0;
		}
		if(params->DIR==1)
			Car_Run_Vx_Vy(target_speed,0,Blance_Pid.Out,0);
		else if(params->DIR==0)
			Car_Run_Vx_Vy(-target_speed,0,Blance_Pid.Out,0);
		else if(params->DIR==2)
			Car_Run_Vx_Vy(0,target_speed,Blance_Pid.Out,0);
		else if(params->DIR==3)
			Car_Run_Vx_Vy(0,-target_speed,Blance_Pid.Out,0);
		else if(params->DIR==4)
			Car_Run_Vx_Vy(target_speed,-target_speed,Blance_Pid.Out,0);
		else if(params->DIR==5)
			Car_Run_Vx_Vy(-target_speed,-target_speed,Blance_Pid.Out,0);
	}
	
	Car_Run_Vx_Vy(0,0,0,0);
  delay_ms(1); 
	set_allcur_position_zero();
}
#define red  	1
#define green 2
#define blue  3
float center_x=180.0,center_y=110.0;
float error_view;
void dingwei(float aim_x,float aim_y,float error)
{
	while(fabsf(blob_float_data[0]-aim_x)>error || fabsf(blob_float_data[1]-aim_y)>error ||fabsf(Angle_Pid.Err)>0.5)
	{
		Position_Pid_x.Actual=blob_float_data[0];
		Position_Pid_x.Target=aim_x;
		PID_Update(&Position_Pid_x);
		
		Position_Pid_y.Actual=blob_float_data[1];
		Position_Pid_y.Target=aim_y;
		PID_Update(&Position_Pid_y);
		
		Angle_Pid.Actual=global_angle_all;
		PID_Update(&Angle_Pid);
		
		if(fabsf(Position_Pid_x.Out)<1.0f)
		{
			if(Position_Pid_x.Out>0)
				Position_Pid_x.Out=1;
			else if(Position_Pid_x.Out<0)
				Position_Pid_x.Out=-1;
		}
		if(fabsf(Position_Pid_y.Out)<1.0f)
		{
			if(Position_Pid_y.Out>0)
				Position_Pid_y.Out=1;
			else if(Position_Pid_y.Out<0)
				Position_Pid_y.Out=-1;
		}
//		if(fabsf(Angle_Pid.Out)<1)
//		{
//			if(Angle_Pid.Out>0)
//				Angle_Pid.Out=1;
//			else if(Angle_Pid.Out<0)
//				Angle_Pid.Out=-1;
//		}
		Car_Run_Vx_Vy(Position_Pid_x.Out,Position_Pid_y.Out,Angle_Pid.Out,0);
	}
		Car_Run_Vx_Vy(0,0,0,0);
}
//小车移动函数 1前 0后 4左上 3后 2右
extern MotorControlParams STEP;
void move(uint8_t dir,uint32_t step,uint16_t speed,uint16_t acc,uint16_t dec)
{
	set_allcur_position_zero();
	STEP.DIR=dir;
	STEP.total_steps=step;
	STEP.accel=acc;
	STEP.decel=dec;
	STEP.max_speed=speed;
	Step_run_acc_dec(&STEP);
}
void material_dingwei(void)
{
	Position_Pid_x.OUTMIN=-20.0;
	Position_Pid_x.OUTMAX=20.0;
	Position_Pid_y.OUTMIN=-20.0;
	Position_Pid_y.OUTMAX=20.0;
	dingwei(center_x,center_y,2);
	HAL_Delay(50);
	dingwei(center_x,center_y,2);
}
//色环定位时的参数 调定位时修改中心点
/*
			y大
	x大	  	x小
				y小
*/
float circle_center_x=129.5,circle_center_y=60.1;//134.5,64.1
//物料定位函数
void material_color_dingwei(uint8_t color)
{
	data_reset();
	if(color==red)
		Serial_Printf(&huart3,"444");
	else if(color==green)
		Serial_Printf(&huart3,"555");
	else if(color==blue)
		Serial_Printf(&huart3,"666");
	Position_Pid_x.OUTMIN=-10.0;
	Position_Pid_x.OUTMAX=10.0;
	Position_Pid_y.OUTMIN=-10.0;
	Position_Pid_y.OUTMAX=10.0;
	while(blob_float_data[0]==0.0 ||blob_float_data[1]==0.0);
	dingwei(131,71,1);
	HAL_Delay(50);
	dingwei(131,71,1);
}
//色环定位
void circle_dingwei(uint8_t color)
{
	if(color==red)
		Serial_Printf(&huart3,"111");
	else if(color==green)
		Serial_Printf(&huart3,"222");
	else if(color==blue)
		Serial_Printf(&huart3,"333");
	Position_Pid_x.OUTMIN=-10.0;
	Position_Pid_x.OUTMAX=10.0;
	Position_Pid_y.OUTMIN=-10.0;
	Position_Pid_y.OUTMAX=10.0;
	dingwei(circle_center_x,circle_center_y,0.5);
	HAL_Delay(80);
	dingwei(circle_center_x,circle_center_y,0.5);
}
//色环粗定位
void circle_dingwei_cu(uint8_t color)
{
	if(color==red)
		Serial_Printf(&huart3,"111");
	else if(color==green)
		Serial_Printf(&huart3,"222");
	else if(color==blue)
		Serial_Printf(&huart3,"333");
	Position_Pid_x.OUTMIN=-20.0;
	Position_Pid_x.OUTMAX=20.0;
	Position_Pid_y.OUTMIN=-20.0;
	Position_Pid_y.OUTMAX=20.0;
	dingwei(circle_center_x,circle_center_y,1);
	HAL_Delay(20);
	dingwei(circle_center_x,circle_center_y,1);
}
//转弯函数
uint8_t turn_haf(float target_angle)
{	
	Angle_Pid.OUTMAX=250;
	Angle_Pid.OUTMIN=-250;
	while(1)
	{
		if(fabsf(global_angle_all-target_angle)<=0.5)
		{			
			Car_Run_Vx_Vy(0,0,0,0);
			Angle_Pid.OUTMAX=10;
			Angle_Pid.OUTMIN=-10;
			return 0;
		}
		else 
		{
			Angle_Pid.Target=target_angle;
			Angle_Pid.Actual=global_angle_all;
			PID_Update(&Angle_Pid);
			
			if(fabsf(Angle_Pid.Out)<1)
			{
				if(Angle_Pid.Out>0)
					Angle_Pid.Out=1;
				else if(Angle_Pid.Out<0)
					Angle_Pid.Out=-1;
			}
			
			Car_Run_Vx_Vy(0,0,Angle_Pid.Out,200);
		}
	}
}

void Step_run_acc_dec_py(MotorControlParams *params,uint32_t py_distance,float aim_angle,float W)
{
	calculate_step_distance(params);
	float v0=1.0f;
	cur_pos=0.0,remain_distance=0.0;
	while(fabsf(params->total_steps-cur_pos)>=2.0f && cur_pos<=params->total_steps)
	{
		position1=read_cur_position(1);
    delay_ms(5); 
		position2=read_cur_position(2);
    delay_ms(5); 
		position3=read_cur_position(3);
    delay_ms(5); 
		position4=read_cur_position(4);
    delay_ms(5); 
		cur_pos=fabsf(position1)+fabsf(position2)+fabsf(position3)+fabsf(position4);
		if(cur_pos>=py_distance)
		{
			Angle_Pid.Target=aim_angle;
			Angle_Pid.OUTMIN=-W;
			Angle_Pid.OUTMAX=W;
		}
		remain_distance=params->total_steps-cur_pos;
		if(cur_pos<=params->accel_step)
		{
			target_speed = sqrtf(v0*v0+ 2*params->accel*cur_pos);
		}
		else if(cur_pos>params->accel_step && cur_pos<(params->accel_step+params->constant_speed_steps))
		{
			target_speed=params->max_speed;
		}
		else if(cur_pos>=(params->accel_step+params->constant_speed_steps))
		{
			target_speed = sqrtf(2*params->decel*remain_distance);
		}
		Angle_Pid.Actual=global_angle_all;
		
		PID_Update(&Angle_Pid);
		if(fabsf(Angle_Pid.Err)<=3.0f)
		{
			Angle_Pid.OUTMAX=10.0;
			Angle_Pid.OUTMIN=-10.0;
		}
		if(params->DIR==1)
			Car_Run_Vx_Vy(target_speed,0,Angle_Pid.Out,0);
		else if(params->DIR==0)
			Car_Run_Vx_Vy(-target_speed,0,Angle_Pid.Out,0);
		else if(params->DIR==2)
			Car_Run_Vx_Vy(0,target_speed,Angle_Pid.Out,0);
		else if(params->DIR==3)
			Car_Run_Vx_Vy(0,-target_speed,Angle_Pid.Out,0);
		else if(params->DIR==4)
			Car_Run_Vx_Vy(target_speed,-target_speed,Angle_Pid.Out,0);
	}
	
	Car_Run_Vx_Vy(0,0,0,0);
  delay_ms(1); 
	set_allcur_position_zero();
}
//小车漂移函数 pydistance 为一段路径中开始漂移的起点，aim_angle为飘移后的目标角度，w为角速度
void move_py(uint8_t dir,uint32_t step,uint16_t speed,uint16_t acc,uint16_t dec,uint32_t py_distance,float aim_angle,float W)
{
	set_allcur_position_zero();
	STEP.DIR=dir;
	STEP.total_steps=step;
	STEP.accel=acc;
	STEP.decel=dec;
	STEP.max_speed=speed;
	Step_run_acc_dec_py(&STEP,py_distance,aim_angle,W);
}
//回启停区的定位
void home_dingwei(void)
{
	Serial_Printf(&huart3,"999");
	Position_Pid_x.OUTMIN=-15.0;
	Position_Pid_x.OUTMAX=15.0;
	Position_Pid_y.OUTMIN=-15.0;
	Position_Pid_y.OUTMAX=15.0;
	while(fabsf(blob_float_data[1]-100)>1 || fabsf(blob_float_data[0]-202)>1 ||fabsf(Angle_Pid.Err)>0.5)
	{
		Position_Pid_x.Actual=blob_float_data[1];
		Position_Pid_x.Target=100;
		PID_Update(&Position_Pid_x);
		
		Position_Pid_y.Actual=blob_float_data[0];
		Position_Pid_y.Target=202;
		PID_Update(&Position_Pid_y);
		
		Angle_Pid.Actual=global_angle_all;
		PID_Update(&Angle_Pid);
		
		if(fabsf(Position_Pid_x.Out)<1.0f)
		{
			if(Position_Pid_x.Out>0)
				Position_Pid_x.Out=1;
			else if(Position_Pid_x.Out<0)
				Position_Pid_x.Out=-1;
		}
		if(fabsf(Position_Pid_y.Out)<1.0f)
		{
			if(Position_Pid_y.Out>0)
				Position_Pid_y.Out=1;
			else if(Position_Pid_y.Out<0)
				Position_Pid_y.Out=-1;
		}
		Car_Run_Vx_Vy(-Position_Pid_x.Out,Position_Pid_y.Out,Angle_Pid.Out,0);
	}
		Car_Run_Vx_Vy(0,0,0,0);	HAL_Delay(20);
}
#define PI    3.1415926535
void move_global(float global_x,float global_y)
{
	float self_x=0.0f,self_y=0.0f;
	self_y=global_x*cos(global_angle*PI/180);
	self_x=global_x*sin(global_angle*PI/180);
	Car_Run_Vx_Vy(self_x,-self_y,60,0);
}