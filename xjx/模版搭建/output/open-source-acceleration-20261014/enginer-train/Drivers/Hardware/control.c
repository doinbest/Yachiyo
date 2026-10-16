#include "stm32f4xx.h"                  // Device header
#include "move.h"
#include "serial_task.h"
#include "math.h"
#include "fashion_star_uart_servo.h"
#include "emm_v5.h"
#include "pwm.h"
#include "serial.h"
#include "Delay.h"
#include "hwt101.h"
#include "timer.h"
extern Usart_DataTypeDef *servo_usart;
extern PID_t Angle_Pid;
#define red    		1
#define green			2
#define blue			3


#define grab_close  				80 //80
#define grab_open_little		55
#define grab_open_complete	20


#define zhuanpan_angle0			60
#define zhuanpan_angle1			92
#define zhuanpan_angle2			35

//#define yuntai_position_zhuanpan1		48
//#define yuntai_position_zhuanpan2		63.5

//等待转盘上物料停止
void wait_stop(void)
{
	//转盘开始是停的
	color=0;
	while(color==0);
	while(last_blob_x==0&&last_blob_y==0);
			if(color ==1)
				while(color==1);
			else if(color==2)
				while(color==2);
			else if(color==3)
				while(color==3);

	//如果转盘停了
	uint8_t stop_flag;
	while(stop_flag<=30)//50->30
	{
		if(last_blob_x==blob_float_data[0]&&last_blob_y==blob_float_data[1])
		{
			if(blob_float_data[0]!=0&&blob_float_data[1]!=0)
			{
				stop_flag++;
			}
		}
		HAL_Delay(1);
	}
}

void grab_material(uint8_t color,uint8_t count)
{
		if(count==1)
			zhuanpan_SetAngle(zhuanpan_angle1);
		else 
			zhuanpan_SetAngle(zhuanpan_angle2);
	
		//320,240
		//判断位置
		Claw_SetAngle(grab_open_complete);		
		delay_ms(100);
		if(color_all_pos[color-1][0]<160&&color_all_pos[color-1][1]<120)//左上的位置
		{
			Claw_SetAngle(grab_open_complete);		
			delay_ms(100);
			Emm_V5_Pos_Control(6,0,255,200,10000,1,0);
			delay_ms(20);
			Emm_V5_Pos_Control(5,1,250,230,4500,1,0);
			delay_ms(20);
			FSUS_SetServoAngleByInterval(servo_usart,0,-33,500,100,300,0,0);

			delay_ms(600);//680
			Claw_SetAngle(grab_close);
			delay_ms(380);  
			
			Emm_V5_Pos_Control(5,1,255,200,0,1,0);
			delay_ms(300);
			Emm_V5_Pos_Control(6,0,255,200,1750,1,0);	
			delay_ms(100);
		}
		else if(color_all_pos[color-1][0]>160&&color_all_pos[color-1][1]<120)//右上的位置
		{
			Claw_SetAngle(grab_open_complete);		
			delay_ms(100);
			Emm_V5_Pos_Control(6,0,255,230,10000,1,0);
			delay_ms(20);
			Emm_V5_Pos_Control(5,1,250,230,4500,1,0);
			delay_ms(20);
			FSUS_SetServoAngleByInterval(servo_usart,0,-7,500,100,300,0,0);

			delay_ms(600);
			Claw_SetAngle(grab_close);
			delay_ms(380);
			
			Emm_V5_Pos_Control(5,1,255,200,0,1,0);
			delay_ms(400);
			Emm_V5_Pos_Control(6,0,255,230,1725,1,0);	
			delay_ms(200); 
		}
		else if(color_all_pos[color-1][1]>120)//处于下方的位置
		{
			Claw_SetAngle(grab_open_complete);		
			delay_ms(100);
			Emm_V5_Pos_Control(6,0,255,200,1550,1,0);//1600->1550
			delay_ms(70);//20->70
			Emm_V5_Pos_Control(5,1,250,200,4500,1,0);
			delay_ms(20);
			FSUS_SetServoAngleByInterval(servo_usart,0,-17,500,100,300,0,0);

			delay_ms(900);//800->900
			Claw_SetAngle(grab_close);
			delay_ms(380);  
			
			Emm_V5_Pos_Control(5,1,255,200,0,1,0);
			delay_ms(300);
			Emm_V5_Pos_Control(6,0,255,200,1750,1,0);	
			delay_ms(100);
		}
		
		if(count==1)
		{
			FSUS_SetServoAngleByInterval(servo_usart,0,48,1100,500,500,0,0);
			delay_ms(900);
			Emm_V5_Pos_Control(5,1,250,200,3900,1,0);
			delay_ms(700);
			Claw_SetAngle(grab_open_little);	
			delay_ms(300);
			Emm_V5_Pos_Control(5,1,255,230,0,1,0);
			delay_ms(500);
			FSUS_SetServoAngleByInterval(servo_usart,0,-20,1200,500,500,0,0);
			Claw_SetAngle(grab_open_complete);	
			Emm_V5_Pos_Control(6,0,255,230,5900,1,0);	
			delay_ms(1250); 
		}
		else if(count==2)
		{
			FSUS_SetServoAngleByInterval(servo_usart,0,49,1100,500,500,0,0);
			delay_ms(900);
			Emm_V5_Pos_Control(5,1,250,200,4100,1,0);
			delay_ms(700);
			Claw_SetAngle(65);//grab_open_little->65
			delay_ms(300);
			Emm_V5_Pos_Control(5,1,255,230,0,1,0);
			delay_ms(600);
			FSUS_SetServoAngleByInterval(servo_usart,0,-20,1200,500,500,0,0);
			Claw_SetAngle(grab_open_complete);	
			Emm_V5_Pos_Control(6,0,255,230,5900,1,0);	
			delay_ms(1250); 
		}
		else if(count==3)
		{
			Emm_V5_Pos_Control(6,0,255,200,500,1,0);	
			HAL_Delay(300);
			FSUS_SetServoAngleByInterval(servo_usart,0,68.5,1200,650,500,0,0);
			delay_ms(1300);
			Emm_V5_Pos_Control(5,1,250,200,3700,1,0);
			delay_ms(700);
			Claw_SetAngle(65);//grab_open_little->65
			delay_ms(300);
			Emm_V5_Pos_Control(5,1,255,200,0,1,0);
			delay_ms(600);
//			FSUS_SetServoAngleByInterval(servo_usart,0,-20,1200,500,500,0,0);
			Claw_SetAngle(grab_open_complete);			
//			Emm_V5_Pos_Control(6,0,255,200,5700,1,0);	
//			delay_ms(700); 
		}
}
uint8_t red_area_temp=0,green_area_temp=0,blue_area_temp=0;
//按顺序抓取物料
void grab_material_by_qrcode(uint16_t num)
{	
	Serial_Printf(&huart3,"777");
	uint8_t color,num1,num2,num3;
	uint8_t count=1;
	num3=num%10;
	num2=num/10%10;
	num1=num/100;

	Claw_SetAngle(grab_open_little);
	zhuanpan_SetAngle(zhuanpan_angle1);
	FSUS_SetServoAngleByInterval(servo_usart,0,-20,1200,500,500,0,0);
	Emm_V5_Pos_Control(6,0,250,200,5900,1,0);
	delay_ms(10);
	Emm_V5_Pos_Control(5,1,250,200,0,1,0);
	delay_ms(10);
	Serial_Printf(&huart3,"777");

	while(1)
	{
		if(color_all_pos[0][0]!=0 && color_all_pos[0][1]!=0 && color_all_pos[1][0]!=0 && color_all_pos[1][1]!=0 && color_all_pos[2][0]!=0 && color_all_pos[2][1]!=0)
			break;
	}
	uint8_t stop_flag=0;
	//定位看全局视野
	while(count<=3)
	{
		data_reset();

		if(count==1)
		{
			color=num1;
			while(red_area==0 || green_area==0 || blue_area==0);

		}
		else if(count==2)
		{
			data_reset();
			stop_flag=0;
			while(stop_flag<=50)
			{
				if(color_all_pos[num2-1][0]==color_all_pos_last[num2-1][0])
				{
					if(color_all_pos[num2-1][1]==color_all_pos_last[num2-1][1])
					{
							stop_flag++;
					}
				}
				HAL_Delay(1);
			}
			if(num1==red)while(green_area==0||blue_area==0);
			else if(num1==green)while(red_area==0||blue_area==0);
			else if(num1==blue)while(red_area==0||green_area==0);
			if(num1==green)
			while(red_area==red_area_temp||blue_area==blue_area_temp);
			else if(num1==red)
			while(green_area==green_area_temp ||blue_area==blue_area_temp);
			else if(num1==blue)
				while(red_area==red_area_temp||green_area==green_area_temp);
						
			stop_flag=0;
				while(stop_flag<=50)
				{
					if(color_all_pos[num2-1][0]==color_all_pos_last[num2-1][0])
					{
						if(color_all_pos[num2-1][1]==color_all_pos_last[num2-1][1])
						{
								stop_flag++;
						}
					}
					HAL_Delay(1);
				}
			color=num2;		
		}

		else if(count==3)
		{				
			data_reset();
			stop_flag=0;
			while(stop_flag<=80)
			{
				if(color_all_pos[num3-1][0]==color_all_pos_last[num3-1][0])
				{
					if(color_all_pos[num3-1][1]==color_all_pos_last[num3-1][1])
					{
							stop_flag++;
					}
				}
				HAL_Delay(1);
			}
			if(num3==red)while(red_area==0);
			else if(num3==green)while(green_area==0);
			else if(num3==blue)while(blue_area==0);
			if(num3==red)
			while(red_area==red_area_temp ||red_area==0);
			else if(num3==green)
			while(green_area==green_area_temp ||green_area==0);
			else if(num3==blue)
				while(blue_area==blue_area_temp||blue_area==0);
						
			stop_flag=0;
				while(stop_flag<=80)
				{
					if(color_all_pos[num3-1][0]==color_all_pos_last[num3-1][0])
					{
						if(color_all_pos[num3-1][1]==color_all_pos_last[num3-1][1])
						{
								stop_flag++;
						}
					}
					HAL_Delay(1);
				}
			color=num3;		
		}
		red_area_temp=red_area;
		blue_area_temp=blue_area;
		green_area_temp=green_area;
		grab_material(color,count);
		count++;
	}
}





void set_material(uint8_t color,uint8_t count)
{
	if(count==1)
	{
		Claw_SetAngle(grab_open_complete);
		HAL_Delay(10);
		Emm_V5_Pos_Control(6,0,230,230,1600,1,0);
		delay_ms(20);
		Emm_V5_Pos_Control(5,1,200,230,6000,1,0);
		delay_ms(800);
	}
		data_reset();
		if(color==red)
			circle_dingwei(red);
		else if(color==green)
			circle_dingwei(green);
		else if(color==blue)
			circle_dingwei(blue);
		Claw_SetAngle(grab_open_little);
		delay_ms(100);		
		
		Emm_V5_Pos_Control(5,1,210,230,500,1,0);
		delay_ms(100);
		float angle;
		uint16_t delay_time;
		if(count==3)
		{
			delay_time=1300;
			angle=69.5;
		}
		else 
		{
			delay_time=1000;
			angle=48;
		}
		FSUS_SetServoAngleByInterval(servo_usart,0,angle,delay_time,500,500,0,0);
		delay_ms(delay_time+50);
		if(count==1)
		{			
			Emm_V5_Pos_Control(6,0,210,230,1600,1,0);			
			delay_ms(20);
			Emm_V5_Pos_Control(5,1,210,230,4000,1,0);
		}
		else if(count==2)
		{			
			Emm_V5_Pos_Control(6,0,210,230,1600,1,0);			
			delay_ms(20);
			Emm_V5_Pos_Control(5,1,210,230,4300,1,0);
		}
		else
		{		
			Emm_V5_Pos_Control(6,0,210,230,500,1,0);			
			delay_ms(20);
			Emm_V5_Pos_Control(5,1,210,230,3800,1,0);
		}
		delay_ms(500);
		Claw_SetAngle(grab_close);
		delay_ms(200);
		Emm_V5_Pos_Control(5,1,210,230,1800,1,0);
		delay_ms(150);
		FSUS_SetServoAngleByInterval(servo_usart,0,-20,delay_time,400,400,0,0);
		delay_ms(700);
		if(count==3)
		{
			Emm_V5_Pos_Control(6,0,210,200,1600,1,0);			
			delay_ms(300);
		}
		Emm_V5_Pos_Control(5,1,180,230,8800,1,0);
		delay_ms(1300);
		Claw_SetAngle(grab_open_complete);
		delay_ms(250);
		Emm_V5_Pos_Control(5,1,210,230,6000,1,0);
		delay_ms(250);
}
//按顺序放物料
uint16_t distance=880;
void set_material_by_qrcode(uint16_t num)
{
	uint8_t color,num1,num2,num3;
	uint8_t count=1;
	num3=num%10;
	num2=num/10%10;
	num1=num/100;
	
	while(count<=3)
	{
		if(count==1)
			color=num1;
		else if(count==2)
		{
			zhuanpan_SetAngle(zhuanpan_angle2);	
			if(num1==red&&num2==green)move(1,distance,150,15,4);
			else if(num1==red&&num2==blue)move(1,distance*2,150,15,4);
			else if(num1==green&&num2==blue)move(1,distance,150,15,4);
			else if(num1==green&&num2==red)move(0,distance,150,15,4);
			else if(num1==blue&&num2==green)move(0,distance,150,15,4);
			else if(num1==blue&&num2==red)move(0,distance*2,150,15,4);
			color=num2;
		}
		else if(count==3)
		{
			if(num2==red&&num3==green)move(1,distance,150,15,4);
			else if(num2==red&&num3==blue)move(1,distance*2,150,15,4);
			else if(num2==green&&num3==blue)move(1,distance,150,15,4);
			else if(num2==green&&num3==red)move(0,distance,150,15,4);
			else if(num2==blue&&num3==green)move(0,distance,150,15,4);
			else if(num2==blue&&num3==red)move(0,distance*2,150,15,4);
			color=num3;
		}
		set_material(color,count);
		count++;
	}
}
void set_material_top(uint8_t color,uint8_t count)
{
	if(count==1)
	{
		FSUS_SetServoAngleByInterval(servo_usart,0,48,1000,500,500,0,0);
		delay_ms(950);
		Emm_V5_Pos_Control(5,1,200,210,3600,1,0);
		delay_ms(720);
	}
	delay_ms(250);
	Claw_SetAngle(grab_close);
	delay_ms(250);
	Emm_V5_Pos_Control(5,1,200,230,1600,1,0);
	delay_ms(200);
	if(color==green)
	{
		FSUS_SetServoAngleByInterval(servo_usart,0,-21,1000,500,500,0,0);
		delay_ms(800);
		Emm_V5_Pos_Control(6,0,210,0,1600,1,0);

		Emm_V5_Pos_Control(5,1,200,230,5300,1,0);
		delay_ms(500);
		Claw_SetAngle(grab_open_little);
		delay_ms(300);
		Emm_V5_Pos_Control(5,1,200,250,500,1,0);
		delay_ms(400);
		if(count==1)
		{
			zhuanpan_SetAngle(zhuanpan_angle2);
			FSUS_SetServoAngleByInterval(servo_usart,0,48,1000,500,500,0,0);
			delay_ms(1000);
			Emm_V5_Pos_Control(5,1,200,210,4300,1,0);
			delay_ms(650);
		}
		else if(count==2)
		{
			FSUS_SetServoAngleByInterval(servo_usart,0,69.5,1100,500,500,0,0);
			Emm_V5_Pos_Control(6,0,210,0,500,1,0);
			delay_ms(1100);
			Emm_V5_Pos_Control(5,1,200,210,4000,1,0);
			Claw_SetAngle(grab_open_little-10);
			delay_ms(650);
		}
	}
	else if(color==red)
	{
		FSUS_SetServoAngleByInterval(servo_usart,0,14.5,800,400,400,0,0);
		Emm_V5_Pos_Control(6,0,200,230,4000,1,0);
		delay_ms(600);
		Emm_V5_Pos_Control(5,1,200,250,5300,1,0);
		delay_ms(450);
		Claw_SetAngle(grab_open_complete);
		delay_ms(300);
		Emm_V5_Pos_Control(6,0,200,230,1600,1,0);
		delay_ms(100);
		Emm_V5_Pos_Control(5,1,200,250,500,1,0);
		delay_ms(400);
		Claw_SetAngle(grab_open_little);
		if(count==1)
		{
			zhuanpan_SetAngle(zhuanpan_angle2);
			FSUS_SetServoAngleByInterval(servo_usart,0,49,800,400,400,0,0);
			delay_ms(800);
			Emm_V5_Pos_Control(5,1,200,210,4300,1,0);
			delay_ms(650);
		}
		else if(count==2)
		{
			FSUS_SetServoAngleByInterval(servo_usart,0,69.5,900,400,500,0,0);
			Emm_V5_Pos_Control(6,0,210,0,500,1,0);
			delay_ms(900);
			Emm_V5_Pos_Control(5,1,200,210,4000,1,0);
			Claw_SetAngle(grab_open_little-10);
			delay_ms(650);
		}
	}
	else if(color==blue)
	{
		Emm_V5_Pos_Control(5,1,200,250,0,1,0);
		delay_ms(200);
		FSUS_SetServoAngleByInterval(servo_usart,0,-53,1500,500,500,0,0);
		delay_ms(1000);
		Emm_V5_Pos_Control(6,0,230,0,4000,1,0);
		delay_ms(500);
		Emm_V5_Pos_Control(5,1,210,230,5300,1,0);
		delay_ms(850);
		Claw_SetAngle(grab_open_complete);
		delay_ms(300);
		Emm_V5_Pos_Control(6,0,200,230,1600,1,0);	
		delay_ms(100);		
		Emm_V5_Pos_Control(5,1,200,250,500,1,0);
		delay_ms(400);
		Claw_SetAngle(grab_open_little);
		if(count==1)
		{
			zhuanpan_SetAngle(zhuanpan_angle2);
			FSUS_SetServoAngleByInterval(servo_usart,0,48,1400,500,500,0,0);
			delay_ms(1400);
			Emm_V5_Pos_Control(5,1,200,210,4300,1,0);
			delay_ms(650);
		}
		else if(count==2)
		{
			FSUS_SetServoAngleByInterval(servo_usart,0,69.5,1500,500,500,0,0);
			Emm_V5_Pos_Control(6,0,210,0,500,1,0);
			delay_ms(1500);
			Emm_V5_Pos_Control(5,1,200,210,4000,1,0);
			Claw_SetAngle(grab_open_little-10);
			delay_ms(650);
		}
	}
		if(count==3)
		{
			FSUS_SetServoAngleByInterval(servo_usart,0,69.5,1500,500,500,0,0);
			delay_ms(400);
		}
}
//按顺序码垛
void set_material_top_by_qrcode(uint16_t num)
{
	uint8_t color,num1,num2,num3;
	uint8_t count=1;
	num3=num%10;
	num2=num/10%10;
	num1=num/100;
	Claw_SetAngle(grab_open_little);
	while(count<=3)
	{
		if(count==1)
			color=num1;
		else if(count==2)
		{
			zhuanpan_SetAngle(zhuanpan_angle2);	
			color=num2;
		}
		else if(count==3)
			color=num3;
		set_material_top(color,count);
		count++;
	}
}

void collect_material(uint8_t color,uint8_t count)
{
	if(count==1)
		zhuanpan_SetAngle(zhuanpan_angle1);
	else if(count==2)
		zhuanpan_SetAngle(zhuanpan_angle2);
 if(color==green)
	{
		Claw_SetAngle(grab_open_complete);
		Emm_V5_Pos_Control(5,1,200,230,8800,1,0);
		delay_ms(1150);
		Claw_SetAngle(grab_close);
		delay_ms(350);
	}
	else if(color==blue)
	{
		Claw_SetAngle(grab_open_complete);
		FSUS_SetServoAngleByInterval(servo_usart,0,-53,1000,400,500,0,0);
		delay_ms(450);
		Emm_V5_Pos_Control(5,1,200,230,8800,1,0);
		delay_ms(350);
		Emm_V5_Pos_Control(6,0,200,0,3900,1,0);
		delay_ms(650);
		Claw_SetAngle(grab_close);
		delay_ms(350);		
	}
	else if(color==red)
	{
		Claw_SetAngle(grab_open_complete);
		FSUS_SetServoAngleByInterval(servo_usart,0,14.5,1000,400,500,0,0);
		delay_ms(450);
		Emm_V5_Pos_Control(5,1,200,230,8800,1,0);
		delay_ms(450);
		Emm_V5_Pos_Control(6,0,200,230,4000,1,0);
		delay_ms(700);
		Claw_SetAngle(grab_close);
		delay_ms(350);		
	}
	uint16_t delay_time;
		if(color==green)
		{
			delay_time=1200;
		}
		else if(color==blue)
		{
			delay_time=1400;
		}
		else if(color==red)
		{
			delay_time=1050;
		}
		if(count!=3)
		{
			Emm_V5_Pos_Control(5,1,200,230,500,1,0);
			delay_ms(350);
		}
		else 
		{
			Emm_V5_Pos_Control(5,1,200,230,0,1,0);
			delay_ms(800);
		}
		if(count==1)
		{
			delay_ms(50);
			FSUS_SetServoAngleByInterval(servo_usart,0,48,delay_time,450,450,0,0);
		}
		else if(count==2)
			FSUS_SetServoAngleByInterval(servo_usart,0,48,delay_time,450,450,0,0);
		else 
			FSUS_SetServoAngleByInterval(servo_usart,0,68.5,delay_time,450,450,0,0);
		if(count!=3)
		{
				Emm_V5_Pos_Control(6,0,200,0,1600,1,0);
				delay_ms(180);
		}
		else if(count==3)
		{
				Emm_V5_Pos_Control(6,0,200,0,500,1,0);
				delay_ms(300);
		}
		delay_ms(delay_time-100);
		if(count==1)
		{
			Emm_V5_Pos_Control(5,1,230,230,3900,1,0);
			delay_ms(450);
		}
		else if(count==2)
		{
			Emm_V5_Pos_Control(5,1,230,230,4200,1,0);
			delay_ms(650);
		}
		else if(count==3)
		{
			Emm_V5_Pos_Control(5,1,230,230,3700,1,0);
			delay_ms(550);
		}
		if(count!=3)
		{
			Claw_SetAngle(grab_open_little);
			delay_ms(200);
			Emm_V5_Pos_Control(5,1,200,230,500,1,0);
			delay_ms(300);
			FSUS_SetServoAngleByInterval(servo_usart,0,-20,1100,500,500,0,0);
			delay_ms(600);
		}
		else if(count==3)
		{
			Claw_SetAngle(grab_open_little);
			delay_ms(200);
			Emm_V5_Pos_Control(5,1,200,230,0,1,0);
			delay_ms(50);	
			zhuanpan_SetAngle(zhuanpan_angle1);
		}

}
//按顺序收取物料
void collect_material_by_qrcode(uint16_t num)
{
	uint8_t color,num1,num2,num3;
	uint8_t count=1;
	num3=num%10;
	num2=num/10%10;
	num1=num/100;
	Emm_V5_Pos_Control(5,1,200,230,0,1,0);
	delay_ms(50);
	Emm_V5_Pos_Control(6,0,200,230,1600,1,0);
	delay_ms(100);
	if(num3==red)
			move(1,distance,150,15,4);
	else if(num3==blue)
			move(0,distance,150,15,4);
	else delay_ms(200);	
	material_color_dingwei(green);
	Claw_SetAngle(grab_open_little);
	while(count<=3)
	{
		if(count==1)color=num1;
		else if(count==2)color=num2;
		else if(count==3)color=num3;
		collect_material(color,count);
		count++;
	}
}



void competition(void);
float aim_angle=0.0f;
//处理串口屏上的按键
void check_screen(void)
{
	if(position_flag==1)
	{
//		data_reset();
//		Serial_Printf(&huart3,"888");
//		wait_stop();
//		material_dingwei();
//		delay_ms(100);
//		Emm_V5_Pos_Control(6,0,100,0,5800,1,0);
//		Serial_Printf(&huart3,"777");
//		collect_material_by_qrcode(132);
			home_dingwei();
			move(0,2230,150,10,10);
//		circle_dingwei(green);
		position_flag=0;
		
	}
	else if(upload_flag==1)
	{		
		Serial_Printf(&huart3,"888");
		wait_stop();
		material_dingwei();
//		grab_material_by_qrcode(213);
		upload_flag=0;
	}
	else if(run_circle_flag==1)
	{

		run_circle_flag=0;
	}
	else if(set_material_flag==1)
	{
		material_color_dingwei(green);
		set_material_top_by_qrcode(213);
//		set_material_by_qrcode(213);
//		collect_material_by_qrcode(213);
		set_material_flag=0;
	}
	
	else if(turn_left_flag==1)
	{
		aim_angle-=90.0f;
		turn_haf(aim_angle);
		turn_left_flag=0;
	}
	else if(turn_right_flag==1)
	{
		aim_angle+=90.0f;
		turn_haf(aim_angle);
		turn_right_flag=0;
	}
	else if(start_flag==1)
	{
		competition();
		start_flag=0;
	}
	else if(ahand_flag==1)
	{
		ahand_flag=0;
	}
	else if(turn_self_zero_flag==1)
	{
		HW101_adjust();
		turn_self_zero_flag=0;
	}
}

uint16_t test_sequence=123;
uint8_t move_dir_flag;
int16_t move_lost_distance;
uint8_t first_lost_flag;
void competition(void)
{
		//出门到二维码区
		move(1,2300,350,15,10);
		move(4,1050,160,10,5);
		Emm_V5_Pos_Control(6,0,230,200,3000,1,0);
		serial2_choose=1;
		//等待扫码
		Tim_enable(&htim6,1);
		while(qrcode_data[0]==0||qrcode_data[1]==0)
		{
			//预防没扫到码的情况
			if(timer_count>=3)
			{
				if(first_lost_flag==0)
				{
					move(1,400,15,5,5);
					move_dir_flag=1;
					first_lost_flag=1;
					move_lost_distance=-400;
				}
				else if(move_dir_flag==0 && first_lost_flag!=0)
				{
					move(1,800,15,5,5);
					move_dir_flag=1;
					move_lost_distance=-400;
				}
				else if(move_dir_flag==1)
				{
					move(0,800,15,5,5);
					move_dir_flag=0;
					move_lost_distance=400;
				}
			}
		}
		Tim_enable(&htim6,0);
		show_qrocde(qrcode_data[0],qrcode_data[1]);
		//移动到原料区		
		FSUS_SetServoAngleByInterval(servo_usart,0,-20,1200,500,400,0,0);
		move(1,4300+move_lost_distance,350,15,12);	
		move(2,700,210,10,10);//650->700
		Emm_V5_Pos_Control(6,0,250,200,1600,1,0);
		Claw_SetAngle(grab_open_complete);
		delay_ms(20);
		Emm_V5_Pos_Control(5,1,250,200,0,1,0);
		delay_ms(1300);
		data_reset();
		Serial_Printf(&huart3,"888");
		//等待转盘停，抓取原料
		wait_stop();
		material_dingwei();
		grab_material_by_qrcode(qrcode_data[0]);
		delay_ms(100);
		zhuanpan_SetAngle(zhuanpan_angle1);
		//移动到粗加工区
		move(5,300,210,10,10);
		move_py(0,2800,400,15,10,0,-90,90);
		turn_haf(-90);
		if(qrcode_data[0]/100==green)
		{
			Serial_Printf(&huart3,"222");
		}
		else if(qrcode_data[0]/100==red)
		{
			Serial_Printf(&huart3,"111");
		}
		else if(qrcode_data[0]/100==blue)
		{
			Serial_Printf(&huart3,"333");
		}
		move(0,8000,350,16,12);
		FSUS_SetServoAngleByInterval(servo_usart,0,-20,1500,500,400,0,0);
		turn_haf(-180);
		delay_ms(300);
		if(qrcode_data[0]/100==red)
		{			
			Serial_Printf(&huart3,"111");
			move(0,distance+600,300,12,10);
		}
		else if(qrcode_data[0]/100==green)
		{			
			Serial_Printf(&huart3,"222");
			move(0,600,300,12,10);
		}
		else if(qrcode_data[0]/100==blue)
		{			
			Serial_Printf(&huart3,"333");
			move(1,distance-600,300,12,10);
		}
		move(2,400,300,15,12);
		data_reset();
		//根据扫码顺序，在粗加工区粗定位
		if(qrcode_data[0]/100==green)
			circle_dingwei_cu(green);
		else if(qrcode_data[0]/100==blue)
			circle_dingwei_cu(blue);
		else if(qrcode_data[0]/100==red)
			circle_dingwei_cu(red);
		//放物料
		set_material_by_qrcode(qrcode_data[0]);
		//收物料
		collect_material_by_qrcode(qrcode_data[0]);
		zhuanpan_SetAngle(zhuanpan_angle0); //zhuanpan_angle1->zhuanpan_angle0

		//移动到暂存区
		move(3,400,300,15,12);//300->350
		move(0,2550,400,15,12);//400- 350
		zhuanpan_SetAngle(zhuanpan_angle1);//恢复之前的角度
		if(qrcode_data[0]/100==red)
		{
			Serial_Printf(&huart3,"111");
			move_py(0,6200+distance,400,8,10,0,-270,80);//15->8
		}
		else if(qrcode_data[0]/100==green)
		{
			Serial_Printf(&huart3,"222");
			move_py(0,6200,400,8,10,0,-270,80);
		}
		else if(qrcode_data[0]/100==blue)
		{
			Serial_Printf(&huart3,"333");
			move_py(0,6200-distance,400,8,10,0,-270,80);
		}
		Claw_SetAngle(grab_open_complete);
		FSUS_SetServoAngleByInterval(servo_usart,0,-20,1200,500,400,0,0);
		delay_ms(1200);
		//暂存区定位
		if(qrcode_data[0]/100==green)
			circle_dingwei_cu(green);
		else if(qrcode_data[0]/100==red)
			circle_dingwei_cu(red);
		else if(qrcode_data[0]/100==blue)
			circle_dingwei_cu(blue);
		//暂存区放环
		set_material_by_qrcode(qrcode_data[0]);
		Emm_V5_Pos_Control(5,1,210,230,0,1,0);
		delay_ms(300);
		zhuanpan_SetAngle(zhuanpan_angle1);
		FSUS_SetServoAngleByInterval(servo_usart,0,70,1200,500,400,0,0);
		zhuanpan_SetAngle(zhuanpan_angle1);
		Claw_SetAngle(grab_open_complete);
		//移动到原料区
		move(3,300,300,15,12);
		if(qrcode_data[0]%10==green)
			move(0,3000,400,15,12);
		else if(qrcode_data[0]%10==blue)
			move(0,3000+distance,400,15,12);
		else if(qrcode_data[0]%10==red)
			move(0,3000-distance,400,15,12);
		move_py(0,4000,400,15,10,0,-360,80);
		
		FSUS_SetServoAngleByInterval(servo_usart,0,-20,1200,500,400,0,0);
		Emm_V5_Pos_Control(6,0,250,200,1600,1,0);
		Claw_SetAngle(grab_open_complete);
		delay_ms(20);
		Emm_V5_Pos_Control(5,1,250,200,0,1,0);
		delay_ms(1200);
		data_reset();
		//第二圈
		Serial_Printf(&huart3,"888");
		wait_stop();
		material_dingwei();
		//抓取原料
		grab_material_by_qrcode(qrcode_data[1]);
		zhuanpan_SetAngle(zhuanpan_angle1);
		//移动到粗加工区
		move(5,300,210,10,10);
		move_py(0,2800,400,15,10,0,-450,90);
		turn_haf(-450);
		if(qrcode_data[1]/100==green)
		{
			Serial_Printf(&huart3,"222");
		}
		else if(qrcode_data[1]/100==red)
		{
			Serial_Printf(&huart3,"111");
		}
		else if(qrcode_data[1]/100==blue)
		{
			Serial_Printf(&huart3,"333");
		}
		move(0,8000,350,16,12);
		FSUS_SetServoAngleByInterval(servo_usart,0,-20,1500,500,400,0,0);
		turn_haf(-540);
		delay_ms(400);
		if(qrcode_data[1]/100==red)  //0->1
		{			
			Serial_Printf(&huart3,"111");
			move(0,distance+600,300,12,10);
		}
		else if(qrcode_data[1]/100==green) //0->1
		{			
			Serial_Printf(&huart3,"222");
			move(0,600,300,12,10);
		}
		else if(qrcode_data[1]/100==blue) //0->1
		{			
			Serial_Printf(&huart3,"333");
			move(1,distance-600,300,12,10);
		}
		move(2,400,300,15,12);
		data_reset();
		if(qrcode_data[1]/100==green)
			circle_dingwei_cu(green);
		else if(qrcode_data[1]/100==blue)
			circle_dingwei_cu(blue);
		else if(qrcode_data[1]/100==red)
			circle_dingwei_cu(red);
		//粗加工区放环
		set_material_by_qrcode(qrcode_data[1]);
		//粗加工区收回物料
		collect_material_by_qrcode(qrcode_data[1]);
		Emm_V5_Pos_Control(6,0,250,200,1600,1,0);
		zhuanpan_SetAngle(zhuanpan_angle0);
		//移动到暂存区
		move(3,400,300,15,12);
		move(0,2550,400,15,12);
		Serial_Printf(&huart3,"555");
		move_py(0,6200,400,10,10,0,-630,80); //15->10
		FSUS_SetServoAngleByInterval(servo_usart,0,-20,1200,500,400,0,0);
		delay_ms(1200);
		zhuanpan_SetAngle(zhuanpan_angle1);//恢复原来的角度
		//暂存区物料定位
		material_color_dingwei(green);
		//暂存区码垛
		set_material_top_by_qrcode(qrcode_data[1]);
		zhuanpan_SetAngle(zhuanpan_angle1);
		//回启停区
		move(3,500,300,15,12);
		move(0,3200,350,15,12);
		//加漂移
		move_py(0,3500,400,15,10,0,-720,80);
		move(0,6300,380,20,20);
		FSUS_SetServoAngleByInterval(servo_usart,0,70,1000,300,400,0,0);
		Emm_V5_Pos_Control(6,0,230,0,9000,1,0);
		delay_ms(300);
		home_dingwei();
		Emm_V5_Pos_Control(6,0,230,0,1500,1,0);
		move(0,2230,150,10,10);
}
