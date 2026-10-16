#ifndef __Motor_H
#define __Motor_H

#include <stdbool.h>
#include "stdio.h"	
#include "bjdj.h"
#include "my_uart.h"
#include <math.h>
#include "oled.h"
#include <main.h>

#define	M_PI 3.1415926  //��ֵ
#define lzjl 58.545264  //����ֱ��//ԭ������ֵ��58.470912
#define vel 240 //�ٶ�  220
#define acc 140  //���ٶ� 90
#define bs_jd 0.0015137  //�Ƕȵ�������ת��������ֵ��//ԭ������ֵ��0.0015137

// ����ϵ��������ֵ����ʵ���ٵ���
#define FRICTION_FACTOR  1.20   // ����������20%��
#define ERROR_MARGIN     1.15   // ���ԣ�ȣ�15%��
#define ACCELERATION_FACTOR 1.5 // ���ٶȲ���ϵ������ ����ʵ�����������

#define smp_mm_x 1.6264*0.4  //��ݮ������ֵת������26
#define smp_mm_y 0.7835*0.6  //��ݮ������ֵת������25
#define CW 0  //˳ʱ��
#define CWW 1 //��ʱ��




void Motor_pos(int fx,double jl,bool num,double jd_num);
void Motor_Stop(void);
void Motor_Delay(uint32_t steps);
int calc_time_for_steps(double steps);
void Motor_pos_s(int fx,double jl,double*pos_X,double*pos_Y,int key,double jd_num,double jd_xs);
void Motor_jd(double jd,bool n,double jd_num);
void Motor_jd_90(double*pos_X,double*pos_Y,bool n);
void Motor_jd_180(double*pos_X,double*pos_Y);
void Motor_jd_0(double*pos_X,double*pos_Y);
void Motor_jd_s(double*pos_X,double*pos_Y,double jd_num);
void Motor_smp(double*pos_X,double*pos_Y,double x,double y,int num,double jd_num);
void Motor_Delay_jd(double jd_num);

// ==================== 导航优化函数 ====================
void Motor_pos_diag(double dx, double dy, double*pos_X, double*pos_Y, int key, double jd_num, double jd_xs);
void Motor_jd_adaptive(double jd, bool n, double jd_num, int max_retries);
void Motor_jd_90_opt(double*pos_X, double*pos_Y, bool n);
void Motor_jd_180_opt(double*pos_X, double*pos_Y);
void Motor_jd_0_opt(double*pos_X, double*pos_Y);
#endif


