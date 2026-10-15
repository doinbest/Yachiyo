#ifndef __PID_H__
#define __PID_H__

typedef struct{
	float Target;
	float Actual;
	float Out;
	
	float KP;
	float KI;
	float KD;
	
	float OUTMAX;
	float OUTMIN;
	
	float Err;
	float Err_last;
	float Jifen;
	
	float Jifen_MAX;
	float Jifen_MIN;
	
}PID_t;


void PID_Clear(PID_t *p);
void PID_Update(PID_t *p);

#endif

