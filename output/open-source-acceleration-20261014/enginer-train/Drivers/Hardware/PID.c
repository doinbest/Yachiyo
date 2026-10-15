#include "PID.h"
#include "math.h"
void PID_Update(PID_t *p)
{
	p->Err=p->Target-p->Actual;
	if(p->KI!=0)
	{
		p->Jifen+=p->Err;
		if(p->Jifen>p->Jifen_MAX){p->Jifen=p->Jifen_MAX;}
		if(p->Jifen<p->Jifen_MIN){p->Jifen=p->Jifen_MIN;}
	}
	else
	{
		p->Jifen=0;
	}
	
	if(fabsf(p->Err)>5.0)
		p->Jifen=0;
	
	p->Out=p->KP*p->Err
					+p->KI*p->Jifen
					+p->KD*(p->Err-p->Err_last);
	if(p->Out>p->OUTMAX){p->Out=p->OUTMAX;}
	if(p->Out<p->OUTMIN){p->Out=p->OUTMIN;}
}

void PID_Clear(PID_t *p)
{
	p->Err=0;
	p->Jifen=0;
	p->Err_last=0;
}
