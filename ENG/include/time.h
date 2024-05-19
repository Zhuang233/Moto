#ifndef __TIME_H__
#define __TIME_H__
#include "main.h"

#define SECOND				0		//��
#define MILLISECOND		1		//����
#define MICROSECOND		2		//΢��

typedef struct 
{
	uint32_t 	timer_cnt_now;
	uint32_t 	timer_cnt_last;
	float	 		timer_cnt_total;
	float			dt;
}TimeTD;

void TimeInit(TimeTD *time);
void GetDt(TimeTD *time, uint32_t time_unit);
#endif