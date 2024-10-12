#ifndef _DELAY_H
#define _DELAY_H
#include <sys.h>	  


typedef struct{
	uint32_t TMStart;
	uint32_t TMInter;
	
}TimeDelay_t;


void SetTime(TimeDelay_t *TimeType,uint32_t TimeInter);

uint8_t  CompareTime(TimeDelay_t *TimeType);



void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);
#endif

