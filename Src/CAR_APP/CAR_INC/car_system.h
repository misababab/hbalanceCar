#ifndef __CAR_SYSTEM_H
#define __CAR_SYSTEM_H

#include <stdint.h>
#include <stdio.h>
#include "delay.h"

void Task_State(void);

void  Led_Contrl(uint8_t cmd);
int Get_battery_volt(void) ;

#endif
