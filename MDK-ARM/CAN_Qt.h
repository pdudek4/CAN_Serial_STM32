#ifndef _CAN_QT
#define _CAN_QT

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>


typedef struct{
	
	uint8_t prescalerCAN;
	bool pasmoCAN;
	
} CAN_param_t;


void CAN_filterConfig(uint32_t*);
void CANFilterActivate(uint32_t filterBank, bool active);
void CAN_Speed_Change(CAN_param_t* CAN_param);

#endif
