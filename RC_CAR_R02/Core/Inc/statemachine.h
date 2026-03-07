/*
 * statemachine.h
 *
 *  Created on: Mar 1, 2026
 *      Author: appletea
 */

#ifndef INC_STATEMACHINE_H_
#define INC_STATEMACHINE_H_

#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "ledbar.h"
#include "speed.h"
#include "direction.h"
#include "car.h"
#include "ultrasonic.h"
#include "stdbool.h"


#define Block_Distance_Front 35
#define Block_Distance_Side 20
#define Crash_Distance 10
#define Stuck_Timeout 300



// 초기화: UART1 인터럽트 시작 등
void STMACHINE_Init(void);

void SHOW_UART2();

typedef enum
{
	AUTO_STATE_SCAN,
	AUTO_STATE_STOP,
	AUTO_STATE_AVOID,
	AUTO_STATE_BACK,
}AUTO_STATE;



void ST_FLAG(uint8_t cmd);
void ST_MACHINE();
void DC_CONTROL_AUTO();





#endif /* INC_STATEMACHINE_H_ */
