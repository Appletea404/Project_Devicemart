/*
 * ledbar.h
 *
 *  Created on: Feb 27, 2026
 *      Author: lenovo
 */

#ifndef INC_LEDBAR_H_
#define INC_LEDBAR_H_






#include <delay.h>
#include "stm32f4xx_hal.h"
#include "stdbool.h"





#define SER_PORT_LED GPIOA
#define SER_PIN_LED GPIO_PIN_12


#define SRCLK_PORT_LED GPIOA
#define SRCLK_PIN_LED GPIO_PIN_11


#define RCLK_PORT_LED GPIOB
#define RCLK_PIN_LED GPIO_PIN_12







void dataOut_LED(uint8_t data);
void LED_BAR_UP_ing();
void LED_BAR_DOWN_ing();






















#endif /* INC_LEDBAR_H_ */
