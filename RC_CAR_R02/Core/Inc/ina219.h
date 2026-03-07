/*
 * ina219.h
 *
 *  Created on: Mar 5, 2026
 *      Author: appletea
 */

#ifndef INC_INA219_H_
#define INC_INA219_H_

#include "stm32f4xx_hal.h" // 사용 중인 MCU 시리즈에 맞게 수정

// INA219 I2C 기본 주소 (A0, A1 GND 연결 시)
#define INA219_ADDR (0x40 << 1)

// 레지스터 포인터 정의
#define REG_CONFIG  0x00
#define REG_SHUNTV  0x01
#define REG_BUSV    0x02
#define REG_POWER   0x03
#define REG_CURRENT 0x04
#define REG_CALIB   0x05

// 함수 선언
HAL_StatusTypeDef INA219_Init(I2C_HandleTypeDef *hi2c);
float INA219_ReadBusVoltage(I2C_HandleTypeDef *hi2c);
float INA219_ReadCurrent(I2C_HandleTypeDef *hi2c);

#endif


