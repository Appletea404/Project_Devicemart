

#ifndef __INA219_H
#define __INA219_H

#include "main.h"
#include "i2c.h"
#include <stdint.h>

#define INA219_ADDR_7BIT   0x40

#define INA219_REG_CONFIG   0x00
#define INA219_REG_SHUNTV   0x01
#define INA219_REG_BUSV     0x02
#define INA219_REG_POWER    0x03
#define INA219_REG_CURRENT  0x04
#define INA219_REG_CALIB    0x05

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint16_t address_7bit;

    float voltage_raw;      // latest raw bus voltage [V]
    float voltage_v;        // filtered / final voltage [V]

    float current_raw_ma;   // latest raw current [mA]
    float current_ma;       // filtered / final current [mA]

    float power_w;          // voltage_v * current_ma

    uint8_t is_initialized;
    uint8_t filter_initialized;

    float filt_v;
    float filt_i;
} INA219_t;

HAL_StatusTypeDef INA219_Init(INA219_t *sensor, I2C_HandleTypeDef *hi2c, uint16_t addr_7bit);
HAL_StatusTypeDef INA219_Update(INA219_t *sensor);
void INA219_BatteryFilterUpdate(INA219_t *batt_sensor);

#endif




// 동원 코드
//#ifndef __INA219_H
//#define __INA219_H
//
//#include "main.h"
//#include "i2c.h"
//
//// INA219 I2C 주소 설정
//#define INA219_ADDR (0x40 << 1)
//
//// 레지스터 주소 정의
//#define REG_CONFIG  0x00
//#define REG_BUSV    0x02
//#define REG_CURRENT 0x04
//#define REG_CALIB   0x05
//
//// 센서별 독립 데이터를 관리하는 구조체
//typedef struct {
//    I2C_HandleTypeDef *hi2c; // hi2c2 또는 hi2c3 핸들 저장
//    uint16_t address;		 // 센서 고유 주소
//    float voltage_v;         // 전압 (V)
//    float voltage_raw;		 // 필터가 읽는 전압
//    float current_ma;        // 전류 (mA)
//    float power_w;           // 전력 (W)
//} INA219_t;
//
//// 함수 선언
//HAL_StatusTypeDef INA219_Init(INA219_t *sensor, I2C_HandleTypeDef *hi2c, uint16_t addr);
//void INA219_Update(INA219_t *sensor);
//void Battery_Filter_Update(INA219_t *batt_sensor);
//
//
//
//#endif






///*
// * ina219.h
// *
// *  Created on: Mar 5, 2026
// *      Author: appletea
// */
//
//#ifndef INC_INA219_H_
//#define INC_INA219_H_
//
//#include "stm32f4xx_hal.h" // 사용 중인 MCU 시리즈에 맞게 수정
//
//// INA219 I2C 기본 주소 (A0, A1 GND 연결 시)
//#define INA219_ADDR (0x40 << 1)
//
//// 레지스터 포인터 정의
//#define REG_CONFIG  0x00
//#define REG_SHUNTV  0x01
//#define REG_BUSV    0x02
//#define REG_POWER   0x03
//#define REG_CURRENT 0x04
//#define REG_CALIB   0x05
//
//// 함수 선언
//HAL_StatusTypeDef INA219_Init(I2C_HandleTypeDef *hi2c);
//float INA219_ReadBusVoltage(I2C_HandleTypeDef *hi2c);
//float INA219_ReadCurrent(I2C_HandleTypeDef *hi2c);
//
//#endif
//
//
