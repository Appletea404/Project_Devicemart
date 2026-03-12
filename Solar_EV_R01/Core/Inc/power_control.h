#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H


#include "ina219.h"

// 제어 주기 (1ms)
#define CONTROL_DT 0.001f

typedef struct {
    // 제어 목표 및 제한치
    float target_v_bat;   // 배터리 목표 전압 (4.1V)
    float limit_i_cc;     // 최대 충전 전류 (0.25A)
    float mppt_i_ref;     // MPPT가 계산한 전류 제한치

    // PI 제어 상태 (전압/전류)
    float v_integral;
    float i_integral;
    float duty_out;

    // MPPT 상태 변수
    float prev_solar_power;
    float prev_solar_voltage;
} PowerControl_t;

// 함수 프로토타입
void PowerControl_Init(PowerControl_t *ctrl);
void PowerControl_Run(PowerControl_t *ctrl, INA219_t *solar, INA219_t *batt);

#endif
