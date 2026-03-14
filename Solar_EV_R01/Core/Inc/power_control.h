

#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H

#include "ina219.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_DT_SEC               0.001f

#define CTRL_TARGET_V_BAT            3.80f
#define CTRL_I_CC_MAX                0.80f

#define CTRL_KP_I                    0.009f
#define CTRL_KI_I                    18.0f

#define CTRL_KP_V                    0.80f
#define CTRL_KI_V                    1.80f

#define CTRL_MPPT_ENABLE_TIME_SEC    0.05f
#define CTRL_MPPT_UPDATE_T_SEC       0.03f
#define CTRL_MPPT_STEP_A             0.006f

#define CTRL_MPPT_I_INIT             0.86f
#define CTRL_MPPT_I_MIN              0.02f
#define CTRL_MPPT_I_MAX              0.90f

#define CTRL_MPPT_FAST_DROP_A        0.020f
#define CTRL_MPPT_FALLBACK_MARGIN_A  0.05f
#define CTRL_MPPT_SAT_DUTY_TH        0.93f

#define CTRL_DUTY_MIN                0.02f
#define CTRL_DUTY_MAX                0.95f

#define CTRL_ICV_MIN                 0.0f
#define CTRL_ICV_MAX                 0.90f

#define CTRL_I_INT_MIN              -0.05f
#define CTRL_I_INT_MAX               0.05f

#define CTRL_V_INT_MIN              -0.08f
#define CTRL_V_INT_MAX               0.90f

#define CTRL_MPPT_DP_DEADBAND_W      0.01f

#define CTRL_SOLAR_MIN_V             2.0f
#define CTRL_SOLAR_BATT_MARGIN_V     0.20f
#define CTRL_BATT_OVP_V              3.90f

typedef enum
{
    POWER_MODE_STOP = 0,
    POWER_MODE_CC,
    POWER_MODE_CV,
    POWER_MODE_MPPT
} PowerMode_t;

typedef struct
{
    float target_v_bat;
    float limit_i_cc;

    float mppt_i_ref;
    float prev_solar_power;
    float mppt_dir;
    float elapsed_time_s;

    float v_integral;
    float i_integral;
    float duty_out;

    float i_ref_last;
    float i_cv_last;
    float i_meas_last;

    float solar_v_last;
    float solar_i_last;
    float batt_v_last;
    float batt_i_last;

    PowerMode_t mode;
} PowerControl_t;

void PowerControl_Init(PowerControl_t *ctrl);
void PowerControl_Run(PowerControl_t *ctrl, INA219_t *solar, INA219_t *batt);
const char *PowerControl_ModeString(PowerMode_t mode);

#ifdef __cplusplus
}
#endif

#endif


// 동원 코드
//#ifndef __POWER_CONTROL_H
//#define __POWER_CONTROL_H
//
//
//#include "ina219.h"
//
//// 제어 주기 (1ms)
//#define CONTROL_DT 0.001f
//
//typedef struct {
//    // 제어 목표 및 제한치
//    float target_v_bat;   // 배터리 목표 전압 (4.1V)
//    float limit_i_cc;     // 최대 충전 전류 (0.25A)
//    float mppt_i_ref;     // MPPT가 계산한 전류 제한치
//
//    // PI 제어 상태 (전압/전류)
//    float v_integral;
//    float i_integral;
//    float duty_out;
//
//    // MPPT 상태 변수
//    float prev_solar_power;
//    float prev_solar_voltage;
//} PowerControl_t;
//
//// 함수 프로토타입
//void PowerControl_Init(PowerControl_t *ctrl);
//void PowerControl_Run(PowerControl_t *ctrl, INA219_t *solar, INA219_t *batt);
//
//#endif
