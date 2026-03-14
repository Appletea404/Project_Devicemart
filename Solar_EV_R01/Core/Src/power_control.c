

#include "power_control.h"
#include "tim.h"
#include <math.h>

static float clampf_local(float x, float xmin, float xmax)
{
    if (x < xmin) return xmin;
    if (x > xmax) return xmax;
    return x;
}

const char *PowerControl_ModeString(PowerMode_t mode)
{
    switch (mode)
    {
    case POWER_MODE_STOP: return "STOP";
    case POWER_MODE_CC:   return "CC";
    case POWER_MODE_CV:   return "CV";
    case POWER_MODE_MPPT: return "MPPT";
    default:              return "UNK";
    }
}

void PowerControl_Init(PowerControl_t *ctrl)
{
    ctrl->target_v_bat     = CTRL_TARGET_V_BAT;
    ctrl->limit_i_cc       = CTRL_I_CC_MAX;

    ctrl->mppt_i_ref       = CTRL_MPPT_I_INIT;
    ctrl->prev_solar_power = 0.0f;
    ctrl->mppt_dir         = +1.0f;
    ctrl->elapsed_time_s   = 0.0f;

    ctrl->v_integral       = 0.0f;
    ctrl->i_integral       = 0.0f;
    ctrl->duty_out         = 0.0f;

    ctrl->i_ref_last       = 0.0f;
    ctrl->i_cv_last        = 0.0f;
    ctrl->i_meas_last      = 0.0f;

    ctrl->solar_v_last     = 0.0f;
    ctrl->solar_i_last     = 0.0f;
    ctrl->batt_v_last      = 0.0f;
    ctrl->batt_i_last      = 0.0f;

    ctrl->mode             = POWER_MODE_STOP;
}

void PowerControl_Run(PowerControl_t *ctrl, INA219_t *solar, INA219_t *batt)
{
    float solar_v, solar_i_a, solar_p;
    float batt_v, batt_i_a;
    float v_err, i_err;
    float i_cv;
    float i_ref;
    float duty_unsat;
    float d_ff;
    float dP;
    static float mppt_timer_s = 0.0f;

    if (ctrl == NULL || solar == NULL || batt == NULL)
        return;

    ctrl->elapsed_time_s += CONTROL_DT_SEC;
    mppt_timer_s += CONTROL_DT_SEC;

    solar_v   = solar->voltage_v;
    solar_i_a = solar->current_ma / 1000.0f;
    solar_p   = solar->power_w;

    batt_v    = batt->voltage_v;
    batt_i_a  = batt->current_ma / 1000.0f;

    ctrl->solar_v_last = solar_v;
    ctrl->solar_i_last = solar_i_a;
    ctrl->batt_v_last  = batt_v;
    ctrl->batt_i_last  = batt_i_a;
    ctrl->i_meas_last  = batt_i_a;

    /* 보호 / 정지 조건 */
    if (batt_v >= CTRL_BATT_OVP_V)
    {
        ctrl->duty_out = 0.0f;
        ctrl->mode = POWER_MODE_STOP;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        return;
    }

    if (solar_v < CTRL_SOLAR_MIN_V || solar_v < (batt_v + CTRL_SOLAR_BATT_MARGIN_V))
    {
        ctrl->duty_out = 0.0f;
        ctrl->mode = POWER_MODE_STOP;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        return;
    }

    /* ============================
       1) CV outer loop
       ============================ */
    v_err = ctrl->target_v_bat - batt_v;

    {
        float v_int_next = ctrl->v_integral + v_err * CONTROL_DT_SEC;
        v_int_next = clampf_local(v_int_next, CTRL_V_INT_MIN, CTRL_V_INT_MAX);

        i_cv = (CTRL_KP_V * v_err) + (CTRL_KI_V * v_int_next);
        i_cv = clampf_local(i_cv, CTRL_ICV_MIN, CTRL_ICV_MAX);

        ctrl->i_cv_last = i_cv;

        /* integral은 나중에 CV 선택 시에만 반영 */
        if (fabsf(i_cv - fminf(ctrl->limit_i_cc, fminf(i_cv, ctrl->mppt_i_ref))) < 1e-6f)
        {
            ctrl->v_integral = v_int_next;
        }
    }

    /* ============================
       2) MPPT P&O
       ============================ */
    if ((ctrl->elapsed_time_s >= CTRL_MPPT_ENABLE_TIME_SEC) &&
        (mppt_timer_s >= CTRL_MPPT_UPDATE_T_SEC))
    {
        mppt_timer_s = 0.0f;

        dP = solar_p - ctrl->prev_solar_power;

        if (fabsf(dP) > CTRL_MPPT_DP_DEADBAND_W)
        {
            if (dP < 0.0f)
            {
                ctrl->mppt_dir = -ctrl->mppt_dir;
            }

            ctrl->mppt_i_ref += ctrl->mppt_dir * CTRL_MPPT_STEP_A;
            ctrl->mppt_i_ref = clampf_local(ctrl->mppt_i_ref, CTRL_MPPT_I_MIN, CTRL_MPPT_I_MAX);
        }

        ctrl->prev_solar_power = solar_p;
    }

    /* ============================
       3) final current reference
       ============================ */
    i_ref = ctrl->limit_i_cc;
    if (ctrl->i_cv_last < i_ref)      i_ref = ctrl->i_cv_last;
    if (ctrl->mppt_i_ref < i_ref)     i_ref = ctrl->mppt_i_ref;

    i_ref = clampf_local(i_ref, 0.0f, CTRL_ICV_MAX);
    ctrl->i_ref_last = i_ref;

    /* ============================
       4) current loop PI
       batt current ~= charging current
       ============================ */
    i_err = i_ref - batt_i_a;

    d_ff = (batt_v + 0.20f * batt_i_a) / fmaxf(solar_v, 0.5f);

    duty_unsat = d_ff + (CTRL_KP_I * i_err) + ctrl->i_integral;
    ctrl->duty_out = clampf_local(duty_unsat, CTRL_DUTY_MIN, CTRL_DUTY_MAX);

    /* anti-windup */
    if (!((ctrl->duty_out >= CTRL_DUTY_MAX && i_err > 0.0f) ||
          (ctrl->duty_out <= CTRL_DUTY_MIN && i_err < 0.0f)))
    {
        ctrl->i_integral += CTRL_KI_I * i_err * CONTROL_DT_SEC;
        ctrl->i_integral = clampf_local(ctrl->i_integral, CTRL_I_INT_MIN, CTRL_I_INT_MAX);
    }

    /* ============================
       5) MPPT fast fallback
       duty 포화 + measured current 부족 시
       MPPT ref를 빨리 낮춤
       ============================ */
    if ((ctrl->duty_out >= CTRL_MPPT_SAT_DUTY_TH) &&
        (batt_i_a < (i_ref - CTRL_MPPT_FALLBACK_MARGIN_A)))
    {
        ctrl->mppt_i_ref -= CTRL_MPPT_FAST_DROP_A;
        ctrl->mppt_i_ref = clampf_local(ctrl->mppt_i_ref, CTRL_MPPT_I_MIN, CTRL_MPPT_I_MAX);

        /* fallback 이후 final reference 재계산 */
        i_ref = ctrl->limit_i_cc;
        if (ctrl->i_cv_last < i_ref)  i_ref = ctrl->i_cv_last;
        if (ctrl->mppt_i_ref < i_ref) i_ref = ctrl->mppt_i_ref;
        i_ref = clampf_local(i_ref, 0.0f, CTRL_ICV_MAX);
        ctrl->i_ref_last = i_ref;
    }

    /* ============================
       6) mode decision
       ============================ */
    if (fabsf(ctrl->i_ref_last - ctrl->i_cv_last) < 1e-4f)
    {
        ctrl->mode = POWER_MODE_CV;
    }
    else if (fabsf(ctrl->i_ref_last - ctrl->mppt_i_ref) < 1e-4f)
    {
        ctrl->mode = POWER_MODE_MPPT;
    }
    else
    {
        ctrl->mode = POWER_MODE_CC;
    }

    /* PWM 출력 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(ctrl->duty_out * 1000.0f));
}



//#include "power_control.h"
//#include "tim.h"
//#include <math.h>
//
//static float clampf_local(float x, float xmin, float xmax)
//{
//    if (x < xmin) return xmin;
//    if (x > xmax) return xmax;
//    return x;
//}
//
//const char *PowerControl_ModeString(PowerMode_t mode)
//{
//    switch (mode)
//    {
//    case POWER_MODE_STOP: return "STOP";
//    case POWER_MODE_CC:   return "CC";
//    case POWER_MODE_CV:   return "CV";
//    case POWER_MODE_MPPT: return "MPPT";
//    default:              return "UNK";
//    }
//}
//
//void PowerControl_Init(PowerControl_t *ctrl)
//{
//    ctrl->target_v_bat     = CTRL_TARGET_V_BAT;
//    ctrl->limit_i_cc       = CTRL_I_CC_MAX;
//
//    ctrl->mppt_i_ref       = CTRL_MPPT_I_INIT;
//    ctrl->prev_solar_power = 0.0f;
//    ctrl->mppt_dir         = +1.0f;
//    ctrl->elapsed_time_s   = 0.0f;
//
//    ctrl->v_integral       = 0.0f;
//    ctrl->i_integral       = 0.0f;
//    ctrl->duty_out         = 0.0f;
//
//    ctrl->i_ref_last       = 0.0f;
//    ctrl->i_cv_last        = 0.0f;
//    ctrl->i_meas_last      = 0.0f;
//
//    ctrl->solar_v_last     = 0.0f;
//    ctrl->solar_i_last     = 0.0f;
//    ctrl->batt_v_last      = 0.0f;
//    ctrl->batt_i_last      = 0.0f;
//
//    ctrl->mode             = POWER_MODE_STOP;
//}
//
//void PowerControl_Run(PowerControl_t *ctrl, INA219_t *solar, INA219_t *batt)
//{
//    float solar_v, solar_i_a, solar_p;
//    float batt_v, batt_i_a;
//    float v_err, i_err;
//    float i_cv;
//    float i_ref;
//    float duty_unsat;
//    float d_ff;
//    float dP;
//    static float mppt_timer_s = 0.0f;
//
//    if (ctrl == NULL || solar == NULL || batt == NULL)
//        return;
//
//    ctrl->elapsed_time_s += CONTROL_DT_SEC;
//    mppt_timer_s += CONTROL_DT_SEC;
//
//    solar_v   = solar->voltage_v;
//    solar_i_a = solar->current_ma / 1000.0f;
//    solar_p   = solar->power_w;
//
//    batt_v    = batt->voltage_v;
//    batt_i_a  = batt->current_ma / 1000.0f;
//
//    ctrl->solar_v_last = solar_v;
//    ctrl->solar_i_last = solar_i_a;
//    ctrl->batt_v_last  = batt_v;
//    ctrl->batt_i_last  = batt_i_a;
//    ctrl->i_meas_last  = batt_i_a;
//
//    /* 보호 / 정지 조건 */
//    if (batt_v >= CTRL_BATT_OVP_V)
//    {
//        ctrl->duty_out = 0.0f;
//        ctrl->mode = POWER_MODE_STOP;
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//        return;
//    }
//
//    if (solar_v < CTRL_SOLAR_MIN_V || solar_v < (batt_v + CTRL_SOLAR_BATT_MARGIN_V))
//    {
//        ctrl->duty_out = 0.0f;
//        ctrl->mode = POWER_MODE_STOP;
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//        return;
//    }
//
//    /* ============================
//       1) CV outer loop
//       ============================ */
//    v_err = ctrl->target_v_bat - batt_v;
//
//    {
//        float v_int_next = ctrl->v_integral + v_err * CONTROL_DT_SEC;
//        v_int_next = clampf_local(v_int_next, CTRL_V_INT_MIN, CTRL_V_INT_MAX);
//
//        i_cv = (CTRL_KP_V * v_err) + (CTRL_KI_V * v_int_next);
//        i_cv = clampf_local(i_cv, CTRL_ICV_MIN, CTRL_ICV_MAX);
//
//        ctrl->i_cv_last = i_cv;
//
//        /* integral은 나중에 CV 선택 시에만 반영 */
//        if (fabsf(i_cv - fminf(ctrl->limit_i_cc, fminf(i_cv, ctrl->mppt_i_ref))) < 1e-6f)
//        {
//            ctrl->v_integral = v_int_next;
//        }
//    }
//
//    /* ============================
//       2) MPPT P&O
//       ============================ */
//    if ((ctrl->elapsed_time_s >= CTRL_MPPT_ENABLE_TIME_SEC) &&
//        (mppt_timer_s >= CTRL_MPPT_UPDATE_T_SEC))
//    {
//        mppt_timer_s = 0.0f;
//
//        dP = solar_p - ctrl->prev_solar_power;
//
//        if (fabsf(dP) > CTRL_MPPT_DP_DEADBAND_W)
//        {
//            if (dP < 0.0f)
//            {
//                ctrl->mppt_dir = -ctrl->mppt_dir;
//            }
//
//            ctrl->mppt_i_ref += ctrl->mppt_dir * CTRL_MPPT_STEP_A;
//            ctrl->mppt_i_ref = clampf_local(ctrl->mppt_i_ref, CTRL_MPPT_I_MIN, CTRL_MPPT_I_MAX);
//        }
//
//        ctrl->prev_solar_power = solar_p;
//    }
//
//    /* ============================
//       3) final current reference
//       ============================ */
//    i_ref = ctrl->limit_i_cc;
//    if (ctrl->i_cv_last < i_ref)      i_ref = ctrl->i_cv_last;
//    if (ctrl->mppt_i_ref < i_ref)     i_ref = ctrl->mppt_i_ref;
//
//    i_ref = clampf_local(i_ref, 0.0f, CTRL_ICV_MAX);
//    ctrl->i_ref_last = i_ref;
//
//    /* ============================
//       4) current loop PI
//       batt current ~= charging current
//       ============================ */
//    i_err = i_ref - batt_i_a;
//
//    d_ff = (batt_v + 0.20f * batt_i_a) / fmaxf(solar_v, 0.5f);
//
//    duty_unsat = d_ff + (CTRL_KP_I * i_err) + ctrl->i_integral;
//    ctrl->duty_out = clampf_local(duty_unsat, CTRL_DUTY_MIN, CTRL_DUTY_MAX);
//
//    /* anti-windup */
//    if (!((ctrl->duty_out >= CTRL_DUTY_MAX && i_err > 0.0f) ||
//          (ctrl->duty_out <= CTRL_DUTY_MIN && i_err < 0.0f)))
//    {
//        ctrl->i_integral += CTRL_KI_I * i_err * CONTROL_DT_SEC;
//        ctrl->i_integral = clampf_local(ctrl->i_integral, CTRL_I_INT_MIN, CTRL_I_INT_MAX);
//    }
//
//    /* ============================
//       5) MPPT fast fallback
//       duty 포화 + measured current 부족 시
//       MPPT ref를 빨리 낮춤
//       ============================ */
//    if ((ctrl->duty_out >= CTRL_MPPT_SAT_DUTY_TH) &&
//        (batt_i_a < (i_ref - CTRL_MPPT_FALLBACK_MARGIN_A)))
//    {
//        ctrl->mppt_i_ref -= CTRL_MPPT_FAST_DROP_A;
//        ctrl->mppt_i_ref = clampf_local(ctrl->mppt_i_ref, CTRL_MPPT_I_MIN, CTRL_MPPT_I_MAX);
//
//        /* fallback 이후 final reference 재계산 */
//        i_ref = ctrl->limit_i_cc;
//        if (ctrl->i_cv_last < i_ref)  i_ref = ctrl->i_cv_last;
//        if (ctrl->mppt_i_ref < i_ref) i_ref = ctrl->mppt_i_ref;
//        i_ref = clampf_local(i_ref, 0.0f, CTRL_ICV_MAX);
//        ctrl->i_ref_last = i_ref;
//    }
//
//    /* ============================
//       6) mode decision
//       ============================ */
//    if (fabsf(ctrl->i_ref_last - ctrl->i_cv_last) < 1e-4f)
//    {
//        ctrl->mode = POWER_MODE_CV;
//    }
//    else if (fabsf(ctrl->i_ref_last - ctrl->mppt_i_ref) < 1e-4f)
//    {
//        ctrl->mode = POWER_MODE_MPPT;
//    }
//    else
//    {
//        ctrl->mode = POWER_MODE_CC;
//    }
//
//    /* PWM 출력 */
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(ctrl->duty_out * 1000.0f));
//}


//#include "power_control.h"
//#include "tim.h"  // htim1 및 __HAL_TIM_SET_COMPARE 사용을 위해 필요
//
//
//// 실제 변수 정의는 소스 파일에서 한 번만 수행
////const float Kp_v = 0.05f;
////const float Ki_v = 0.5f;
////const float Kp_i = 0.113f;
////const float Ki_i = 180.8f;
//
//
//const float Kp_v = 6.537434f;  // 시뮬레이션 결과: Voltage Loop Kp
//const float Ki_v = 8.215182f;  // 시뮬레이션 결과: Voltage Loop Ki
//const float Kp_i = 0.128181f;  // 시뮬레이션 결과: Current Loop Kp
//const float Ki_i = 80.538710f; // 시뮬레이션 결과: Current Loop Ki
//
//void PowerControl_Init(PowerControl_t *ctrl) {
//    ctrl->target_v_bat = 3.8f;
//    ctrl->limit_i_cc = 0.80f;
////    ctrl->mppt_i_ref = 0.20f;
//    ctrl->mppt_i_ref = 0.050f;
//    ctrl->v_integral = 0.0f;
//    ctrl->i_integral = 0.0f;
//    ctrl->duty_out = 0.0f;
//    ctrl->prev_solar_power = 0.0f;
//    ctrl->prev_solar_voltage = 0.0f;
//}
//
//void PowerControl_Run(PowerControl_t *ctrl, INA219_t *solar, INA219_t *batt) {
//
//
//	// 필터 전의 생 데이터로 과전압을 먼저 체크 (매우 중요!)
//	    if (batt->voltage_v > 4.25f) {
//	        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//	        ctrl->duty_out = 0.0f;
//	        return;
//	    }
//
//	// [1. 필터 업데이트를 가장 먼저!]
//	    // 그래야 모든 제어 로직이 '세척된' 데이터를 사용합니다.
//	    Battery_Filter_Update(batt);
//
//	    // [2. 통합된 스위칭 차단 조건]
//	    // 패널 전압이 배터리보다 낮거나 너무 낮으면 즉시 정지합니다.
//	    if (solar->voltage_v < batt->voltage_v + 0.7f || solar->voltage_v < 2.0f) {
//	        ctrl->duty_out = 0.0f;
//	        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//
//	        // PWM이 꺼지면 노이즈도 사라지므로,
//	        // 필터가 0V를 향해 더 빠르게 수렴하도록 유도합니다.
////	        batt->voltage_v *= 0.8f;
////	        if (batt->voltage_v < 0.1f) batt->voltage_v = 0.0f;
//	        return;
//	    }
//
//
//
//
//
//    // 1. MPPT (P&O) 알고리즘
//    float current_power = solar->power_w;
//    float delta_p = current_power - ctrl->prev_solar_power;
//    float delta_v = solar->voltage_v - ctrl->prev_solar_voltage;
//
//    // 전력 변화량이 아주 작을 때는 노이즈로 간주하여 무시 (0.001W)
//    if (delta_p > 0.001f || delta_p < -0.001f) {
//        if (delta_p > 0) {
//            if (delta_v > 0) ctrl->mppt_i_ref -= 0.005f;
//            else ctrl->mppt_i_ref += 0.005f;
//        } else {
//            if (delta_v > 0) ctrl->mppt_i_ref += 0.005f;
//            else ctrl->mppt_i_ref -= 0.005f;
//        }
//    }
//
//    // [추가] MPPT 전류 지령치 범위 제한 (최소 20mA ~ 최대 CC 제한값)
//    if (ctrl->mppt_i_ref < 0.020f) ctrl->mppt_i_ref = 0.020f; // 최소값 20mA로 고정
//    if (ctrl->mppt_i_ref > ctrl->limit_i_cc) ctrl->mppt_i_ref = ctrl->limit_i_cc;
//
//    ctrl->prev_solar_power = current_power;
//    ctrl->prev_solar_voltage = solar->voltage_v;
//
//    // 2. 전압 루프 (Outer Loop)
//    float v_err = ctrl->target_v_bat - batt->voltage_v;
//    float v_integ_next = ctrl->v_integral + (v_err * CONTROL_DT);
//    float I_cv = (Kp_v * v_err) + (Ki_v * v_integ_next);
//
//    // 3. 민-셀렉터 (Min-Selector)
//    float I_ref = ctrl->limit_i_cc;
//    if (I_cv < I_ref) I_ref = I_cv;
//    if (ctrl->mppt_i_ref < I_ref) I_ref = ctrl->mppt_i_ref;
//
//    // 4. 안티 윈드업
//    if (I_ref == I_cv) ctrl->v_integral = v_integ_next;
//
//    // 5. 전류 루프 (Inner Loop)
//    float i_err = I_ref - (batt->current_ma / 1000.0f);
//    ctrl->i_integral += i_err * CONTROL_DT;
//
//    // [추가 권장] 적분기 와인드업 방지 (Duty가 0~0.9 사이에서만 동작하도록)
//    if (ctrl->i_integral > 0.9f / Ki_i) ctrl->i_integral = 0.9f / Ki_i;
//    if (ctrl->i_integral < 0.0f) ctrl->i_integral = 0.0f;
//
//    ctrl->duty_out = (Kp_i * i_err) + (Ki_i * ctrl->i_integral);
//
//    // 6. 제한 및 PWM 출력 (ARR=999 기준)
//    if (ctrl->duty_out > 0.9f) ctrl->duty_out = 0.9f;
//    if (ctrl->duty_out < 0.0f) ctrl->duty_out = 0.0f;
//
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(ctrl->duty_out * 1000));
//}
