#include "power_control.h"
#include "tim.h"  // htim1 및 __HAL_TIM_SET_COMPARE 사용을 위해 필요


// 실제 변수 정의는 소스 파일에서 한 번만 수행
//const float Kp_v = 0.05f;
//const float Ki_v = 0.5f;
//const float Kp_i = 0.113f;
//const float Ki_i = 180.8f;


const float Kp_v = 6.537434f;  // 시뮬레이션 결과: Voltage Loop Kp
const float Ki_v = 8.215182f;  // 시뮬레이션 결과: Voltage Loop Ki
const float Kp_i = 0.128181f;  // 시뮬레이션 결과: Current Loop Kp
const float Ki_i = 80.538710f; // 시뮬레이션 결과: Current Loop Ki

void PowerControl_Init(PowerControl_t *ctrl) {
    ctrl->target_v_bat = 4.20f;
    ctrl->limit_i_cc = 0.50f;
//    ctrl->mppt_i_ref = 0.20f;
    ctrl->mppt_i_ref = 0.050f;
    ctrl->v_integral = 0.0f;
    ctrl->i_integral = 0.0f;
    ctrl->duty_out = 0.0f;
    ctrl->prev_solar_power = 0.0f;
    ctrl->prev_solar_voltage = 0.0f;
}

void PowerControl_Run(PowerControl_t *ctrl, INA219_t *solar, INA219_t *batt) {


	// 필터 전의 생 데이터로 과전압을 먼저 체크 (매우 중요!)
	    if (batt->voltage_v > 4.25f) {
	        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	        ctrl->duty_out = 0.0f;
	        return;
	    }

	// [1. 필터 업데이트를 가장 먼저!]
	    // 그래야 모든 제어 로직이 '세척된' 데이터를 사용합니다.
	    Battery_Filter_Update(batt);

	    // [2. 통합된 스위칭 차단 조건]
	    // 패널 전압이 배터리보다 낮거나 너무 낮으면 즉시 정지합니다.
	    if (solar->voltage_v < batt->voltage_v + 0.7f || solar->voltage_v < 2.0f) {
	        ctrl->duty_out = 0.0f;
	        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

	        // PWM이 꺼지면 노이즈도 사라지므로,
	        // 필터가 0V를 향해 더 빠르게 수렴하도록 유도합니다.
//	        batt->voltage_v *= 0.8f;
//	        if (batt->voltage_v < 0.1f) batt->voltage_v = 0.0f;
	        return;
	    }





    // 1. MPPT (P&O) 알고리즘
    float current_power = solar->power_w;
    float delta_p = current_power - ctrl->prev_solar_power;
    float delta_v = solar->voltage_v - ctrl->prev_solar_voltage;

    // 전력 변화량이 아주 작을 때는 노이즈로 간주하여 무시 (0.001W)
    if (delta_p > 0.001f || delta_p < -0.001f) {
        if (delta_p > 0) {
            if (delta_v > 0) ctrl->mppt_i_ref -= 0.005f;
            else ctrl->mppt_i_ref += 0.005f;
        } else {
            if (delta_v > 0) ctrl->mppt_i_ref += 0.005f;
            else ctrl->mppt_i_ref -= 0.005f;
        }
    }

    // [추가] MPPT 전류 지령치 범위 제한 (최소 20mA ~ 최대 CC 제한값)
    if (ctrl->mppt_i_ref < 0.020f) ctrl->mppt_i_ref = 0.020f; // 최소값 20mA로 고정
    if (ctrl->mppt_i_ref > ctrl->limit_i_cc) ctrl->mppt_i_ref = ctrl->limit_i_cc;

    ctrl->prev_solar_power = current_power;
    ctrl->prev_solar_voltage = solar->voltage_v;

    // 2. 전압 루프 (Outer Loop)
    float v_err = ctrl->target_v_bat - batt->voltage_v;
    float v_integ_next = ctrl->v_integral + (v_err * CONTROL_DT);
    float I_cv = (Kp_v * v_err) + (Ki_v * v_integ_next);

    // 3. 민-셀렉터 (Min-Selector)
    float I_ref = ctrl->limit_i_cc;
    if (I_cv < I_ref) I_ref = I_cv;
    if (ctrl->mppt_i_ref < I_ref) I_ref = ctrl->mppt_i_ref;

    // 4. 안티 윈드업
    if (I_ref == I_cv) ctrl->v_integral = v_integ_next;

    // 5. 전류 루프 (Inner Loop)
    float i_err = I_ref - (batt->current_ma / 1000.0f);
    ctrl->i_integral += i_err * CONTROL_DT;

    // [추가 권장] 적분기 와인드업 방지 (Duty가 0~0.9 사이에서만 동작하도록)
    if (ctrl->i_integral > 0.9f / Ki_i) ctrl->i_integral = 0.9f / Ki_i;
    if (ctrl->i_integral < 0.0f) ctrl->i_integral = 0.0f;

    ctrl->duty_out = (Kp_i * i_err) + (Ki_i * ctrl->i_integral);

    // 6. 제한 및 PWM 출력 (ARR=999 기준)
    if (ctrl->duty_out > 0.9f) ctrl->duty_out = 0.9f;
    if (ctrl->duty_out < 0.0f) ctrl->duty_out = 0.0f;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(ctrl->duty_out * 1000));
}
