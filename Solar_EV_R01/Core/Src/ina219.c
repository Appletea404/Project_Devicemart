#include "ina219.h"

/**
 * @brief INA219 초기화
 */
HAL_StatusTypeDef INA219_Init(INA219_t *sensor, I2C_HandleTypeDef *hi2c, uint16_t addr) {
    sensor->hi2c = hi2c;
    sensor->address = addr;
    uint8_t data[3];

    // 1. Calibration 설정 (0.1 Ohm 션트 기준)
    data[0] = REG_CALIB;
    data[1] = 0x10;
    data[2] = 0x00;
    if (HAL_I2C_Master_Transmit(sensor->hi2c, sensor->address << 1, data, 3, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    // 2. Configuration 설정 (128 Samples Averaging 적용)
    data[0] = REG_CONFIG;
    data[1] = 0x3F; // 32V 범위 , 128 samples
    data[2] = 0xFF; // 연속 측정 모드
    return HAL_I2C_Master_Transmit(sensor->hi2c, sensor->address << 1, data, 3, 100);
}






/* ina219.c */
#include "ina219.h"
#include "i2c.h"  // hi2c2, hi2c3 식별을 위해 반드시 필요

void INA219_Update(INA219_t *sensor) {
    uint8_t reg = REG_BUSV;
    uint8_t data[2];

    // 1. 버스 전압(Bus Voltage) 읽기
    if (HAL_I2C_Master_Transmit(sensor->hi2c, sensor->address << 1, &reg, 1, 100) == HAL_OK) {
        if (HAL_I2C_Master_Receive(sensor->hi2c, sensor->address << 1, data, 2, 100) == HAL_OK) {
            int16_t raw_v = (int16_t)((data[0] << 8) | data[1]);
            sensor->voltage_raw = (float)((raw_v >> 3) * 4) * 0.001f;

            // --- 여기가 핵심 해결 구간입니다 ---
            if (sensor->hi2c == &hi2c2) {
                // 태양광 센서(I2C2): 필터를 안 쓰므로 즉시 대입
                sensor->voltage_v = sensor->voltage_raw;
            }
            // 배터리 센서(I2C3): 여기선 v를 건드리지 않음.
            // 나중에 Battery_Filter_Update가 raw를 가져가서 v를 예쁘게 깎아줄 것임.
        }
    }

    // 2. 전류(Current) 읽기
    reg = REG_CURRENT;
    if (HAL_I2C_Master_Transmit(sensor->hi2c, sensor->address << 1, &reg, 1, 100) == HAL_OK) {
        if (HAL_I2C_Master_Receive(sensor->hi2c, sensor->address << 1, data, 2, 100) == HAL_OK) {
            int16_t raw_i = (int16_t)((data[0] << 8) | data[1]);
            sensor->current_ma = (float)raw_i * 0.1f;
        }
    }

    // 3. 전력 계산 (MPPT용)
    sensor->power_w = sensor->voltage_v * (sensor->current_ma / 1000.0f);
}








///**
// * @brief 전압 및 전류 원본 데이터 읽기
// */
//void INA219_Update(INA219_t *sensor) {
//    uint8_t reg;
//    uint8_t data[2];
//
//    // 1. 버스 전압(Bus Voltage) 읽기 -> voltage_raw에 저장
//    reg = REG_BUSV;
//    if (HAL_I2C_Master_Transmit(sensor->hi2c, sensor->address << 1, &reg, 1, 100) == HAL_OK) {
//        if (HAL_I2C_Master_Receive(sensor->hi2c, sensor->address << 1, data, 2, 100) == HAL_OK) {
//            int16_t raw_v = (int16_t)((data[0] << 8) | data[1]);
//            // 필터 전 원본 데이터 저장
//            sensor->voltage_raw = (float)((raw_v >> 3) * 4) * 0.001f;
//
//            // [수정] 이 센서가 만약 솔라 센서라면(address 확인 등으로 분기 가능)
//                        // 배터리 센서가 아닐 때만 v에 직접 대입합니다.
//                        if (sensor->address == (0x40 << 1) && sensor->hi2c == &hi2c2) {
//                            sensor->voltage_v = sensor->voltage_raw;
//                        }
//        }
//    }
//
//    // 2. 전류(Current) 읽기 -> current_ma에 저장 (필터 전 임시 저장)
//    reg = REG_CURRENT;
//    if (HAL_I2C_Master_Transmit(sensor->hi2c, sensor->address << 1, &reg, 1, 100) == HAL_OK) {
//        if (HAL_I2C_Master_Receive(sensor->hi2c, sensor->address << 1, data, 2, 100) == HAL_OK) {
//            int16_t raw_i = (int16_t)((data[0] << 8) | data[1]);
//            sensor->current_ma = (float)raw_i * 0.1f;
//        }
//    }
//
//    // [중요] 전력 계산식이 있어야 MPPT가 작동함
//        sensor->power_w = sensor->voltage_v * (sensor->current_ma / 1000.0f);
//}

/* 필터 상태 유지 변수 */
static float batt_v_filtered = 0.0f;
static float batt_i_filtered = 0.0f;

/**
 * @brief 배터리 전압 및 전류 필터 업데이트
 */
void Battery_Filter_Update(INA219_t *batt_sensor) {
	const float alpha = 0.005f;

	    // [추가] 물리적으로 불가능한 전압(노이즈)은 아예 무시합니다.
	    // 2.5V 미만은 리튬 배터리에서 나올 수 없는 전압이므로 업데이트 안 함.
	    if (batt_sensor->voltage_raw < 2.5f) {
	        // 이전의 안정적인 필터값을 그대로 유지시켜 제어 루프를 보호함.
	        batt_sensor->voltage_v = batt_v_filtered;
	        return;
	    }

	    if (batt_v_filtered < 0.1f) {
	        batt_v_filtered = batt_sensor->voltage_raw;
	        batt_i_filtered = batt_sensor->current_ma;
	    }

	    batt_v_filtered = (alpha * batt_sensor->voltage_raw) + ((1.0f - alpha) * batt_v_filtered);
	    batt_i_filtered = (alpha * batt_sensor->current_ma) + ((1.0f - alpha) * batt_i_filtered);

	    batt_sensor->voltage_v = batt_v_filtered;
	    batt_sensor->current_ma = batt_i_filtered;
    // 4. 전력(Power) 계산: 필터된 값들을 기준으로 계산
    batt_sensor->power_w = batt_sensor->voltage_v * (batt_sensor->current_ma / 1000.0f);
}




///* Global 또는 Static 변수로 필터값 저장 */
//static float batt_v_filtered = 0.0f;
//static float batt_i_filtered = 0.0f;



//void Battery_Filter_Update(INA219_t *batt_sensor) {
//    // 1. 가중치 alpha를 더 낮게 설정 (0.005f는 약 200회 평균 효과)
//    const float alpha = 0.005f;
//    const float max_diff = 0.5f; // 0.5V 이상 튀는 값은 노이즈로 간주
//
//    if (batt_v_filtered < 0.1f) {
//        batt_v_filtered = batt_sensor->voltage_v;
//        batt_i_filtered = batt_sensor->current_ma;
//        return;
//    }
//
//    // 2. 이상치 방어 로직 (이전 필터값과 너무 차이가 크면 업데이트 스킵)
//    // 단, 0.0f가 아닐 때만 작동
//    if (batt_v_filtered > 0.1f) {
//        float diff = batt_sensor->voltage_v - batt_v_filtered;
//        if (diff > max_diff || diff < -max_diff) {
//            // 너무 튀는 값은 이번 샘플에서 무시하고 이전 필터값을 유지
//            batt_sensor->voltage_v = batt_v_filtered;
//            batt_sensor->current_ma = batt_i_filtered;
//            return;
//        }
//    }
//
//    // 3. 지수 이동 평균(EMA) 필터 적용
//    batt_v_filtered = (alpha * batt_sensor->voltage_v) + ((1.0f - alpha) * batt_v_filtered);
//    batt_i_filtered = (alpha * batt_sensor->current_ma) + ((1.0f - alpha) * batt_i_filtered);
//
//    // 필터링된 값을 구조체에 반영
//    batt_sensor->voltage_v = batt_v_filtered;
//    batt_sensor->current_ma = batt_i_filtered;
//}

