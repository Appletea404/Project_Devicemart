#include "ina219.h"

/**
 * @brief INA219 초기화 및 Calibration 설정
 */
HAL_StatusTypeDef INA219_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data[3];

    // Calibration 값 설정 (0.1 Ohm 션트 저항 기준, 약 2A 범위)
    data[0] = REG_CALIB;
    data[1] = 0x10; // MSB
    data[2] = 0x00; // LSB
    if (HAL_I2C_Master_Transmit(hi2c, INA219_ADDR, data, 3, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    // Configuration 설정 (32V 범위, +/-320mV 션트 범위, 12비트 해상도)
    data[0] = REG_CONFIG;
    data[1] = 0x39;
    data[2] = 0x9F;
    return HAL_I2C_Master_Transmit(hi2c, INA219_ADDR, data, 3, 100);
}

/**
 * @brief 버스 전압 읽기 (단위: V)
 */
float INA219_ReadBusVoltage(I2C_HandleTypeDef *hi2c) {
    uint8_t reg = REG_BUSV;
    uint8_t data[2];

    HAL_I2C_Master_Transmit(hi2c, INA219_ADDR, &reg, 1, 100);
    if (HAL_I2C_Master_Receive(hi2c, INA219_ADDR, data, 2, 100) == HAL_OK) {
        int16_t raw_v = (int16_t)((data[0] << 8) | data[1]);
        return (float)((raw_v >> 3) * 4) * 0.001f;
    }
    return -1.0f;
}

/**
 * @brief 전류 읽기 (단위: mA)
 */
float INA219_ReadCurrent(I2C_HandleTypeDef *hi2c) {
    uint8_t reg = REG_CURRENT;
    uint8_t data[2];

    HAL_I2C_Master_Transmit(hi2c, INA219_ADDR, &reg, 1, 100);
    if (HAL_I2C_Master_Receive(hi2c, INA219_ADDR, data, 2, 100) == HAL_OK) {
        int16_t raw_i = (int16_t)((data[0] << 8) | data[1]);
        return (float)raw_i * 0.1f; // Current_LSB = 0.1mA 가정
    }
    return -1.0f;
}
