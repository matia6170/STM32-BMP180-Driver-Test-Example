/*
 * ==================================================
 *   BMP180.h - Library for BMP180 pressure sensor.
 *   Created by Matia Choi, 01/08/2024
 * ==================================================
 */
#ifndef BMP180_H
#define BMP180_H

#include "stm32f1xx_hal.h"  // for I2C

/* ===========
 *   DEFINES
 * ===========
 */
#define BMP180_I2C_ADDR 0xEE  // 0x77 << 1

#define BMP180_DEVICE_ID 0x55

/* =============
 *   REGISTERS
 * =============
 */
#define BMP180_REG_OUT_XLSB 0xF8
#define BMP180_REG_OUT_LSB 0xF7
#define BMP180_REG_OUT_MSB 0xF6
#define BMP180_REG_CTRL_MEAS 0xF4
#define BMP180_REG_SOFT_RESET 0xE0
#define BMP180_REG_ID 0xD0
#define BMP180_REG_CALIB_21 0xBF
#define BMP180_REG_CALIB_0 0xAA
/* =================
 *   Sensor struct
 * =================
 */
typedef struct {
    int16_t AC1;
    int16_t AC2;
    int16_t AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t B1;
    int16_t B2;
    int16_t MB;
    int16_t MC;
    int16_t MD;
} CALIBRATION_DATA;

typedef struct {
    // I2c handle
    I2C_HandleTypeDef *hi2c;

    uint8_t curr_ctrl_meas;

    // temperature data
    float temp_C;

    // pressure data
    float pressure_Bar;

    CALIBRATION_DATA calibration_data;

} BMP180;

/*
 * Initialization
 */

uint8_t BMP180_Init(BMP180 *device, I2C_HandleTypeDef *hi2c, uint8_t ctrl_meas);

HAL_StatusTypeDef BMP180_Read_Calibration_Data(BMP180 *device, CALIBRATION_DATA *calibration_data);

/*
 * SET Parameters
 */

// set the measurement contorl word
HAL_StatusTypeDef BMP180_SET_CTRL_MEAS(BMP180 *device, uint8_t ctrl_meas);

// set start of conversion
HAL_StatusTypeDef BMP180_SET_SCO(BMP180 *device, uint8_t sco_bit);
// set oversampling smoothness
// HAL_StatusTypeDef BMP180_SET_OSS(BMP180 *device, VAL_OSS oss_bits);

/*
 * GET DATA
 */

HAL_StatusTypeDef BMP180_ReadTemp(BMP180 *device);
HAL_StatusTypeDef BMP180_ReadPressure(BMP180 *device);

HAL_StatusTypeDef BMP180_ReadReg(BMP180 *device, uint8_t reg_addr, uint8_t *data);
HAL_StatusTypeDef BMP180_ReadRegs(BMP180 *device, uint8_t reg_addr, uint8_t *data, uint8_t size);
HAL_StatusTypeDef BMP180_WriteReg(BMP180 *device, uint8_t reg_addr, uint8_t *data);

#endif