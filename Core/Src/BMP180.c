/*
 * ==================================================
 *   BMP180.c - Library for BMP180 pressure sensor.
 *   Created by Matia Choi, 01/08/2024
 * ==================================================
 */

#include "BMP180.h"

#include "math.h"

uint8_t BMP180_Init(BMP180 *device, I2C_HandleTypeDef *hi2c) {
    device->hi2c = hi2c;
    device->temp_C = 0.0f;

    uint8_t errNum = 0;
    uint8_t regData;

    HAL_StatusTypeDef deviceIdStatus = BMP180_ReadReg(device, BMP180_REG_ID, &regData);
    device->device_id_status = deviceIdStatus;

    errNum += (device != HAL_OK);

    if (regData != BMP180_DEVICE_ID) return 255;

    // load calibratoin data
    errNum += (BMP180_Read_Calibration_Data(device, &(device->calibration_data)) != HAL_OK);

    return errNum;
}

HAL_StatusTypeDef BMP180_Read_Calibration_Data(BMP180 *device, CALIBRATION_DATA *calibration_data) {
    uint8_t rawCalibratoinData[22];
    HAL_StatusTypeDef status = BMP180_ReadRegs(device, BMP180_REG_CALIB_0, rawCalibratoinData, 22);

    int16_t *ptr = (int16_t *)calibration_data;

    // Pointer arithmetic to copy values into struct
    for (int i = 0; i < 11; i++) {
        *(ptr + i) = (rawCalibratoinData[2 * i] << 8) | rawCalibratoinData[2 * i + 1];
    }

    return status;
}

HAL_StatusTypeDef BMP180_SET_CTRL_MEAS(BMP180 *device, uint8_t ctrl_meas) {
    return BMP180_WriteReg(device, BMP180_REG_CTRL_MEAS, &ctrl_meas);
}

HAL_StatusTypeDef BMP180_ReadTemp(BMP180 *device) {
    uint16_t UT = 0x00;
    uint8_t tempMSB = 0x00;
    uint8_t tempLSB = 0x00;

    CALIBRATION_DATA cd = device->calibration_data;

    // Request for temp data
    BMP180_SET_CTRL_MEAS(device, CTRL_REG_TEMPERATURE);

    HAL_StatusTypeDef status = BMP180_ReadReg(device, BMP180_REG_OUT_MSB, &tempMSB);
    if (status != HAL_OK) return status;
    status = BMP180_ReadReg(device, BMP180_REG_OUT_LSB, &tempLSB);
    if (status != HAL_OK) return status;

    UT = (tempMSB << 8) | tempLSB;

    // device->raw_temp = UT;

    // Convert Raw data
    int32_t X1 = (UT - cd.AC6) * cd.AC5 / 0x8000;
    int32_t X2 = cd.MC * 0x800 / (X1 + cd.MD);
    int32_t B5 = X1 + X2;
    int32_t T = (B5 + 8) / 0x10;

    device->temp_C = (float)T / 10.0f;

    return status;
}

HAL_StatusTypeDef BMP180_ReadPressure(BMP180 *device, CONTROL_REGISTER control_register) {
    // add to parameter later

    // BMP180_Read_Calibration_Data(device, &(device->calibration_data));
    uint16_t UT;
    uint8_t XLSB;
    uint8_t LSB;
    uint8_t MSB;

    int32_t UP = 0x00;
    uint8_t tempMSB = 0x00;
    uint8_t tempLSB = 0x00;

    volatile int32_t B5, T, B6, X1, X2, X3, B3, B7, p;
    volatile uint32_t B4;

    int OSS = 0;

    CALIBRATION_DATA cd = device->calibration_data;
    HAL_StatusTypeDef status;

    /* cd.AC1=408;
    cd.AC2=-72;
    cd.AC3=-14383;
    cd.AC4=32741;
    cd.AC5=32757;
    cd.AC6=23153;
    cd.B1=6190;
    cd.B2=4;
    cd.MB=-32767;
    cd.MC=-8711;
    cd.MD=2868; */

    BMP180_SET_CTRL_MEAS(device, CTRL_REG_TEMPERATURE);
    HAL_Delay(5);

    status = BMP180_ReadReg(device, BMP180_REG_OUT_MSB, &tempMSB);
    if (status != HAL_OK) return status;
    status = BMP180_ReadReg(device, BMP180_REG_OUT_LSB, &tempLSB);
    if (status != HAL_OK) return status;

    UT = (tempMSB << 8) | tempLSB;

    BMP180_SET_CTRL_MEAS(device, control_register);
    if (control_register == CTRL_REG_PRESSURE_OSS_0) {
        OSS = 0;
        HAL_Delay(5);
    } else if (control_register == CTRL_REG_PRESSURE_OSS_1) {
        OSS = 1;
        HAL_Delay(8);
    } else if (control_register == CTRL_REG_PRESSURE_OSS_2) {
        OSS = 2;
        HAL_Delay(14);
    } else if (control_register == CTRL_REG_PRESSURE_OSS_3) {
        OSS = 3;
        HAL_Delay(26);
    }

    status = BMP180_ReadReg(device, BMP180_REG_OUT_MSB, &MSB);
    if (status != HAL_OK) return status;
    status = BMP180_ReadReg(device, BMP180_REG_OUT_LSB, &LSB);
    if (status != HAL_OK) return status;
    status = BMP180_ReadReg(device, BMP180_REG_OUT_XLSB, &XLSB);
    if (status != HAL_OK) return status;

    UP = ((MSB << 16) | (LSB << 8) | XLSB) >> (8 - OSS);

    device->raw_temperature = UT;
    device->raw_pressure = UP;

    // Convert TEMP Raw data
    X1 = (UT - (int32_t)cd.AC6) * ((int32_t)cd.AC5) >> 15;
    X2 = ((int32_t)cd.MC << 11) / (X1 + (int32_t)cd.MD);
    B5 = X1 + X2;
    T = (B5 + 8) >> 4;

    device->temp_C = (float)T / 10.0f;

    // start conversion pressure
    B6 = B5 - 4000;
    //B3 calculation
    X1 = ((int32_t)cd.B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = ((int32_t)cd.AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = (((((int32_t)cd.AC1) * 4 + X3) << OSS) + 2) >> 2;

    //B4 calculation
    X1 = ((int32_t)cd.AC3 * B6) >>13;
    X2 = ((int32_t)cd.B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;

    B4 = ((int32_t)cd.AC4 * (uint32_t)(X3 + 32768)) >> 15;
    B7 = ((uint32_t)UP - B3) * (int32_t)(50000 >> OSS);

    if (B7 < 0x80000000) {
        p = (B7 << 1) / B4;
    } else {
        p = (B7 / B4) << 1;
    }
    X1 = (p >> 8);
    X1 *= X1;
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p += (X1 + X2 + 3791) >> 4;

    device->pressure_Pa = p;
    device->altitude_M = 44330 * (1-pow((float)p/(float)PRESSURE_SEA_LEVEL_PA,0.19029495718));


    return status;
}

HAL_StatusTypeDef BMP180_ReadReg(BMP180 *device, uint8_t reg_addr, uint8_t *data) {
    // I2C-Handle, device-addr, loc-to-read-from, size-of-mem, output-ptr, how-many-bytes-to-read, time-out-delay
    return HAL_I2C_Mem_Read(device->hi2c, BMP180_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}
HAL_StatusTypeDef BMP180_ReadRegs(BMP180 *device, uint8_t reg_addr, uint8_t *data, uint8_t size) {
    return HAL_I2C_Mem_Read(device->hi2c, BMP180_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BMP180_WriteReg(BMP180 *device, uint8_t reg_addr, uint8_t *data) {
    return HAL_I2C_Mem_Write(device->hi2c, BMP180_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}