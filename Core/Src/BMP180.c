/*
 * ==================================================
 *   BMP180.c - Library for BMP180 pressure sensor.
 *   Created by Matia Choi, 01/08/2024
 * ==================================================
 */

#include "BMP180.h"

uint8_t BMP180_Init(BMP180 *device, I2C_HandleTypeDef *hi2c, uint8_t ctrl_meas) {
    device->hi2c = hi2c;
    device->curr_ctrl_meas = ctrl_meas;
    device->temp_C = 0.0f;
    device->pressure_Bar = 0.0f;

    uint8_t errNum = 0;
    uint8_t regData;

    errNum += (BMP180_ReadReg(device, BMP180_REG_ID, &regData) != HAL_OK);

    if (regData != BMP180_DEVICE_ID) return 255;
    
    errNum += (BMP180_SET_CTRL_MEAS(device, device->curr_ctrl_meas) != HAL_OK);

    //load calibratoin data
    errNum += (BMP180_Read_Calibration_Data(device, &(device->calibration_data)) != HAL_OK);


    return errNum;
}

HAL_StatusTypeDef BMP180_Read_Calibration_Data(BMP180 *device, CALIBRATION_DATA *calibration_data){
    uint8_t rawCalibratoinData[22];
    HAL_StatusTypeDef status = BMP180_ReadRegs(device, BMP180_REG_CALIB_0, rawCalibratoinData, 22);

    int16_t* ptr = (int16_t*)calibration_data;
    
    //Pointer arithmetic to copy values into struct
    for (int i = 0; i < 11; i++) {
        *(ptr+i) = (rawCalibratoinData[2*i] << 8) | rawCalibratoinData[2*i+1];
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

    //00 1 0 0000
  
    

    BMP180_ReadReg(device, BMP180_REG_OUT_MSB, &tempMSB);
    BMP180_ReadReg(device, BMP180_REG_OUT_LSB, &tempLSB);

    UT = (tempMSB << 8) | tempLSB;



}

HAL_StatusTypeDef BMP180_ReadPressure(BMP180 *device) {}

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