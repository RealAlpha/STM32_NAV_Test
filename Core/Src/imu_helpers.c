/*
 * imu_helpers.c
 *
 *  Created on: Aug 11, 2021
 *      Author: alpha-v
 */
#include "imu_helpers.h"


void PerformConfiguration(I2C_TypeDef *hi2c, uint16_t AccelRate, uit16_t GyroRate)
{
	uint8_t POWER_CTL_START = 0b00001000;
	uint8_t DATA_FORMAT = 0b00001000;
	uint8_t BW_RATE = 0b00001111;

	HAL_I2C_Mem_Write(&hi2c, ADXL345_ADDR, 0x31, 1, &DATA_FORMAT, 1, 1000);
	HAL_I2C_Mem_Write(&hi2c, ADXL345_ADDR, 0x2C, 1, &BW_RATE, 1, 1000);
	HAL_I2C_Mem_Write(&hi2c, ADXL345_ADDR, 0x2D, 1, &POWER_CTL_START, 1, 1000);
}

