/*
 * imu_helpers.c
 *
 *  Created on: Aug 11, 2021
 *      Author: alpha-v
 */
#include "imu_helpers.h"


void PerformImuConfiguration(I2C_HandleTypeDef *hi2c, uint16_t AccelRate, uint16_t GyroRate)
{
	imuI2CHandle = hi2c;

	uint8_t POWER_CTL_STOP = 0;
	uint8_t POWER_CTL_START = 0b00001000;
	uint8_t DATA_FORMAT = 0b00001000;
	uint8_t BW_RATE = 0b00001111;
	// Enable the DATA_READY interrupt; it should already be mapped/routed to the INT1 pin by default, so no need to set the map
	uint8_t INT_ENABLE = 0b1 << 7;


	HAL_I2C_Mem_Write(imuI2CHandle, ADXL345_ADDR, 0x2D, 1, &POWER_CTL_STOP, 1, 1000);
	HAL_I2C_Mem_Write(imuI2CHandle, ADXL345_ADDR, 0x31, 1, &DATA_FORMAT, 1, 1000);
	//HAL_I2C_Mem_Write(imuI2CHandle, ADXL345_ADDR, 0x2C, 1, &BW_RATE, 1, 1000);
	HAL_I2C_Mem_Write(imuI2CHandle, ADXL345_ADDR, 0x2E, 1, &INT_ENABLE, 1, 1000);
	HAL_I2C_Mem_Write(imuI2CHandle, ADXL345_ADDR, 0x2D, 1, &POWER_CTL_START, 1, 1000);

	// Force-handle interrupt to avoid staying HIGH; TODO: only trigger when it was low?
	HandleAccelInterrupt();
	//HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)
}


void HandleAccelInterrupt()
{
	// TODO: Switch interrupts? Now we're assuming the only interrupt source is that data is available
	// TODO: Make hi2c configurable
	internalStateFlags |= ACCEL_AWAITING_I2C;
	HAL_I2C_Mem_Read_IT(imuI2CHandle, ADXL345_ADDR, 0x32, 1, accelDataBuffer, 6);


	// NOTE: Not handling conversion here to avoid wasting CPU cycles inside of an interrupt + not 100% sure if FPU can be used from interrupt
	//	  float ax_conv = (float)x * 4E-3;
	//	  float ay_conv = (float)y * 4E-3;
	//	  float az_conv = (float)z * 4E-3;
}


void HandleI2CInterrupt(I2C_HandleTypeDef *hi2c)
{
	// TODO: Fix simultaneous interrupt edge case - see header
	if (internalStateFlags & ACCEL_AWAITING_I2C)
	{
		//TODO: Implement some form of mutex/locking to ensure we are't currently reading?
		accelMeasRaw.x = ((int)accelDataBuffer[1] << 8) | accelDataBuffer[0];
		accelMeasRaw.y = ((int)accelDataBuffer[3] << 8) | accelDataBuffer[2];
		accelMeasRaw.z = ((int)accelDataBuffer[5] << 8) | accelDataBuffer[4];

		imuUpdateFlag |= ACCEL_AVAILABLE_FLAG;

		// Clear flag
		internalStateFlags &= ~ACCEL_AWAITING_I2C;
	}
}

vector3f GetAccelData()
{
	// TODO: Make this a local variable instead?
	// Convert the gyro measurements into g's assuming full scale measurement range // 4mg/LSB
	accelMeas.x = (float)accelMeasRaw.x * 4E-3;
	accelMeas.y = (float)accelMeasRaw.y * 4E-3;
	accelMeas.z = (float)accelMeasRaw.z * 4E-3;

	// Clear the accelerometer update available flag if it was set
	imuUpdateFlag &= ~ACCEL_AVAILABLE_FLAG;

	return accelMeas;
}
