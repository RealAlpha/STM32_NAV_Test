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

	// Initialize the accelerometer
	uint8_t POWER_CTL_STOP = 0;
	uint8_t POWER_CTL_START = 0b00001000;
	uint8_t DATA_FORMAT = 0b00001000;
	uint8_t BW_RATE = 0b00001111;
	// Enable the DATA_READY interrupt; it should already be mapped/routed to the INT1 pin by default, so no need to set the map
	uint8_t INT_ENABLE = 0b1 << 7;


	HAL_I2C_Mem_Write(imuI2CHandle, ADXL345_ADDR, 0x2D, 1, &POWER_CTL_STOP, 1, 1000);
	HAL_I2C_Mem_Write(imuI2CHandle, ADXL345_ADDR, 0x31, 1, &DATA_FORMAT, 1, 1000);
	HAL_I2C_Mem_Write(imuI2CHandle, ADXL345_ADDR, 0x2E, 1, &INT_ENABLE, 1, 1000);
	HAL_I2C_Mem_Write(imuI2CHandle, ADXL345_ADDR, 0x2C, 1, &BW_RATE, 1, 1000);
	HAL_I2C_Mem_Write(imuI2CHandle, ADXL345_ADDR, 0x2D, 1, &POWER_CTL_START, 1, 1000);


	HAL_Delay(1);
	// Initialize the gyro
	uint8_t GyroDLPF = 0b00011000;
	uint8_t GyroSMPLRT_DIV = 4; // (8*10^3)/1600 - 1; opted for 1600 since 3200 did not produce a whole/integer divisor
	// NOTE: Choosing 50us pulse to avoid issue similar to ADXL345 where it can get "stuck" on subsequent runs
	uint8_t GyroInterruptConfig = 0b00010001;uint8_t GyroStatus[0x3F];
	HAL_StatusTypeDef GyroStatusResult = HAL_I2C_Mem_Read(imuI2CHandle, GYRO_ADDR, 0x00, 1, &GyroStatus, 0x3F, 1000);
	HAL_StatusTypeDef GyroSampleRateResult = HAL_I2C_Mem_Write(imuI2CHandle, GYRO_ADDR, 0x15, 1, &GyroSMPLRT_DIV, 1, 1000);
	HAL_StatusTypeDef GyroInterruptResult = HAL_I2C_Mem_Write(imuI2CHandle, GYRO_ADDR, 0x17, 1, &GyroInterruptConfig, 1, 1000);
	HAL_StatusTypeDef GyroEnableResult = HAL_I2C_Mem_Write(imuI2CHandle, GYRO_ADDR, 0x16, 1, &GyroDLPF, 1, 1000);



	// Force-handle interrupt to avoid staying HIGH; TODO: only trigger when it was low?
	HandleAccelInterrupt();
	//HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)
}


void HandleAccelInterrupt()
{
	// Ensure we don't start an I2C request while still waiting for another one to complete!
	if (!(internalStateFlags & GYRO_AWAITING_I2C))
	{
		internalStateFlags |= ACCEL_AWAITING_I2C;
		HAL_StatusTypeDef Status = HAL_I2C_Mem_Read_IT(imuI2CHandle, ADXL345_ADDR, 0x32, 1, accelDataBuffer, 6);
	}
	else
	{
		internalStateFlags |= ACCEL_NEED_I2C;
	}
}

void HandleGyroInterrupt()
{
	// Ensure we don't start an I2C request while still waiting for another one to complete!
	if (!(internalStateFlags & ACCEL_AWAITING_I2C))
	{
		internalStateFlags |= GYRO_AWAITING_I2C;
		HAL_StatusTypeDef Status = HAL_I2C_Mem_Read_IT(imuI2CHandle, GYRO_ADDR, 0x1D, 1, gyroDataBuffer, 6);
	}
	else
	{
		internalStateFlags |= GYRO_NEED_I2C;
	}
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
	else if (internalStateFlags & GYRO_AWAITING_I2C)
	{
		//TODO: Implement some form of mutex/locking to ensure we are't currently reading?
		gyroRatesRaw.x = (((int)gyroDataBuffer[0] << 8) | gyroDataBuffer[1]);
		gyroRatesRaw.y = (((int)gyroDataBuffer[2] << 8) | gyroDataBuffer[3]);
		gyroRatesRaw.z = (((int)gyroDataBuffer[4] << 8) | gyroDataBuffer[5]);

		// Make it known that an update is available
		imuUpdateFlag |= GYRO_AVAILABLE_FLAG;

		// Clear flag
		internalStateFlags &= ~GYRO_AWAITING_I2C;
	}

	// Allow pending actions from the gyro/accelerometer to be run (useful when data available overlaps with an existingn transfer)
	// TODO: Currently calls interrupt function sincne all it really does is start the I2C / settingn the flag twice won't do any harm - but this might channge in the future!
	if (internalStateFlags & ACCEL_NEED_I2C)
	{
		internalStateFlags &= ~ACCEL_NEED_I2C;
		HandleAccelInterrupt();
	}
	else if (internalStateFlags & GYRO_NEED_I2C)
	{
		internalStateFlags &= ~GYRO_NEED_I2C;
		HandleGyroInterrupt();
	}
}

vector3f GetAccelData()
{
	// TODO: Make this a local variable instead?
	// Convert the gyro measurements into g's assuming full scale measurement range // 4mg/LSB
	// TOOD: Investigate/fix slow operation caused by these bias operations (possibly still not float literals, despite the f...but even that wouldn't explain it?)
	accelMeas.x = (float)accelMeasRaw.x * 4E-3;// - ACCEL_BIAS_X;
	accelMeas.y = (float)accelMeasRaw.y * 4E-3;// - ACCEL_BIAS_Y;
	accelMeas.z = (float)accelMeasRaw.z * 4E-3;// - ACCEL_BIAS_Z;

	// Clear the accelerometer update available flag if it was set
	imuUpdateFlag &= ~ACCEL_AVAILABLE_FLAG;

	return accelMeas;
}

vector3f GetGyroData()
{
	// Convert the measurements into deg/s
	gyroRates.x = (float)gyroRatesRaw.x / 14.375f;
	gyroRates.y = (float)gyroRatesRaw.y / 14.375f;
	gyroRates.z = (float)gyroRatesRaw.z / 14.375f;

	// Clear the gyro update available flag if it was set
	imuUpdateFlag &= ~GYRO_AVAILABLE_FLAG;

	return gyroRates;
}

void RunWatchdogTick()
{
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) && !(internalStateFlags & ACCEL_AWAITING_I2C))
	{
		// Stale accel connectionn! Perform manual innterrupt handle/simulate an innterrupt to (hopefully) clear it!
		HandleAccelInterrupt();
	}
}
