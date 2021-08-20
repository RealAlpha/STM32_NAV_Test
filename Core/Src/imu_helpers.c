/*
 * imu_helpers.c
 *
 *  Created on: Aug 11, 2021
 *      Author: alpha-v
 */
#include "imu_helpers.h"

// Start PD
/*
 * QMC Chip Register Helper Defines
 */
// Register Addresses
#define QMC_ADD_CR1 0x09
#define QMC_ADD_CR2 0x0A
#define QMC_ADD_STATUS 0x06




// Bitfield Offsets (CR1)
#define QMC_OFFS_CR1_MODE 0 // Mode
#define QMC_OFFS_CR1_ODR 2 // Output Data Rate
#define QMC_OFFS_CR1_RNG 4 // measurement RaNGe
#define QMC_OFS_CR1_OSR 6 // Over Sample Ratio


// Bitfield Values (CR1)
#define QMC_CR1_MODE_STBY 0b00 << QMC_OFFS_CR1_MODE
#define QMC_CR1_MODE_CONT 0b01 << QMC_OFFS_CR1_MODE

#define QMC_CR1_ODR_10HZ 0b00 << QMC_OFFS_CR1_ODR
#define QMC_CR1_ODR_50HZ 0b01 << QMC_OFFS_CR1_ODR
#define QMC_CR1_ODR_100HZ 0b10 << QMC_OFFS_CR1_ODR
#define QMC_CR1_ODR_200HZ 0b11 << QMC_OFFS_CR1_ODR

#define QMC_CR1_RNG_2G 0b00 << QMC_OFFS_CR1_RNG
#define QMC_CR1_RNG_8G 0b01 << QMC_OFFS_CR1_RNG

#define QMC_CR1_OSR_512 0b00 << QMC_OFS_CR1_OSR
#define QMC_CR1_OSR_256 0b01 << QMC_OFS_CR1_OSR
#define QMC_CR1_OSR_128 0b10 << QMC_OFS_CR1_OSR
#define QMC_CR1_OSR_64 0b11 << QMC_OFS_CR1_OSR




// Bitfield Offsets (CR2)
#define QMC_OFFS_CR2_INT_ENB 0 // Interrupt Enable
#define QMC_OFFS_CR2_ROL_PNT 6 // Automatic pointer rollover when reading in ranges 0x00 ~ 0x06 (data registers + status register
#define QMC_OFFS_CR2_SOFT_RST 7 // Software reset


// Bitfield Values (CR2)
#define QMC_CR2_INT_ENB 0b0 << QMC_OFFS_CR2_INT_ENB // NOTE: Kinda weird, but a 0 enables it and a 1 disables it
#define QMC_CR2_ROL_PNT 0b1 << QMC_OFFS_CR2_ROL_PNT
#define QMC_CR2_SOFT_RST 0b1 << QMC_OFFS_CR2_SOFT_RST




// Bitfield Offsets (STATUS)
#define QMC_OFFS_STATUS_DRDY 0 // Indicates that all three axis' data is available to be read
#define QMC_OFFS_STATUS_OVL 1 // Indicates that the measured data is out of range (i.e. exceeds plus or minus 2^15)
#define QMC_OFFS_STATUS_DOR 2 // Possibly indicates that previous read did not complete before it was flushed with new data, but not 100% sure


// Bitfield Values (STATUS)
#define QMC_STATUS_DRDY 0b1 << QMC_OFFS_STATUS_DRDY
#define QMC_STATUS_OVL 0b1 << QMC_OFFS_STATUS_OVL
#define QMC_STATUS_DOR 0b1 << QMC_OFFS_STATUS_DOR

// End PD


void PerformImuConfiguration(I2C_HandleTypeDef *hi2c, uint16_t AccelRate, uint16_t GyroRate)
{
	imuI2CHandle = hi2c;
/*
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
*/

	HAL_Delay(1);
	uint8_t QMCDataDump[14];
	HAL_StatusTypeDef DumpStatus = HAL_I2C_Mem_Read(imuI2CHandle, QMC_ADDR, 0x00, 1, &QMCDataDump, 14, 1000);

	// Initialize the magnetometer
	HAL_StatusTypeDef Status; // Makes debugging easier // we can step -> only need one; TODO: Implement status checking/retrying?
	// Continuous mode, 200Hz sample rate, +-8G data range, max. oversampling (TODO: Can probably be 2G barring large magnets),
	uint8_t QMC_CR1 = QMC_CR1_MODE_CONT | QMC_CR1_ODR_200HZ | QMC_CR1_OSR_512 | QMC_CR1_RNG_8G;
	// Enable rollover so we can read the status + all 6 pieces of address data in one read operation. That way we still comply with the advisory to check it, but don't have to read twice // we can always discard data if it isn't valid
	uint8_t QMC_CR2 = QMC_CR2_INT_ENB | QMC_CR2_ROL_PNT;
	uint8_t QMC_SET_RESET = 0x01;
	Status = HAL_I2C_Mem_Write(imuI2CHandle, QMC_ADDR, 0x0B, 1, &QMC_SET_RESET, 1, 1000); // TODO: Make neater
	Status = HAL_I2C_Mem_Write(imuI2CHandle, QMC_ADDR, QMC_ADD_CR1, 1, &QMC_CR1, 1, 1000);
	Status = HAL_I2C_Mem_Write(imuI2CHandle, QMC_ADDR, QMC_ADD_CR2, 1, &QMC_CR2, 1, 1000);



	// Force-handle interrupt to avoid staying HIGH; TODO: only trigger when it was low?
	////HandleAccelInterrupt();
	HandleQMCInterrupt();
	//HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)
}


void HandleAccelInterrupt()
{
	// TODO
	return;
	// Ensure we don't start an I2C request while still waiting for another one to complete!
	if (!(internalStateFlags & STATE_AWAITING_MASK))
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
	// TODO
	return;
	// Ensure we don't start an I2C request while still waiting for another one to complete!
	if (!(internalStateFlags & STATE_AWAITING_MASK))
	{
		internalStateFlags |= GYRO_AWAITING_I2C;
		HAL_StatusTypeDef Status = HAL_I2C_Mem_Read_IT(imuI2CHandle, GYRO_ADDR, 0x1D, 1, gyroDataBuffer, 6);
	}
	else
	{
		internalStateFlags |= GYRO_NEED_I2C;
	}
}

void HandleQMCInterrupt()
{
	// Ensure we don't start an I2C request while still waiting for another one to complete!
	if (!(internalStateFlags & STATE_AWAITING_MASK))
	{
		internalStateFlags |= QMC_AWAITING_I2C;
		// See the initialization - we enabled pointer rollover. This means that if we want to read the 6 data registers
		// (2 for each axis; these are 0x00-0x05), as well as the status in one go, we can simply read 7 bytes starting at
		// the status (reading the status after the data registers will clear some of its flags -> we don't want that).
		// While traditionally this would result in reading registers like 0x07, 0x08, ..., it won't in this case because
		// of that really nice pointer rollover feature, which maps us from the status register back to the data registers.
		HAL_StatusTypeDef Status = HAL_I2C_Mem_Read_IT(imuI2CHandle, QMC_ADDR, QMC_ADD_STATUS, 1, qmcDataBuffer, 7);
	}
	else
	{
		internalStateFlags |= QMC_NEED_I2C;
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
	else if (internalStateFlags & QMC_AWAITING_I2C)
	{
		// First byte/register is the status one - ensure that data was available, and that it did not exceed the ranges/produce an invalid result
		// NOTE: Don't currently care about DOR
		if ((accelDataBuffer[0] & QMC_STATUS_DRDY) && !(accelDataBuffer[0] & QMC_STATUS_OVL))
		{
			qmcMeasRaw.x = ((int)qmcDataBuffer[1] << 8) | qmcDataBuffer[1];
			qmcMeasRaw.y = ((int)qmcDataBuffer[4] << 8) | qmcDataBuffer[3];
			qmcMeasRaw.z = ((int)qmcDataBuffer[6] << 8) | qmcDataBuffer[5];
		}

		// Make it known that an update is available
		imuUpdateFlag |= QMC_AVAILABLE_FLAG;

		// Clear flag
		internalStateFlags &= ~QMC_AWAITING_I2C;
	}

	// Allow pending actions from the gyro/accelerometer/magnetometer to be run (useful when data available overlaps with an existingn transfer)
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
	else if (internalStateFlags & QMC_NEED_I2C)
	{
		internalStateFlags &= ~QMC_NEED_I2C;
		HandleQMCInterrupt();
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

vector3f GetMagData()
{
	// TODO: Convert the measurements into Gausses
	// Pretty much, we need access to the NVM stuff which stores the gains, but the datasheet has literally nothing about that
	qmcMeas.x = (float)qmcMeasRaw.x;
	qmcMeas.y = (float)qmcMeasRaw.y;
	qmcMeas.z = (float)qmcMeasRaw.z;

	// Clear the gyro update available flag if it was set
	imuUpdateFlag &= ~GYRO_AVAILABLE_FLAG;

	return gyroRates;
}


void RunWatchdogTick()
{
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) && !(internalStateFlags & STATE_AWAITING_MASK))
	{
		// Stale accel connectionn! Perform manual innterrupt handle/simulate an innterrupt to (hopefully) clear it!
		HandleAccelInterrupt();
	}
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) && !(internalStateFlags & STATE_AWAITING_MASK))
	{
		// Stale accel connectionn! Perform manual innterrupt handle/simulate an innterrupt to (hopefully) clear it!
		HandleQMCInterrupt();
	}
}
