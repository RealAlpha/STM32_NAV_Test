/*
 * imu_helpers.h
 *
 *  Created on: Aug 11, 2021
 *      Author: alpha-v
 */

#ifndef IMU_HELPERS_H_
#define IMU_HELPERS_H_

#include "common.h"
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

// Convert the 7-bit accelerometer/gyro/magnetometer addresses to the format HAL expects
#define ADXL345_ADDR 0x53 << 1
#define GYRO_ADDR 0x68 << 1
#define QMC_ADDR 0x0D << 1//0x1E << 1

// Define update flags so the main loop can check/clear incoming measurements
#define ACCEL_AVAILABLE_FLAG 0b1
#define GYRO_AVAILABLE_FLAG 0b1 << 1
#define QMC_AVAILABLE_FLAG 0b1 << 2

// Flags used for internal state operation
// Define masks/offsets to make it easy to check if anything is still awaiting/using the bus (better than manually coding in all of the combo's)
#define STATE_NEED_OFFS 4
#define STATE_AWAITING_MASK 0b1111
#define STATE_NEED_MASK 0b1111 << STATE_NEED_OFFS

// TODO: Currently does not implement a mechanism for when an interrupt is triggered while we're still waiting for data - this needs to be fixed, possibly by using a latent check and/or a second state variable that figures out which i2c read call was the first one
#define ACCEL_AWAITING_I2C 0b1
#define GYRO_AWAITING_I2C 0b1 << 1
#define QMC_AWAITING_I2C 0b1 << 2
// NOTE: Below 2 are used for overlapping interrupts / starting an accel or gyro read after we're done with the one that is ongoing at the time of the interrupt
#define ACCEL_NEED_I2C 0b1 << (STATE_NEED_OFFS + 0)
#define GYRO_NEED_I2C 0b1 << (STATE_NEED_OFFS + 1)
#define QMC_NEED_I2C 0b1 << (STATE_NEED_OFFS + 2)

// Measured biasses (deviation from 1g when held steady in g's - should be SUBTRACTED from measurements)
// TODO: These are measured - should they be params insntead?
#define ACCEL_BIAS_X -0.01229200000000008f
#define ACCEL_BIAS_Y 0.027628000000000208f
#define ACCEL_BIAS_Z 0.1611999999999998f

// Globals for storing the latest measurements + flags
// TODO: Remove/superceded by function - or should we set these if we need the data multiple times between updates?
vector3f accelMeas;
vector3f gyroRates;
vector3f qmcMeas;
uint8_t imuUpdateFlag;

// Internal state - used for communication between various IRQ handlers
uint8_t internalStateFlags;
/// Raw data values to avoid having to call the FPU/perform conversion steps during interrupts. Currently does convert into the integer representation, but I suppoose that even that could be changed
vector3i16 accelMeasRaw;
vector3i16 gyroRatesRaw;
vector3i16 qmcMeasRaw;
/// Buffers used for memory reads
// TODO: Unify data buffers? Can only have one I2C transfer at once / can just pick whatever is the largest needed buffer
uint8_t accelDataBuffer[6];
uint8_t gyroDataBuffer[6];
uint8_t qmcDataBuffer[7];
/// Copy of the i2c peripheral to use for communication
I2C_HandleTypeDef *imuI2CHandle;

/**
 * Performs initializatio of the GY-85 IMU.
 * @param hi2c The I2C interface the IMU is connected to
 * @param AccelRate The accelerometer's RATE
 * @param GyroRate The gyroscope's RATE
 */
void PerformImuConfiguration(I2C_HandleTypeDef *hi2c, uint16_t AccelRate, uint16_t GyroRate);

void HandleAccelInterrupt();
void HandleGyroInterrupt();
void HandleQMCInterrupt();

void HandleI2CInterrupt(I2C_HandleTypeDef *hi2c);

/*
 * Returns the latest accelerometer data, converted to g's
 */
vector3f GetAccelData();

/*
 * Returns the latest gyroscope data, converted to deg/s
 */
vector3f GetGyroData();

/*
 * Returns the latest magnetometer data, converted to Gauss
 */
vector3f GetMagData();

// Checks for "stale" ADXL interrupts / line remaining HIGH w/o I2C interraction
void RunWatchdogTick();

#endif /* IMU_HELPERS_H_ */
