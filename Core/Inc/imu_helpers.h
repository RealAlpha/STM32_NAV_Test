/*
 * imu_helpers.h
 *
 *  Created on: Aug 11, 2021
 *      Author: alpha-v
 */

#ifndef IMU_HELPERS_H_
#define IMU_HELPERS_H_

#include "common.h"
#include "stm32f4xx_hal.h"

// Convert the 7-bit accelerometer/gyro addresses to the format HAL expects
#define ADXL345_ADDR 0x53 << 1
#define GYRO_ADDR 0x68 << 1

// Define update flags so the main loop can check/clear incoming measurements
#define ACCEL_AVAILABLE_FLAG 0b1;
#define GYRO_AVAILABLE_FLAG 0b1 << 1;

// Globals for storing the latest measurements + flags
vector3f accelMeas;
vector3f gyroRates;
bool imuUpdateFlag;

/**
 * Performs initializatio of the GY-85 IMU.
 * @param hi2c The I2C interface the IMU is connected to
 * @param AccelRate The accelerometer's RATE
 * @param GyroRate The gyroscope's RATE
 */
void PerformConfiguration(I2C_TypeDef *hi2c, uint16_t AccelRate, uit16_t GyroRate);

#endif /* IMU_HELPERS_H_ */
