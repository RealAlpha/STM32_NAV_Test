/*
 * commo.h
 *
 *  Created on: Aug 11, 2021
 *      Author: alpha-v
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#include <stdint.h>

/*
 * Simple float vector in R^3
 */
typedef struct
{
	float x;
	float y;
	float z;
} vector3f;

/*
 * Simple int16_t vector in R^3
 */
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
} vector3i16;

#endif /* INC_COMMON_H_ */
