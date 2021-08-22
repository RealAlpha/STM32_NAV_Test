/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
	// Contains only the relevant/slightly processed GPS params
	bool bValidFix;
	float latitude;
	float longitude;
	float height;
	float vN;
	float vE;
	float vD;
	float hAcc;
	float vAcc;
	float sAcc;
} GPSPacket;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOA
#define QMC_INT_Pin GPIO_PIN_4
#define QMC_INT_GPIO_Port GPIOC
#define QMC_INT_EXTI_IRQn EXTI4_IRQn
#define GYRO_INT_Pin GPIO_PIN_0
#define GYRO_INT_GPIO_Port GPIOB
#define GYRO_INT_EXTI_IRQn EXTI0_IRQn
#define ACCEL_INT_Pin GPIO_PIN_1
#define ACCEL_INT_GPIO_Port GPIOB
#define ACCEL_INT_EXTI_IRQn EXTI1_IRQn
/* USER CODE BEGIN Private defines */
#define ACCEL_PACKET_ID 0
#define GYRO_PACKET_ID 1
#define MAG_PACKET_ID 2
#define GPS_PACKET_ID 3
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
