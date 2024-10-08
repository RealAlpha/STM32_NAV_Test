/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gps_helpers.h"
#include "imu_helpers.h"
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>
#define CRC_CCITT
#include "../ThirdParty/CRC/crc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERIAL_BUFFER_SIZE 4096
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

// Avoid processing interrupts while we're still initing
bool initCompleted = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t Buffer[256];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Turn off the GPS fix status LED
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  // Initialize the GPS module and configure it to use the higher baudrate/UBX protocol
  // NOTE: Innitial delay is to give the sensor bus some time to start up / testing showed the GPS module needs ~1-2 seconds to start up before we can talk to it/configure it
  HAL_Delay(2000);
  PerformProtoNegotiation(&huart2, 9600, 115200);
  HAL_StatusTypeDef StartListenResult = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, GPSBuffer, GPS_BUFFER_SIZE);

/*
  uint8_t Buffer[25] = {0};
  uint8_t Space[] = " - ";
  uint8_t StartMSG[] = "Starting I2C Scanning: \r\n";
  uint8_t EndMSG[] = "Done! \r\n\r\n";
  HAL_UART_Transmit(&huart2, StartMSG, sizeof(StartMSG), 10000);
  uint8_t i = 0, ret;
      for(i=1; i<128; i++)
      {
          ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
          if (ret != HAL_OK) // No ACK Received At That Address
          {
              HAL_UART_Transmit(&huart2, Space, sizeof(Space), 10000);
          }
          else if(ret == HAL_OK)
          {
              sprintf(Buffer, "0x%X", i);
              HAL_UART_Transmit(&huart2, Buffer, sizeof(Buffer), 10000);
          }
      }
      HAL_UART_Transmit(&huart2, EndMSG, sizeof(EndMSG), 10000);
      //--[ Scanning Done ]--
*/


  PerformImuConfiguration(&hi2c1, 3200, 3200);
  unsigned long int last_iTOW = 0;
  char buffer[SERIAL_BUFFER_SIZE];

  // Completed setup!
  initCompleted = true;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*
	  // TODO: Not ideal//replace with more robust alternative
	  // Pretty much catches case when no update is available for 1x
	  if (imuUpdateFlag == 0)
	  {
		  HAL_Delay(100);
		  if (imuUpdateFlag == 0)
		  {
			  LogDebugMessage("Lockup Detected!");
			  HAL_I2C_Master_Abort_IT(&hi2c1, GYRO_ADDR);
			  HAL_I2C_Master_Abort_IT(&hi2c1, ADXL345_ADDR);
			  HAL_I2C_StateTypeDef I2CState = HAL_I2C_GetState(&hi2c1);
			  LogDebugMessage("I2C State: %d", (int)I2CState);
			  // Disable and then re-ennable the i2c peripheral

			  hi2c1.Instance->CR1 |= I2C_CR1_STOP;
			  hi2c1.Instance->CR1 &= ~I2C_CR1_PE;
			  HAL_Delay(100);
			  hi2c1.Instance->CR1 &= ~I2C_CR1_STOP;
			  hi2c1.Instance->CR1 |= I2C_CR1_PE;
		  }
	  }*/

	  // Turn the second built-in LED on when a successful (new) valid GNSS fix was found
	  if (GPSUpdateFlags & GPS_UPDATE_AVAILABLE)
	  {
		  // Was this fix valid?
		  if (lastNavFix.flags & NAV_PVT_FLAGS_OKFIX && lastNavFix.fixType == 3)
		  {
			  // Valid fix! Turn the fix light on
			  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

			  // Copy the relevant data to our GPS packet and send it
			  GPSPacket gpsPacket;
			  gpsPacket.bValidFix = true;
			  gpsPacket.latitude = lastNavFix.lat*powf(10, -7);
			  gpsPacket.longitude = lastNavFix.lon*powf(10, -7);
			  gpsPacket.height = lastNavFix.height*powf(10, -3);
			  gpsPacket.vN = lastNavFix.velN*powf(10, -3);
			  gpsPacket.vE = lastNavFix.velE*powf(10, -3);
			  gpsPacket.vD = lastNavFix.velD*powf(10, -3);
			  gpsPacket.hAcc = lastNavFix.hAcc*powf(10, -3);
			  gpsPacket.vAcc =lastNavFix.vAcc*powf(10, -3);
			  gpsPacket.sAcc = lastNavFix.sAcc*powf(10, -3);
			  SendPacket(GPS_PACKET_ID, (void*)&gpsPacket, sizeof(gpsPacket));
		  }
		  else
		  {
			  // Invalid fix! Turn the fix light off
			  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

			  // Send a packet telling the listener that we don't currently have a valid GPS fix
			  GPSPacket gpsPacket;
			  gpsPacket.bValidFix = false;
			  SendPacket(GPS_PACKET_ID, (void*)&gpsPacket, sizeof(gpsPacket));
		  }

		  // Reset the fix available flag
		  GPSUpdateFlags &= ~GPS_UPDATE_AVAILABLE;
	  }

	  if (imuUpdateFlag & ACCEL_AVAILABLE_FLAG)
	  {

		  vector3f AccelData = GetAccelData();
		  // NOTE: Blocking fn / won't unset data -> can use local var as ptr
		  SendPacket(ACCEL_PACKET_ID, (void*)&AccelData, sizeof(AccelData));
	  }

	  if (imuUpdateFlag & GYRO_AVAILABLE_FLAG)
	  {
		  vector3f GyroData = GetGyroData();
		  SendPacket(GYRO_PACKET_ID, (void*)&GyroData, sizeof(GyroData));
	  }

	  if (imuUpdateFlag & QMC_AVAILABLE_FLAG)
	  {
		  vector3f MagData = GetMagData();
		  // Obtain the norm/magnitude of the magnetic field vector
		  ///float MagNorm = sqrtf(powf(MagData.x, 2) + powf(MagData.y, 2) + powf(MagData.z, 2));

		  SendPacket(MAG_PACKET_ID, (void*)&MagData, sizeof(MagData));
	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 4000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1.0447968*9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : QMC_INT_Pin */
  GPIO_InitStruct.Pin = QMC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(QMC_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GYRO_INT_Pin ACCEL_INT_Pin */
  GPIO_InitStruct.Pin = GYRO_INT_Pin|ACCEL_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	// Is this message relevant to the UART connenction used for GPS traffic? If so, pass it along to the GPS handler for further processinng.
	// NOTE: Assuming only one (global) huart instance exists -> the address will be the same, and functions as an equals check between the two objects.
	if (huart == &huart2)
	{
		ProcessGPSBuffer(Size);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	uint32_t Error = HAL_UART_GetError(huart);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	//__HAL_UART_CLEAR_OREFLAG(huart);
	//UNUSED(huart);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// Ensure initialization was completed before we handle interrupts - it will trigger a hardFault otherwise
	if (!initCompleted)
	{
		return;
	}

	// TODO: Check IQn and/or port - pretty much, make sure this is indeed the iterrupt we want
	if (GPIO_Pin == ACCEL_INT_Pin)
	{
		HandleAccelInterrupt();
	}
	else if (GPIO_Pin == GYRO_INT_Pin)
	{
		HandleGyroInterrupt();
	}
	else if (GPIO_Pin == QMC_INT_Pin)
	{
		HandleQMCInterrupt();
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	// TODO: Implemet I2C checks (i.e. make sure it only triggers for hi2c1, not the others)
	HandleI2CInterrupt(hi2c);
}

/**
 * Simple debug print function that dumps the result to the serial. Limited to at most 2047 characters and suffixes a \n by default.
 * @param fmt The format string.
 */
void LogDebugMessage(char *fmt, ...)
{
	// TODO: Does not appear to work well with multiple floats - that will need to be fixed!
	// TODO: Replace hard-coded buffer size?
	char buffer[2048];

	va_list argp;
	va_start(argp, fmt);
	int charsWritten = vsnprintf(buffer, 2048, fmt, argp);
	va_end(argp);

	// Ensure the formatting was successful
	if (charsWritten < 2048 && charsWritten > 0)
	{
		// NOTE: We don't really care about the \0, but we do want to make sure the serial newlines after each message
		buffer[charsWritten] = '\n';
		// OTE: The +1 ensures the \n is transmitted as well
		HAL_UART_Transmit/*_IT*/(&huart1, buffer, charsWritten+1, 1000);
	}
}

void HAL_I2C_ErrorCallback (I2C_HandleTypeDef *hi2c)
{
	LogDebugMessage("ErrorCallback!");
	// Reset IMU awaiting flags so this failed transfer does not lead to blocking all future attempts at communication because we suspect the I2C bus is still busy
	// NOTE: Not automagically continuing to the next transfer as the I2C bus might need some time to recover from the error (? no basis, but just a guess) + we don't
	// want this interrupt to take too long; we might want to change this in the future to help improve bus utilization after an error. (right now the watchdog triggers pretty often / main doesn't take long to process)
	internalStateFlags &= ~STATE_AWAITING_MASK;

	return;
}

void HAL_I2C_AbortCpltCallback (I2C_HandleTypeDef *hi2c)
{
	LogDebugMessage("AbortCpltCallback!");
}


// Basic custom serial data protocol; there might be better/more "formal" ways to do this, but this is just a quick demo // our final protocol will be more advanced + will likely rely on dedicated hardware/multiple transport layers
// NOTE: This is just a made up sync pattern - no data analysis or research was performed, so it might be really bad.
#define SYNC_PATTERN_A 0xFF
#define SYNC_PATTERN_B 0x8C
#define SYNC_PATTERN_SIZE 2 // Helper function to aovid hard-coding the two in


void SendPacket(uint8_t pktId, void *data, uint16_t dataSize)
{
	// Create a temporary buffer to store the message in
	size_t packetSize = /*sync*/2 + /*packet ID*/sizeof(pktId) + /*length*/sizeof(dataSize) + /* data*/dataSize + /*checksum*/sizeof(uint16_t);
	uint8_t *packet = malloc(packetSize);

	// Pack the header
	packet[0] = SYNC_PATTERN_A;
	packet[1] = SYNC_PATTERN_B;

	// Packet ID
	packet[SYNC_PATTERN_SIZE] = pktId;
	// NOTE: Below is essentially a reinterpret cast / because we know it's large enough (due to the sizeof), we
	// can set it in this way. This avoids messing with bitmasks + ensures it's encoded in the same way as
	// the void* argument (which is probably a casted struct). A more robust protocol would probably make this
	// more deterministic/better, but for now this works...OK?
	*((uint16_t*)&packet[SYNC_PATTERN_SIZE+sizeof(pktId)]) = dataSize;

	// Copy over the actual data
	memcpy(&packet[SYNC_PATTERN_SIZE+sizeof(pktId)+sizeof(dataSize)], data, dataSize);

	// Compute the checksum over all of the packet less the space that is "reserved" for the checksum and the sync pattern
	// NOTE: Would have used the CRC peripheral, but unfortuantely that requires the data to have a size that is
	// a multiple of the size of a word -> that would need padding. This was deemed not worth it (for now), so
	// we're using an online 16-bit CRC implementation (which is probably more appropriate anyway considering the
	// reasonably small packet sizes).
	// NOTE: This static way isn't perfect, but since this is the only code that uses CRC / this is a temporary protocol anyway, it should work; KISS
	static bool crcInitialized = false;
	if (!crcInitialized)
	{
		crcInit();
		crcInitialized = true;
	}

	// Compute/set the checksum
	// NOTE: This appears to be CCITT-FALSE!
	uint16_t checksum = crcFast(&packet[SYNC_PATTERN_SIZE], sizeof(pktId)+sizeof(dataSize)+dataSize);
	*((uint16_t*)&packet[packetSize-sizeof(uint16_t)]) = checksum;

	// Transmit the packet
	HAL_UART_Transmit_DMA(&huart1, packet, packetSize);

	// Free the packet buffer to avoid leaking
	free(packet);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
