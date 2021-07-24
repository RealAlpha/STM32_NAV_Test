/*
 * gps_helpers.cpp
 *
 *  Created on: Jul 24, 2021
 *      Author: alpha-v
 */

#include "gps_helpers.h"
//#include <cstring>

size_t GetPacketData(uint8_t MessageClass, uint8_t MessageID, uint8_t *payload,
		uint16_t payloadSize, uint8_t *buffer, size_t bufferSize) {
	// Calculate the total packet length to ensure that it will fit in the buffer
	// Components: sync (2), msg CLS (1), msg ID (1), length (2), payload (provided), checksum (2)
	uint16_t totLength = 2 + 1 + 1 + 2 + payloadSize + 2;
	if (totLength > bufferSize)
	{
		// Won't fit!
		return 0;
	}

	// Copy over the sync pattern, message identifiers and payload size
	buffer[0] = SYNC_A;
	buffer[1] = SYNC_B;
	buffer[2] = MessageClass;
	buffer[3] = MessageID;
	buffer[4] = (payloadSize >> 8) & 0xFF;
	buffer[5] = payloadSize & 0xFF;

	// Copy over the payload to the buffer
	memcpy(&buffer[6], buffer, bufferSize);

	// Compute the checksum over the entire packet (up to this point) minus the sync pattern (still easier to just copy everything +
	// then start at two than to compute the checksum of the first part manually). totLength - 2 because checksum
	// is also not included
	uint8_t CK_A, CK_B = 0;
	for (int i = 2; i < totLength - 2; i++)
	{
		CK_A += buffer[i];
		CK_B += CK_A;
	}

	// Add the checksum to the packet (don't forget 0-indexing) + produce a function response
	buffer[totLength - 2] = CK_A;
	buffer[totLength - 1] = CK_B;

	return totLength;
}

void PerformProtoNegotiation(UART_HandleTypeDef *huart, unsigned int receiverBaudRate, unsigned int desiredBaudRate)
{
	// Change baudrate to the receiver's baudrate
	// TODO: Perform some kind of BRR check to not change unnecessarily?
	UpdateBaudRate(huart, receiverBaudRate);

	// Generate the new port configuration
	CFG_PRT NewPortConfig;
	NewPortConfig.portID = 0x01; // UART port
	NewPortConfig.reserved0 = 0b00000001; // Reserved - obtained from u-center's implementation
	NewPortConfig.txReady = 0; // Not using any txReady features
	NewPortConfig.mode = PRT_CFG_MODE_BASE | PRT_CFG_MODE_CHAR_LEN_8 | PRT_CFG_MODE_PARITY_NONE | PRT_CFG_MODE_STOPBITS_1;
	NewPortConfig.baudRate = desiredBaudRate;
	NewPortConfig.inProtoMask = PRT_CFG_PROTO_UBX;
	NewPortConfig.outProtoMask = PRT_CFG_PROTO_UBX;
	NewPortConfig.flags = PRT_CFG_FLAGS_NONE; // No need for an extended TX timeout

	uint8_t packet[GET_PACKET_LENGTH(CFG_PRT)];
	size_t ProducedSize = GetPacketData(MSG_CLASS_PRT, 0x00, (uint8_t*)&NewPortConfig, sizeof(NewPortConfig), packet, sizeof(packet));
	if (ProducedSize == 0)
	{
		// Something went wrong creating the packet!
		return;
	}

	// Send the generated PRT-CFG update packet (synchronously as we need to switch the backrate back afterwards)
	HAL_StatusTypeDef UpdateResult = HAL_UART_Transmit(huart, packet, ProducedSize, 1000);
	if (UpdateResult != HAL_OK)
	{
		// Something went wrong sending the packet
		return;
	}
	// TODO: Listen for ACK(/NACK)? Also, might want to turn this into a boolean returning function

	// Update the baudrate to the new one
	UpdateBaudRate(huart, desiredBaudRate);
;}

void UpdateBaudRate(UART_HandleTypeDef *huart,
		unsigned int newBaudRate) {

	// Disable the serial
	huart->Instance->CR1 &= ~(USART_CR1_UE);

	// Find the appropriate division thingy & set BRR (adapted from the hal_usart library)
	uint32_t pclk;
#if defined(USART6) && defined(UART9) && defined(UART10)
    if ((huart->Instance == USART1) || (huart->Instance == USART6) || (huart->Instance == UART9) || (huart->Instance == UART10))
    {
      pclk = HAL_RCC_GetPCLK2Freq();
    }
#elif defined(USART6)
    if ((huart->Instance == USART1) || (huart->Instance == USART6))
    {
      pclk = HAL_RCC_GetPCLK2Freq();
    }
#else
    if (huart->Instance == USART1)
    {
      pclk = HAL_RCC_GetPCLK2Freq();
    }
#endif /* USART6 */
    else
    {
      pclk = HAL_RCC_GetPCLK1Freq();
    }
  /*-------------------------- USART BRR Configuration ---------------------*/
  if (huart->Init.OverSampling == UART_OVERSAMPLING_8)
  {
    huart->Instance->BRR = UART_BRR_SAMPLING8(pclk, newBaudRate * BAUDRATE_FUDGEFACTOR);
  }
  else
  {
    huart->Instance->BRR = UART_BRR_SAMPLING16(pclk, newBaudRate * BAUDRATE_FUDGEFACTOR);
  }

  // Re-enable the serial
  huart->Instance->CR1 |= USART_CR1_UE;
}
