/*
 * gps_helpers.cpp
 *
 *  Created on: Jul 24, 2021
 *      Author: alpha-v
 */

#include "gps_helpers.h"
//#include <cstring>
#include <stdbool.h>

// Ensure we start parsing the buffer at the start, but also avoid multiple definitions issues when placing default in the header
unsigned int currentGPSBufferPosition = 0;


uint8_t GPSBuffer[GPS_BUFFER_SIZE] = {'\0'};

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
	// TODO: Figure out why these two need to be swapped (likely MSB vs LSB)
	buffer[5] = (payloadSize >> 8) & 0xFF;
	buffer[4] = payloadSize & 0xFF;

	// Copy over the payload to the buffer
	memcpy(&buffer[6], payload, payloadSize);

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
	UpdateBaudRate(huart, 10030);

	// Generate the new port configuration
	CFG_PRT NewPortConfig;
	NewPortConfig.portID = 0x01; // UART port
	NewPortConfig.reserved0 = 0x00; // Reserved - obtained from u-center's implementation
	NewPortConfig.txReady = 0; // Not using any txReady features
	NewPortConfig.mode = PRT_CFG_MODE_BASE | PRT_CFG_MODE_CHAR_LEN_8 | PRT_CFG_MODE_PARITY_NONE | PRT_CFG_MODE_STOPBITS_1;
	NewPortConfig.baudRate = desiredBaudRate;
	NewPortConfig.inProtoMask = PRT_CFG_PROTO_UBX;
	NewPortConfig.outProtoMask = PRT_CFG_PROTO_UBX;
	NewPortConfig.flags = PRT_CFG_FLAGS_NONE; // No need for an extended TX timeout
	NewPortConfig.reserved5 = 0;

	uint8_t packet[GET_PACKET_LENGTH(CFG_PRT)];
	size_t ProducedSize = GetPacketData(MSG_CLASS_CFG, 0x00, (uint8_t*)&NewPortConfig, sizeof(NewPortConfig), packet, sizeof(packet));
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
	UpdateBaudRate(huart, 120361);

	// Give the GPS module 500ms to make the change
	HAL_Delay(500);

	// Configure the PVT/fix information to be sent every nav epoch
	CFG_MSG NewPvtMsgConfig;
	NewPvtMsgConfig.msgClass = 0x01;
	NewPvtMsgConfig.msgID = 0x07;
	NewPvtMsgConfig.rate = 0x01;
	uint8_t msgPacket[GET_PACKET_LENGTH(CFG_MSG)];
	size_t msgProducedSize = GetPacketData(MSG_CLASS_CFG, 0x01, (uint8_t*)&NewPvtMsgConfig, sizeof(NewPvtMsgConfig), msgPacket, sizeof(msgPacket));
	UpdateResult = HAL_UART_Transmit(huart, msgPacket, msgProducedSize, 1000);
	if (UpdateResult != HAL_OK)
	{
		// Something went wrong sending the packet
		return;
	}
}

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
    huart->Instance->BRR = UART_BRR_SAMPLING8(pclk, newBaudRate);
  }
  else
  {
    huart->Instance->BRR = UART_BRR_SAMPLING16(pclk, newBaudRate);
  }

  // Re-enable the serial
  huart->Instance->CR1 |= USART_CR1_UE;
}

void ProcessGPSBuffer(uint16_t EndPos)
{
	// NOTE: Written for a single-core system, and assuming interrupts requiring the data happen.
	// TODO: Re-evaluate this/possibly implement locking in the future.

	// Declare some variables to keep track of the parse state of the message
	// TODO: Investigate implementing flags or enums rather than separate booleans to save memory?
	EParseState parseState = SEARCHING_SYNC_A;
	uint8_t ClassID = 0;
	uint8_t MessageID = 0;
	uint16_t PayloadLength = 0;
	uint8_t CK_A, CK_B = 0;

	// NOTE: EndPos can require us to "wrap" around the end of the buffer, so to implement this in an easy way,
	//       we'll allow it to "extend" to numbers greater than the buffer size, and then use the modulo operator
	//       to make sure it's a valid index. This way we naturally wrap around the end of the array without having
	//       to write completely different code in case the end position is less than our starting position (i.e. we need to wrap)
	// NOTE: For the loop, the first set of brackets A. checks if we need to wrap, and B. allows it to wrap around to whatever the place
	//       in the buffer from the start we want. The second set is for the "regular" case where the end position is "in the future"
	unsigned int parsePos = currentGPSBufferPosition;
	while ((currentGPSBufferPosition > EndPos && parsePos < (GPS_BUFFER_SIZE + EndPos)) || (parsePos < EndPos))
	{
		switch (parseState)
		{
		case SEARCHING_SYNC_A:
			// Still searching for the first sync pattern byte - is this it?
			if (GPSBuffer[parsePos % GPS_BUFFER_SIZE] == SYNC_A)
			{
				// Yes! Now we just need to make sure the second byte matches as well
				parseState = SEARCHING_SYNC_B;
			}
			break;
		case SEARCHING_SYNC_B:
			if (GPSBuffer[parsePos % GPS_BUFFER_SIZE] == SYNC_B)
			{
				// Successfully found the full sync pattern! Now we can start copying over message data
				parseState = EXTRACT_CLASS_ID;
			}
			else
			{
				// Invalid sync pattern! Reset everything and start looking for the next (fully valid) sync pattern
				parseState = SEARCHING_SYNC_A;
			}
			break;

		case EXTRACT_CLASS_ID:
			// This byte represents the class ID
			ClassID = GPSBuffer[parsePos % GPS_BUFFER_SIZE];
			parseState = EXTRACT_MESSAGE_ID;
			CK_A += GPSBuffer[parsePos % GPS_BUFFER_SIZE]; CK_B += CK_A;
			break;
		case EXTRACT_MESSAGE_ID:
			// This byte represents the message ID
			ClassID = GPSBuffer[parsePos % GPS_BUFFER_SIZE];
			parseState = EXTRACT_PAYLOAD_LENGTH_A;
			CK_A += GPSBuffer[parsePos % GPS_BUFFER_SIZE]; CK_B += CK_A;
			break;

		// Assume GPS and STM32 chip are both LSB -> that means that the first byte (of the 16-bit length)
		// should be shifted over by 8 bits, and the second byte should just be OR-ed "on" to it
		case EXTRACT_PAYLOAD_LENGTH_A:
			PayloadLength = GPSBuffer[parsePos % GPS_BUFFER_SIZE];// << 8;
			parseState = EXTRACT_PAYLOAD_LENGTH_B;
			CK_A += GPSBuffer[parsePos % GPS_BUFFER_SIZE]; CK_B += CK_A;
			break;
		case EXTRACT_PAYLOAD_LENGTH_B:
			PayloadLength |= GPSBuffer[parsePos % GPS_BUFFER_SIZE] << 8;
			parseState = COPY_PAYLOAD;
			CK_A += GPSBuffer[parsePos % GPS_BUFFER_SIZE]; CK_B += CK_A;
			break;

		case COPY_PAYLOAD:
			// At this point, we've collected enough information to A. figure out which packet we're extracting, and B. perform a preliminary
			// check to make sure the length matches up with what we expect

			// TODO: Might want to "manually" fetch the checksum and run the computation on the buffer now, then make sure it's valid and if
			// it is we can just write to the global (see single-threaded/no locking considerations earlier) -> avoids having to allocate
			// and free (addl.) memory for holding the payload during the parse + then later having to use different variables to compute
			// the checksum

			// Ensure the payload + two checksum bytes fit within the buffer of what is avaiable - if not the message was either malformed
			// or we simply don't have enough data available.
			// TODO: check if bufferEnd is 0-indexed or really 1-indexed like we currently assume
			// NOTE: Also incorporate check to avoid the payload being multiple times the size of the buffer -> wrapping around by more than
			// once...this assumes that currentGPSBufferPosition is the positionn where the parse started
			if ((parsePos + PayloadLength + 2) % GPS_BUFFER_SIZE >= EndPos || (parsePos + PayloadLength + 2 - currentGPSBufferPosition) > GPS_BUFFER_SIZE)
			{
				parseState = SEARCHING_SYNC_A;
				PayloadLength = 0;
				CK_A = 0;
				CK_B = 0;
				goto parse_end;
			}

			// Compute the checksum over the payload (the other metadata has already been checksummed by the previous steps
			for (unsigned int i = parsePos; i < (parsePos + PayloadLength); i++)
			{
				CK_A += GPSBuffer[i % GPS_BUFFER_SIZE];
				CK_B += CK_A;
			}

			// Ensure the checksum matches the CK_A/CK_B bytes provided in the packet.
			// NOTE: Earlier length check ensures the buffer is this long/we can just index them
			// TODO: Add this back!

			if (CK_A != GPSBuffer[(parsePos + PayloadLength) % GPS_BUFFER_SIZE] || CK_B != GPSBuffer[(parsePos + PayloadLength + 1) % GPS_BUFFER_SIZE])
			{
				// The checksum failed! Since the whole buffer is available, we will consider the whole packet to be corrupted and skip over it.
				// TODO: We currently don't really "skip" over it, but continue lookingn for a syncn patternn (which could be half way through the packet)
				//       Is this inndeed the desired behavior, or should we just increase parsePos to the end of the parsed packet size?
				parseState = SEARCHING_SYNC_A;
				PayloadLength = 0;
				CK_A = 0;
				CK_B = 0;
				goto parse_end;
			}
			// The packet passed all checks and is valid! Attempt to match it to a known message identifier
			// TODO: Implement this
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
			parsePos += PayloadLength + 1;
			currentGPSBufferPosition = parsePos % GPS_BUFFER_SIZE;
			parseState = SEARCHING_SYNC_A;
			PayloadLength = 0;
			CK_A = 0;
			CK_B = 0;
			continue;
			break;

		}

parse_end:
		parsePos++;

	}

}
