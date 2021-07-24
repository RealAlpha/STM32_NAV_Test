/*
 * gps_helpers.h
 *
 *  Created on: Jul 24, 2021
 *      Author: alpha-v
 */

#ifndef INC_GPS_HELPERS_H_
#define INC_GPS_HELPERS_H_

//#include <cstdint>
//#include <cstddef>
#include "stm32f4xx_hal.h"

#define SYNC_A 0xB5
#define SYNC_B 0x62

// After much head scratching, it became apparent that hte author's dev board contained a serial port with a wrong baudrate.
// This (measured) correction factor fixes that/avoids noise interrupts. Set to 1 for "normal" boards
#define BAUDRATE_FUDGEFACTOR 1.0447968

/**
 * Generates the bytes corresponding (including checksum et al) that can be sent to the UBX receiver.
 *
 * @param MessageClass The class identifier of the packet
 * @param MessageID The message ID of the packet
 * @param payload The "raw" data to include in the packet - likely obtained by casting a struct
 * @param payloadSize The number of bytes payload contains
 * @param buffer A buffer that can be mutated to store the resulting packet
 * @param bufferSize The (maximum) number of bytes that can be stored in the buffer
 *
 * @return The number of bytes of packet data stored in the passed in buffer, or 0 in case of failure
 */
size_t GetPacketData(uint8_t MessageClass, uint8_t MessageID, uint8_t *payload, uint16_t payloadSize, uint8_t *buffer, size_t bufferSize);

/**
 * Updates the baudrate of the passed in serial.
 * @note Includes the weird clock issue correction factor.
 * @note Will likely cancel all existing transfers/interactions.
 *
 * @param huart The U(S)ART port to update
 * @param baudrate The desired baudrate
 * @return
 */
void UpdateBaudRate(UART_HandleTypeDef *huart, unsigned int newBaudRate);

//bool DecodePacket(uint8_t *packet, uint8_t packetMaxSize,  uint8_t &MessageClass, uint8_t &MessageID, size_t &payloadSize);


/*
 * Structures representing common requests/updates/responses to and from the UBX receiver
 */
//TODO

#endif /* INC_GPS_HELPERS_H_ */
