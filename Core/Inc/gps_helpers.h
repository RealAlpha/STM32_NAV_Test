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

// Calculate the total packet length to allocate a buffer of the right size
// Components: sync (2), msg CLS (1), msg ID (1), length (2), payload (provided), checksum (2)
// NOTE: This is a macro to allow it to be used during compile time
#define GET_PACKET_LENGTH(payload)  2 + 1 + 1 + 2 + sizeof(payload) + 2

// After much head scratching, it became apparent that hte author's dev board contained a serial port with a wrong baudrate.
// This (measured) correction factor fixes that/avoids noise interrupts. Set to 1 for "normal" boards
#define BAUDRATE_FUDGEFACTOR 1.0447968

// Shorthands for fields with no flags
#define NO_FLAGS_8 0b00000000;
#define NO_FLAGS_16 0b0000000000000000;

#define PRT_CFG_MODE_BASE 0b1 << 4 // The reserved1 compatibility thing
#define PRT_CFG_MODE_STOPBITS_1 0b00000000000000000000000000000000
#define PRT_CFG_MODE_CHAR_LEN_8 0b00000000000000000000000011000000
//#define PRT_CFG_MODE_PARITY_NONE 0b00000000000000000000100000000000
#define PRT_CFG_MODE_PARITY_NONE 0b100 << 11
#define PRT_CFG_PROTO_UBX 0b1
#define PRT_CFG_FLAGS_NONE 0


#define MSG_CLASS_PRT 0x06

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
 * Performs protocol negotiation / switches to UBX-only mode and ups the baudrate to the desired value.
 * NOTE: This is a blocking call that will abort/break existing interactions with the peripheral (due to baudrate changes and the like)
 *
 * @param huart The U(S)ART port the GPS module is connected to.
 * @param receiverBaudRate The baudrate the receiver is currently at.
 * @param desiredBaudRate The baudrate that should be used for communications from now on.
 */
void PerformProtoNegotiation(UART_HandleTypeDef *huart, unsigned int receiverBaudRate/* = 9600*/, unsigned int desiredBaudRate/* = 115200*/);

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
typedef struct {
	unsigned char portID;
	unsigned char reserved0;
	uint16_t txReady;
	uint32_t mode;
	unsigned int baudRate;
	uint16_t inProtoMask;
	uint16_t outProtoMask;
	uint16_t flags;
	unsigned short reserved5;
} CFG_PRT;

#endif /* INC_GPS_HELPERS_H_ */
