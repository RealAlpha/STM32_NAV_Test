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
#define PRT_CFG_MODE_PARITY_NONE 0b100 << (11 - 2)
#define PRT_CFG_PROTO_UBX 0b1
#define PRT_CFG_FLAGS_NONE 0


// Various NAV-PVT field flags. Currently only defined "relevant" ones
#define NAV_PVT_VALIDITY_FULLYRESOLVED 0b1 << 2
#define NAV_PVT_VALIDITY_VALIDTIME 0b1 << 1
#define NAV_PVT_VALIDITY_VALIDDATE 0b1 << 0
#define NAV_PVT_FLAGS_OKFIX 0b1 << 0


#define MSG_CLASS_NAV 0x01
#define MSG_CLASS_CFG 0x06



/*
 * Data types used in the UBX protocol description. Makes it easier to copy stuff from the docs
 */
#define UBX_U1 unsigned char
#define UBX_I1 char
#define UBX_X1 uint8_t
#define UBX_U2 unsigned short
#define UBX_I2 short
#define UBX_X2 uint16_t
#define UBX_U4 unsigned long
#define UBX_I4 long
#define UBX_X4 uint32_t



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

/**
 * Processed the GPS buffer when a half, complete or idle timeout is detected on the GPS's serial connection
 * @note This might update global states depending on the packets that come in
 * @note To avoid overfilling the buffer, data is processed immediately, which might block the thread for some time.
 *
 * @param EndPos The Size/place in the buffer till where data is available.
 */
void ProcessGPSBuffer(uint16_t EndPos);

//bool DecodePacket(uint8_t *packet, uint8_t packetMaxSize,  uint8_t &MessageClass, uint8_t &MessageID, size_t &payloadSize);


/*
 * Structures representing common requests/updates/responses to and from the UBX receiver
 */
typedef struct {
	UBX_U1 portID;
	UBX_U1 reserved0;
	UBX_X2 txReady;
	UBX_X4 mode;
	UBX_I4 baudRate;
	UBX_X2 inProtoMask;
	UBX_X2 outProtoMask;
	UBX_X2 flags;
	UBX_U2 reserved5;
} CFG_PRT;

typedef struct {
	// Class of the message you'd like to change the rate of
	UBX_U1 msgClass;

	// ID of the message you'd like to change the rate of
	UBX_U1 msgID;

	// Every how many nav epochs the message should be sent
	UBX_U1 rate;
} CFG_MSG;

typedef struct {
	UBX_U4 iTOW; // GPS time of week in miliseconds
	UBX_U2 year; // Current year (in Zulu)
	UBX_U1 month; // Current month (in Zulu)
	UBX_U1 day; // Current day (in Zulu)
	UBX_U1 hour; // Current hour (in Zulu)
	UBX_U1 min; // Current minute (in Zulu)
	UBX_U1 sec; // Current second (in Zulu)

	UBX_X1 valid; // TODO
	UBX_U4 tAcc; // Time accuracy in nanoseconds
	UBX_I4 nano; // Current nanosecond "compensation" - could be + or - (in Zulu)
	// ???
	UBX_U1 fixType;
	UBX_X1 flags; // TODO
	UBX_U1 reserved1; // Reserved

	UBX_U1 numSV; // Number of sats used in NAV solution

	UBX_I4 lon; // Current longitude in 10^-7 degrees
	UBX_I4 lat; // Current latitude in 10^-7 degrees
	UBX_I4 height; // Height above ellipsoid model measured in millimetres
	UBX_I4 hMSL; // Height above the mean sea level(/equipotential surface?) in millimeters

	UBX_U4 hAcc; // Horizontal (NOT height) measurement accuracy estimate in mm
	UBX_U4 vAcc; // Vertical measurement accuracy estimate in mm

	UBX_I4 velN; // Velocity in the North direction in mm/s (NED model)
	UBX_I4 velE; // Velocity in the East direction in mm/s (NED model)
	UBX_I4 velD; // Velocity in the Down direction in mm/s (NED model)
	UBX_I4 gSpeed; // 2D ground speed in mm/s
	UBX_I4 heading; // Current heading in 10^-5 degrees

	UBX_U4 sAcc; // Speed measurement accuracy estimate in mm/s
	UBX_U4 headingAcc; // Heading accueracy estimate in 10^-5 degrees

	UBX_U2 pDOP; // Position DOP scaled to 0.01/count

	UBX_X2 reserved2; // Reserved
	UBX_U4 reserved3; // Reserved
} NAV_PVT;


/*
 * Enums used for representing states/properties with a finite set of options
 */
// Keeps track of the parsing state
typedef enum
{
	SEARCHING_SYNC_A,
	SEARCHING_SYNC_B,
	EXTRACT_CLASS_ID,
	EXTRACT_MESSAGE_ID,
	EXTRACT_PAYLOAD_LENGTH_A,
	EXTRACT_PAYLOAD_LENGTH_B,
	COPY_PAYLOAD,
	EXTRACT_CHEKSUM_A,
	EXTRACT_CHECKSUM_B,
	VERIFY_CHECKSUM,
	STATE_UNKNOWN
} EParseState;


/*
 * Globals used for parsing and the like. It isn't ideal, but it might be the best we can do with C / without OOP
 */
// Number of bytes the (circular/DMA-filled) GPS buffer should have
#define GPS_BUFFER_SIZE 512

// The GPS buffer global that will be filled by incoming UART messages // used as a DMA circular buffer
uint8_t GPSBuffer[GPS_BUFFER_SIZE];

// The current position in the GPS buffer - considering everything before this position to have been previously parsed/no longer relevant
unsigned int currentGPSBufferPosition;

/*
 * Global state variables. Again, not ideal but probably the best way to implement "background"/interrupt-based processing without using C++/OOP.
 */
NAV_PVT lastNavFix;

#endif /* INC_GPS_HELPERS_H_ */
