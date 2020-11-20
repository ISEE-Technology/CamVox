/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef IS_SIMPLE_INTERFACE_H
#define IS_SIMPLE_INTERFACE_H

#include "data_sets.h"
#include "stddef.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 *	DEFINITIONS AND CONVENTIONS
 *	
 *	INS		= inertial navigation system
 *	AHRS	= attitude heading reference system
 *	IMU		= inertial measurement unit: gyros (rad/s), accelerometers (m/s^2)
 *	ECEF	= earth-centered earth fixed: x,y,z or vx,vy,vz (m or m/s)
 *	LLA		= latitude, longitude, altitude (degrees,m)
 *	NED		= north, east, down (m or m/s)
 *	QE2B	= quaternion rotation from ECEF frame to local frame.
 *	QN2B	= quaternion rotation from NED frame to local frame.
 *	UVW		= velocities in local frame.
*/

// -------------------------------------------------------------------------------------------------------------------------------
// Inertial Sense simple communications interface --------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------------------
// The simple comm interface does not require any of the com manager APIs and is designed for simple or lightweight scenarios, tiny embedded platforms, etc.
// *****************************************************************************
// ****** Binary messages                                                 ******
// *****************************************************************************

/** INS/AHRS */
#define _DID_INS_ECEF_QE2B			DID_INS_4				/** (see ins_4_t) INS output: ECEF position (m) and velocity (m/s), quaternion from ECEF */
#define _DID_INS_LLA_EULER_NED		DID_INS_1				/** (see ins_1_t) INS/AHRS output: euler from NED, LLA (degrees,m), NED pos (m) and vel (m/s) from refLLA */
#define _DID_INS_LLA_QN2B			DID_INS_3				/** (see ins_3_t) INS/AHRS output: quaternion from NED, LLA (degrees,m) */

/** IMU */
#define _DID_IMU_DUAL				DID_DUAL_IMU			/** (see dual_imu_t) Dual IMU output: angular rate (rad/s) and linear acceleration (m/s^2) */
#define _DID_IMU_PREINTEGRATED_IMU	DID_PREINTEGRATED_IMU	/** (see preintegrated_imu_t) Dual IMU output: Conning and sculling integrated at IMU update rate. */	

/** GPS */
#define _DID_GPS1_POS				DID_GPS1_POS			/** (see gps_pos_t) GPS output */

/** Magnetometer, Barometer, and other Sensor */
#define _DID_MAG_CAL				DID_MAG_CAL				/** (see mag_cal_t) Magnetometer calibration */
#define _DID_MAGNETOMETER_1			DID_MAGNETOMETER_1		/** (see magnetometer_t) Magnetometer sensor 1 output */
#define _DID_MAGNETOMETER_2			DID_MAGNETOMETER_2		/** (see magnetometer_t) Magnetometer sensor 2 output */
#define _DID_BAROMETER				DID_BAROMETER			/** (see barometer_t) Barometric pressure sensor data */
#define _DID_WHEEL_ENCODER			DID_WHEEL_ENCODER		/** (see wheel_encoder_t) Wheel encoder sensor data */
#define _DID_POS_MEASUREMENT		DID_POSITION_MEASUREMENT/** (see pos_measurement_t) Position Measurement data*/

/** Utilities */
#define _DID_DEV_INFO				DID_DEV_INFO			/** (see dev_info_t) Device information */
#define _DID_BIT					DID_BIT					/** (see bit_t) System built-in self-test */
#define _DID_STROBE_IN_TIME			DID_STROBE_IN_TIME		/** (see strobe_in_time_t) Timestamp for input strobe */

/** Configuration */
#define _DID_FLASH_CONFIG			DID_FLASH_CONFIG 		/** (see nvm_flash_cfg_t) Flash memory configuration */
#define _DID_RMC					DID_RMC					/** (see rmc_t) Realtime message controller */

/** Protocol Type */
typedef enum
{
	_PTYPE_NONE                 = 0,						/** No complete valid data available yet */
	_PTYPE_PARSE_ERROR          = 0xFFFFFFFF,				/** Invalid data or checksum error */
	_PTYPE_INERTIAL_SENSE_DATA  = 0xEFFFFFFF,				/** Protocol Type: Inertial Sense binary data */
	_PTYPE_INERTIAL_SENSE_CMD   = 0xDFFFFFFF,				/** Protocol Type: Inertial Sense binary command */
	_PTYPE_INERTIAL_SENSE_ACK   = 0xCFFFFFFF,				/** Protocol Type: Inertial Sense binary acknowledge (ack) or negative acknowledge (nack)  */
	_PTYPE_ASCII_NMEA           = 0xBFFFFFFF,				/** Protocol Type: ASCII NMEA (National Marine Electronics Association) */
	_PTYPE_UBLOX                = 0xAFFFFFFF,				/** Protocol Type: uBlox binary */
	_PTYPE_RTCM3                = 0x9FFFFFFF,				/** Protocol Type: RTCM3 binary (Radio Technical Commission for Maritime Services) */
} protocol_type_t;

/** uINS default baud rate */
#define IS_COM_BAUDRATE_DEFAULT IS_BAUDRATE_921600

/** The maximum allowable dataset size */
#define MAX_DATASET_SIZE        1024

/** The decoded overhead involved in sending a packet - 4 bytes for header, 4 bytes for footer */
#define PKT_OVERHEAD_SIZE       8       // = START_BYTE + INFO_BYTE + COUNTER_BYTE + FLAGS_BYTE + CHECKSUM_BYTE_1 + CHECKSUM_BYTE_2 + CHECKSUM_BYTE_3 + END_BYTE

/** The maximum buffer space that is used for sending and receiving packets */
#ifndef PKT_BUF_SIZE
#define PKT_BUF_SIZE            2048
#endif

/** The maximum encoded overhead size in sending a packet (7 bytes for header, 7 bytes for footer). The packet start and end bytes are never encoded. */
#define MAX_PKT_OVERHEAD_SIZE   (PKT_OVERHEAD_SIZE + PKT_OVERHEAD_SIZE - 2)  // worst case for packet encoding header / footer

/** The maximum size of an decoded packet body */
#define MAX_PKT_BODY_SIZE       (((PKT_BUF_SIZE - MAX_PKT_OVERHEAD_SIZE) / 2) & 0xFFFFFFFE) // worst case for packet encoding body, rounded down to even number

/** The maximum size of decoded data in a packet body */
#define MAX_P_DATA_BODY_SIZE    (MAX_PKT_BODY_SIZE-sizeof(p_data_hdr_t))    // Data size limit

/** The maximum size of a decoded ACK message */
#define MAX_P_ACK_BODY_SIZE     (MAX_PKT_BODY_SIZE-sizeof(p_ack_hdr_t))     // Ack data size

/** Binary checksum start value */
#define CHECKSUM_SEED 0x00AAAAAA

/** Defines the 4 parts to the communications version. Major changes involve changes to the com manager. Minor changes involve additions to data structures */

// Major (in com_manager.h)
#define PROTOCOL_VERSION_CHAR0			(1)

// version 1: initial release
// version 2: 24 bit checksum support
#define PROTOCOL_VERSION_CHAR1			(2)

// Minor (in data_sets.h)
// #define PROTOCOL_VERSION_CHAR2		0
// #define PROTOCOL_VERSION_CHAR3		0

#define UBLOX_HEADER_SIZE 6
#define RTCM3_HEADER_SIZE 3

/** We must not allow any packing or shifting as these data structures must match exactly in memory on all devices */
PUSH_PACK_1

/** Valid baud rates for Inertial Sense hardware */
typedef enum
{
	IS_BAUDRATE_19200 = 19200,
	IS_BAUDRATE_38400 = 38400,
	IS_BAUDRATE_57600 = 57600,
	IS_BAUDRATE_115200 = 115200,
	IS_BAUDRATE_230400 = 230400,
	IS_BAUDRATE_460800 = 460800,
	IS_BAUDRATE_921600 = 921600,
	IS_BAUDRATE_3000000 = 3000000,

	IS_BAUDRATE_COUNT = 8
} baud_rate_t;

/** List of valid baud rates */
extern const unsigned int g_validBaudRates[IS_BAUDRATE_COUNT];

/*
Packet Overview

Byte
0			Packet start byte
1			Packet indo: ID (mask 0x1F) | reserved bits (mask 0xE)
2			Packet counter (for ACK and retry)
3			Packet flags

// packet body, may or may not exist depending on packet id - packet body is made up of 4 byte or 8 byte values.
4-7			Data identifier
8-11		Data length
12-15		Data offset
16-19		Data start
(n-8)-(n-5)	Last piece of data
// end data

n-4			Reserved
n-3			Checksum high byte
n-2			Checksum low byte
n-1			Packet end byte
*/

// Packet IDs	
typedef uint32_t ePacketIDs;

#define PID_INVALID                         (ePacketIDs)0   /** Invalid packet id */
#define PID_ACK                             (ePacketIDs)1   /** (ACK) received valid packet */
#define PID_NACK                            (ePacketIDs)2   /** (NACK) received invalid packet */
#define PID_GET_DATA                        (ePacketIDs)3   /** Request for data to be broadcast, response is PID_DATA. See data structures for list of possible broadcast data. */
#define PID_DATA                            (ePacketIDs)4   /** Data sent in response to PID_GET_DATA (no PID_ACK is sent) */
#define PID_SET_DATA                        (ePacketIDs)5   /** Data sent, such as configuration options.  PID_ACK is sent in response. */
#define PID_STOP_BROADCASTS_ALL_PORTS       (ePacketIDs)6   /** Stop all data broadcasts on all ports. Responds with an ACK */
#define PID_STOP_DID_BROADCAST              (ePacketIDs)7   /** Stop a specific broadcast */
#define PID_STOP_BROADCASTS_CURRENT_PORT    (ePacketIDs)8   /** Stop all data broadcasts on current port. Responds with an ACK */
#define PID_COUNT                           (ePacketIDs)9   /** The number of packet identifiers, keep this at the end! */
#define PID_MAX_COUNT                       (ePacketIDs)32  /** The maximum count of packet identifiers, 0x1F (PACKET_INFO_ID_MASK) */

/** Represents size number of bytes in memory, up to a maximum of PKT_BUF_SIZE */
typedef struct
{
	/** Number of bytes - for partial data requests, this will be less than the size of the data structure */
	uint32_t            size;

	/** Buffer to hold the bytes */
	uint8_t             buf[PKT_BUF_SIZE];
} buffer_t;

/** Represents size number of bytes in memory, pointing to a BYTE pointer that is owned elsewhere */
typedef struct
{
	/** External bytes owned elsewhere */
	uint8_t             *ptr;

	/** Number of bytes in ptr */
	uint32_t            size;
} bufPtr_t;

/** Represents both a send and receive buffer */
typedef struct
{
	/** send buffer */
	uint8_t             *txPtr;

	/** receive buffer */
	uint8_t             *rxPtr;

	/** size of both buffers */
	uint32_t            size;
} bufTxRxPtr_t;

/** Types of values allowed in ASCII data */
typedef enum
{
	/** 32 bit integer */
	asciiTypeInt = 0,

	/** 32 bit unsigned integer */
	asciiTypeUInt = 1,

	/** 32 bit floating point */
	asciiTypeFloat = 2,

	/** 64 bit floating point */
	asciiTypeDouble = 3
} asciiDataType;

/** create a uint from an ASCII message id that is the same, regardless of CPU architecture */
#define ASCII_MESSAGEID_TO_UINT(c4) ((uint32_t)(c4)[0] << 24 | ((uint32_t)(c4)[1] << 16) | ((uint32_t)(c4)[2] << 8) | ((uint32_t)(c4)[3]))

enum ePktHdrFlags
{
	// bit set for little endian, bit cleared for big endian
	CM_PKT_FLAGS_LITTLE_ENDIAN = 0x01,
	CM_PKT_FLAGS_ENDIANNESS_MASK = 0x01,

	// has any valid packet been received
	CM_PKT_FLAGS_RX_VALID_DATA = 0x02,

	// multi-packet data set
	CM_PKT_FLAGS_MORE_DATA_AVAILABLE = 0x04,

	// Allow for arbitrary length in bytes of data, not necessarily multiple of 4. Don't auto-swap bytes for endianness
	CM_PKT_FLAGS_RAW_DATA_NO_SWAP = 0x08,

	// Checksum is the new 24 bit algorithm instead of the old 16 bit algorithm
	CM_PKT_FLAGS_CHECKSUM_24_BIT = 0x10
};

/**
Built in special bytes that will need to be encoded in the binary packet format. This is not an exhaustive list, as other bytes such as ublox and rtcm preambles
will be encoded as well, but these messages are not parsed and handled in the com manager, rather they are forwarded via the pass through handler.
A byte is encoded by writing a 0xFD byte (encoded byte marker), followed by the encoded byte, which is created by inverting all the bits of the original byte.
These bytes are not encoded when written in the proper spot in the packet (i.e. when writing the first byte for a binary packet, the 0xFF byte, no encoding
is performed).
*/
enum ePktSpecialChars
{
	/** Dollar sign ($), used by ASCII protocol to signify start of message (36) */
	PSC_ASCII_START_BYTE = 0x24,

	/** New line (\n), used by ASCII protocol to signify end of message (10) */
	PSC_ASCII_END_BYTE = 0x0A,

	/** Binary packet start byte, must only exist at the very start of a binary packet and no where else (255) */
	PSC_START_BYTE = 0xFF,

	/** Binary packet end byte, must only exist at the end of a binary packet and no where else (254) */
	PSC_END_BYTE = 0xFE,

	/** Encoded byte marker, must only be used to prefix encoded bytes (253) */
	PSC_RESERVED_KEY = 0xFD,

	/** Ublox start byte 1 (181) */
	UBLOX_START_BYTE1 = 0xB5,

	/** Ublox start byte 2 (98) */
	UBLOX_START_BYTE2 = 0x62,

	/** Rtcm3 start byte (211) */
	RTCM3_START_BYTE = 0xD3
};

/** Represents an ASCII message and how it is mapped to a structure in memory */
typedef struct
{
	/** the message, always 4 characters long */
	unsigned char messageId[4];

	/** the ptr of the start of the struct to modify */
	uint8_t* ptr;

	/** the total size of the structure that ptr points to */
	int ptrSize;

	/** field count - the number of items in fieldsAndOffsets */
	int fieldCount;

	/** an array of 1 byte asciiDataType and 1 byte offset (shifted << 8) */
	uint16_t* fieldsAndOffsets;
} asciiMessageMap_t;

/** Represents the 4 bytes that begin each binary packet */
typedef struct
{
	/** Packet start byte, always 0xFF */
	uint8_t             startByte;

	/** Packet identifier (see ePacketIDs) */
	uint8_t             pid;

	/** Packet counter, for ACK and retry */
	uint8_t             counter;

	/**
	Packet flags (see ePktHdrFlags)
	Bit 0 : unset means big endian, set means little endian format
	Bit 1 : unset means no valid communication received yet, set means valid communication received
	Bit 2 : unset means no more related packets available, set means additional packet(s) available related to this packet
	Bit 3 : unset means perform swap, set means do not swap
	*/
	uint8_t             flags;
} packet_hdr_t;

/** Represents the 4 bytes that end each binary packet */
typedef struct
{
	/** Checksum byte 3 */
	uint8_t             cksum3;

	/** Checksum byte 2 */
	uint8_t             cksum2;

	/** Checksum byte 1 */
	uint8_t             cksum1;

	/** Packet end byte, always 0xFE */
	uint8_t             stopByte;
} packet_ftr_t;

/** Represents a packet header and body */
typedef struct
{
	/** Packet header */
	packet_hdr_t        hdr;

	/** Packet body */
	bufPtr_t            body;
} packet_t;

/** Represents a packet header, packet body and a buffer with data to send */
typedef struct
{
	packet_hdr_t        hdr;                    // Packet header
	bufPtr_t            bodyHdr;                // Body header
	bufPtr_t            txData;                 // Pointer and size of data to send
} pkt_info_t;

/** Specifies the data id, size and offset of a PID_DATA and PID_DATA_SET packet */
typedef struct
{
	/** Data identifier (see eDataIDs) */
	uint32_t            id;

	/** Size of data, for partial requests this will be less than the size of the data structure */
	uint32_t            size;

	/** Offset into data structure */
	uint32_t            offset;
} p_data_hdr_t;

/** Represents the complete packet body of a PID_DATA and PID_DATA_SET packet */
typedef struct
{
	/** Header with id, size and offset */
	p_data_hdr_t        hdr;

	/** Data */
	uint8_t             buf[MAX_DATASET_SIZE];
} p_data_t, p_data_set_t;

/** Represents the complete body of a PID_DATA_GET packet */
typedef struct
{
	/** ID of data being requested */
	uint32_t            id;

	/** Byte length of data from offset */
	uint32_t            size;

	/** Byte offset into data */
	uint32_t            offset;

	/**
	The broadcast source period multiples, or 0 for a one-time broadcast. Depending on data size and baud/transfer rates,
	some data may be dropped if this period is too short.
	*/
	uint32_t            bc_period_multiple;
} p_data_get_t;

/** Represents the body of a disable broadcast for data id packet */
typedef struct
{
	/** The packet identifier to disable broadcasts for */
	uint32_t            id;
} p_data_disable_t;

/** Represents the body header of an ACK or NACK packet */
typedef struct
{
	/** Packet info of the received packet */
	uint32_t            pktInfo;

	/** Packet counter of the received packet */
	uint32_t            pktCounter;
} p_ack_hdr_t;

/** Represents the entire body of an ACK or NACK packet */
typedef struct
{
	/** Body header */
	p_ack_hdr_t         hdr;

	/** Body buffer */
	union 
	{
		uint8_t         buf[sizeof(p_data_hdr_t)];
		p_data_hdr_t	dataHdr;
	}					body;
} p_ack_t, p_nack_t;

typedef struct
{
	/** Start of available buffer */
	uint8_t* start;

	/** End of available buffer */
	uint8_t* end;

	/** Size of buffer */
	uint32_t size;

	/** Start of data in buffer */
	uint8_t* head;

	/** End of data in buffer */
	uint8_t* tail;

	/** Search pointer in data (head <= scan <= tail) */
	uint8_t* scan;

} is_comm_buffer_t;

typedef struct  
{
	/** Enable protocol parsing: Inertial Sense binary */
	uint8_t enableISB;

	/** Enable protocol parsing: ASCII NMEA */
	uint8_t enableASCII;

	/** Enable protocol parsing: ublox */
	uint8_t enableUblox;

	/** Enable protocol parsing: RTCM3 */
	uint8_t enableRTCM3;
} is_comm_config_t;

/** An instance of an is_comm interface.  Do not modify these values. */
typedef struct
{
	/**
	The buffer to use for communications send and receive - this buffer should be large enough to handle the largest data structure you expect * 2 + 32 for worst case packet encoding
	A minimum of 128 is recommended. Set once before calling init.
	*/		
	is_comm_buffer_t buf;
	
	/** Enable/disable protocol parsing */
	is_comm_config_t config;

	/** Number of packets written */
	uint32_t txPktCount;

	/** Communications error counter */
	uint32_t rxErrorCount;

	/** Start byte */
	uint32_t hasStartByte;

	/** Used to validate ublox, RTCM, and ASCII packets */
	int32_t parseState;

	/** Data identifier (DID), size and offset */
	p_data_hdr_t dataHdr;

	/** Data pointer to start of valid data set */
	uint8_t* dataPtr;

	/** Packet pointer to start of valid packet */
	uint8_t* pktPtr;
	
	/** Alternate buffer location to decode packets.  This buffer must be PKT_BUF_SIZE in size.  NULL value will caused packet decode to occurr at head of is_comm_instance_t.buf.  Using an alternate buffer will preserve the original packet (as used in EVB-2 com_bridge).  */
	uint8_t* altDecodeBuf;

	/** Acknowledge packet needed in response to the last packet received */
	uint32_t ackNeeded;

	/** IS binary packet */
	packet_t pkt;

} is_comm_instance_t;

/** Pop off the packing argument, we can safely allow packing and shifting in memory at this point */
POP_PACK

/**
* Init simple communications interface - call this before doing anything else
* @param instance communications instance, please ensure that you have set the buffer and bufferSize
*/
void is_comm_init(is_comm_instance_t* instance, uint8_t *buffer, int bufferSize);

/**
* Decode packet data - when data is available, return value will be the protocol type (see protocol_type_t) and the comm instance dataPtr will point to the start of the valid data.  For Inertial Sense binary protocol, comm instance dataHdr contains the data ID (DID), size, and offset.
* @param instance the comm instance passed to is_comm_init
* @param byte the byte to decode
* @return protocol type when complete valid data is found, otherwise _PTYPE_NONE (0) (see protocol_type_t)
* @remarks when data is available, you can cast the comm instance dataPtr into the appropriate data structure pointer (see binary messages above and data_sets.h)
  For example usage, see comManagerStepRxInstance() in com_manager.c.

	// Read one byte (simple method)
	uint8_t c;
	protocol_type_t ptype;
	// Read from serial buffer until empty
	while (mySerialPortRead(&c, 1))
	{
		if ((ptype = is_comm_parse_byte(comm, c)) != _PTYPE_NONE)
		{
			switch (ptype)
			{
			case _PTYPE_INERTIAL_SENSE_DATA:
			case _PTYPE_INERTIAL_SENSE_CMD:
			case _PTYPE_INERTIAL_SENSE_ACK:
				break;
			case _PTYPE_UBLOX:
				break;
			case _PTYPE_RTCM3:
				break;
			case _PTYPE_ASCII_NMEA:
				break;
			}
		}
	}
*/
protocol_type_t is_comm_parse_byte(is_comm_instance_t* instance, uint8_t byte);

/**
* Decode packet data - when data is available, return value will be the protocol type (see protocol_type_t) and the comm instance dataPtr will point to the start of the valid data.  For Inertial Sense binary protocol, comm instance dataHdr contains the data ID (DID), size, and offset.
* @param instance the comm instance passed to is_comm_init
* @return protocol type when complete valid data is found, otherwise _PTYPE_NONE (0) (see protocol_type_t)
* @remarks when data is available, you can cast the comm instance dataPtr into the appropriate data structure pointer (see binary messages above and data_sets.h)
  For example usage, see comManagerStepRxInstance() in com_manager.c.

	// Read a set of bytes (fast method)
	protocol_type_t ptype;

	// Get available size of comm buffer
	int n = is_comm_free(comm);

	// Read data directly into comm buffer
	if ((n = mySerialPortRead(comm->buf.tail, n)))
	{
		// Update comm buffer tail pointer
		comm->buf.tail += n;

		// Search comm buffer for valid packets
		while ((ptype = is_comm_parse(comm)) != _PTYPE_NONE)
		{
			switch (ptype)
			{
			case _PTYPE_INERTIAL_SENSE_DATA:
			case _PTYPE_INERTIAL_SENSE_CMD:
			case _PTYPE_INERTIAL_SENSE_ACK:
				break;
			case _PTYPE_UBLOX:
				break;
			case _PTYPE_RTCM3:
				break;
			case _PTYPE_ASCII_NMEA:
				break;
			}
		}
	}
*/
protocol_type_t is_comm_parse(is_comm_instance_t* instance);

/**
* Removed old data and shift unparsed data to the the buffer start if running out of space at the buffer end.  Returns number of bytes available in the bufer.
* @param instance the comm instance passed to is_comm_init
* @return the number of bytes available in the comm buffer 
*/
int is_comm_free(is_comm_instance_t* instance);

/**
* Encode a binary packet to get data from the device - puts the data ready to send into the buffer passed into is_comm_init
* @param instance the comm instance passed to is_comm_init
* @param dataId the data id to request (see DID_* at top of this file)
* @param offset the offset into data to request. Set offset and length to 0 for entire data structure.
* @param length the length into data from offset to request. Set offset and length to 0 for entire data structure.
* @param periodMultiple how often you want the data to stream out, 0 for a one time message and turn off.
* @return the number of bytes written to the comm buffer (from is_comm_init), will be less than 1 if error
* @remarks pass an offset and length of 0 to request the entire data structure
*/
int is_comm_get_data(is_comm_instance_t* instance, uint32_t dataId, uint32_t offset, uint32_t size, uint32_t periodMultiple);

/**
* Encode a binary packet to get predefined list of data sets from the device - puts the data ready to send into the buffer passed into is_comm_init
* @param instance the comm instance passed to is_comm_init
* @param RMC bits specifying data messages to stream.  See presets: RMC_PRESET_PPD_BITS = post processing data, RMC_PRESET_INS_BITS = INS2 and GPS data at full rate
* @return the number of bytes written to the comm buffer (from is_comm_init), will be less than 1 if error
* @remarks pass an offset and length of 0 to request the entire data structure
*/
int is_comm_get_data_rmc(is_comm_instance_t* instance, uint64_t rmcBits);

/**
* Encode a binary packet to set data on the device - puts the data ready to send into the buffer passed into is_comm_init.  An acknowledge packet is sent in response to this packet.
* @param instance the comm instance passed to is_comm_init
* @param dataId the data id to set on the device (see DID_* at top of this file)
* @param offset the offset to start setting data at on the data structure on the device
* @param size the number of bytes to set on the data structure on the device
* @param data the actual data to change on the data structure on the device - this should have at least size bytes available
* @return the number of bytes written to the comm buffer (from is_comm_init), will be less than 1 if error
* @remarks pass an offset and length of 0 to set the entire data structure, in which case data needs to have the full number of bytes available for the appropriate struct matching the dataId parameter.
*/
int is_comm_set_data(is_comm_instance_t* instance, uint32_t dataId, uint32_t offset, uint32_t size, void* data);

/**
* Same as is_comm_set_data() except NO acknowledge packet is sent in response to this packet.
*/
int is_comm_data(is_comm_instance_t* instance, uint32_t dataId, uint32_t offset, uint32_t size, void* data);

/**
* Encode a binary packet to stop all messages being broadcast on the device on all ports - puts the data ready to send into the buffer passed into is_comm_init
* @param instance the comm instance passed to is_comm_init
* @return 0 if success, otherwise an error code
*/
int is_comm_stop_broadcasts_all_ports(is_comm_instance_t* instance);

/**
* Encode a binary packet to stop all messages being broadcast on the device on this port - puts the data ready to send into the buffer passed into is_comm_init
* @param instance the comm instance passed to is_comm_init
* @return 0 if success, otherwise an error code
*/
int is_comm_stop_broadcasts_current_port(is_comm_instance_t* instance);


// -------------------------------------------------------------------------------------------------------------------------------
// Common packet encode / decode functions
// -------------------------------------------------------------------------------------------------------------------------------
// common encode / decode for com manager and simple interface
int is_encode_binary_packet(void* srcBuffer, unsigned int srcBufferLength, packet_hdr_t* hdr, uint8_t additionalPktFlags, void* encodedPacket, int encodedPacketLength);
int is_decode_binary_packet(packet_t *pkt, unsigned char* pbuf, int pbufSize);
int is_decode_binary_packet_byte(uint8_t** _ptrSrc, uint8_t** _ptrDest, uint32_t* checksum, uint32_t shift);
void is_decode_binary_packet_footer(packet_ftr_t* ftr, uint8_t* ptrSrc, uint8_t** ptrSrcEnd, uint32_t* checksum);
void is_enable_packet_encoding(int enabled); // default is enabled

unsigned int calculate24BitCRCQ(unsigned char* buffer, unsigned int len);
unsigned int getBitsAsUInt32(const unsigned char* buffer, unsigned int pos, unsigned int len);

/** Copies packet data into a data structure.  Returns 0 on success, -1 on failure. */
char copyDataPToStructP(void *sptr, const p_data_t *data, const unsigned int maxsize);

/** Copies packet data into a data structure.  Returns 0 on success, -1 on failure. */
char copyDataPToStructP2(void *sptr, const p_data_hdr_t *dataHdr, const uint8_t *dataBuf, const unsigned int maxsize);

/** Copies is_comm_instance data into a data structure.  Returns 0 on success, -1 on failure. */
char is_comm_copy_to_struct(void *sptr, const is_comm_instance_t *com, const unsigned int maxsize);

#ifdef __cplusplus
}
#endif

#endif // IS_SIMPLE_INTERFACE_H
