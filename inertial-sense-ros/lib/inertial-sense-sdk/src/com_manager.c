/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "com_manager.h"
#include <string.h>
#include <stdlib.h>


// enable filtering of duplicate packets
#define ENABLE_FILTER_DUPLICATE_PACKETS 1

// whether the first character or all characters are checked in duplicate packets
#define ENABLE_FILTER_DUPLICATE_PACKETS_MATCH_ALL_CHARACTERS 0

#define PARSE_DOUBLE(str) strtod(str, 0)
#define PARSE_FLOAT(str) strtof(str, 0)

#define MIN_REQUEST_PERIOD_MS       1               // (ms) 1 KHz
#define MAX_REQUEST_PERIOD_MS       100000          // (ms)
#define MSG_PERIOD_SEND_ONCE		-1
#define MSG_PERIOD_DISABLED			0

static com_manager_t g_cm;

int initComManagerInstanceInternal
(
	com_manager_t* cmInstance,
	int numHandles,
	int maxEnsuredPackets,
	int stepPeriodMilliseconds,
	int retryCount,
	pfnComManagerRead readFnc,
	pfnComManagerSend sendFnc,
	pfnComManagerSendBufferAvailableBytes txFreeFnc,
	pfnComManagerPostRead pstRxFnc,
	pfnComManagerPostAck pstAckFnc,
	pfnComManagerDisableBroadcasts disableBcastFnc,
	com_manager_init_t *buffers,
	com_manager_port_t *cmPorts
);
// int processAsciiRxPacket(com_manager_t* cmInstance, int pHandle, unsigned char* start, int count);
// void parseAsciiPacket(com_manager_t* cmInstance, int pHandle, unsigned char* buf, int count);
int processBinaryRxPacket(com_manager_t* cmInstance, int pHandle, packet_t *pkt);
void enableBroadcastMsg(com_manager_t* cmInstance, broadcast_msg_t *msg, int periodMultiple);
void disableBroadcastMsg(com_manager_t* cmInstance, broadcast_msg_t *msg);
void disableDidBroadcast(com_manager_t* cmInstance, int pHandle, p_data_disable_t *disable);
int sendPacket(com_manager_t* cmInstance, int pHandle, packet_t *dPkt, uint8_t additionalPktFlags);
int sendDataPacket(com_manager_t* cmInstance, int pHandle, pkt_info_t *msg);
void sendAck(com_manager_t* cmInstance, int pHandle, packet_t *pkt, unsigned char pid_ack);
int findAsciiMessage(const void * a, const void * b);

//  Packet processing
// com manager only...
int encodeBinaryPacket(com_manager_t* cmInstance, int pHandle, buffer_t *pkt, packet_t *dPkt, uint8_t additionalPktFlags);
// 1 if valid
int asciiMessageCompare(const void* elem1, const void* elem2);

//  Packet Retry
void stepPacketRetry(com_manager_t* cmInstance);
packet_t* registerPacketRetry(com_manager_t* cmInstance, int pHandle, uint8_t pid, unsigned char data[], unsigned int dataSize);
void updatePacketRetryData(com_manager_t* cmInstance, packet_t *pkt);
void updatePacketRetryAck(com_manager_t* cmInstance, packet_t *pkt);

void stepComManagerSendMessages(void);
void stepComManagerSendMessagesInstance(CMHANDLE cmInstance);

CMHANDLE comManagerGetGlobal(void) { return &g_cm; }

int comManagerInit
(
	int numHandles,
	int maxEnsuredPackets,
	int stepPeriodMilliseconds,
	int retryCount,
	pfnComManagerRead readFnc,
	pfnComManagerSend sendFnc,
	pfnComManagerSendBufferAvailableBytes txFreeFnc,
	pfnComManagerPostRead pstRxFnc,
	pfnComManagerPostAck pstAckFnc,
	pfnComManagerDisableBroadcasts disableBcastFnc,
	com_manager_init_t *buffers,
	com_manager_port_t *cmPorts
)
{
	return initComManagerInstanceInternal(
		&g_cm, 
		numHandles, 
		maxEnsuredPackets, 
		stepPeriodMilliseconds, 
		retryCount, 
		readFnc, 
		sendFnc, 
		txFreeFnc, 
		pstRxFnc, 
		pstAckFnc, 
		disableBcastFnc, 
		buffers,
		cmPorts);
}

int comManagerInitInstance
(
	CMHANDLE cmHandle,
	int numHandles,
	int maxEnsuredPackets,
	int stepPeriodMilliseconds,
	int retryCount,
	pfnComManagerRead readFnc,
	pfnComManagerSend sendFnc,
	pfnComManagerSendBufferAvailableBytes txFreeFnc,
	pfnComManagerPostRead pstRxFnc,
	pfnComManagerPostAck pstAckFnc,
	pfnComManagerDisableBroadcasts disableBcastFnc,
	com_manager_init_t *buffers,
	com_manager_port_t *cmPorts
)
{
	int result = 0;

	com_manager_t* cmInstance = (com_manager_t*)cmHandle;
	if (cmInstance != 0)
	{
		memset(cmInstance, 0, sizeof(com_manager_t));
		result = initComManagerInstanceInternal(
			cmInstance, 
			numHandles, 
			maxEnsuredPackets, 
			stepPeriodMilliseconds, 
			retryCount, 
			readFnc, 
			sendFnc, 
			txFreeFnc, 
			pstRxFnc, 
			pstAckFnc, 
			disableBcastFnc,
			buffers, 
			cmPorts);
	}
	return result;
}

int initComManagerInstanceInternal
(
	com_manager_t* cmInstance,
	int numHandles,
	int maxEnsuredPackets,
	int stepPeriodMilliseconds,
	int retryCount,
	pfnComManagerRead readFnc,
	pfnComManagerSend sendFnc,
	pfnComManagerSendBufferAvailableBytes txFreeFnc,
	pfnComManagerPostRead pstRxFnc,
	pfnComManagerPostAck pstAckFnc,
	pfnComManagerDisableBroadcasts disableBcastFnc,
	com_manager_init_t *buffers,
	com_manager_port_t *cmPorts
)
{
	int32_t i;

	if (numHandles <= 0)
	{
		return -1;
	}
	numHandles = _CLAMP(numHandles, 1, 1024);

	// assign new variables
	cmInstance->maxEnsuredPackets = maxEnsuredPackets;
	cmInstance->readCallback = readFnc;
	cmInstance->sendPacketCallback = sendFnc;
	cmInstance->txFreeCallback = txFreeFnc;
	cmInstance->pstRxFnc = pstRxFnc;
	cmInstance->pstAckFnc = pstAckFnc;
	cmInstance->disableBcastFnc = disableBcastFnc;
	cmInstance->numHandles = numHandles;
	cmInstance->stepPeriodMilliseconds = stepPeriodMilliseconds;
	cmInstance->ensureRetryCount = retryCount;
	cmInstance->cmMsgHandlerAscii = NULL;
	cmInstance->cmMsgHandlerUblox = NULL;
	cmInstance->cmMsgHandlerRtcm3 = NULL;

	if (buffers == NULL)
	{
		return -1;
	}
	
	// Buffer: message broadcasts
	if (buffers->broadcastMsg == NULL || buffers->broadcastMsgSize < COM_MANAGER_BUF_SIZE_BCAST_MSG(MAX_NUM_BCAST_MSGS))
	{
		return -1;
	}
	cmInstance->broadcastMessages = (broadcast_msg_t*)buffers->broadcastMsg;
	memset(cmInstance->broadcastMessages, 0, buffers->broadcastMsgSize);
		
	// Port specific info
	cmInstance->ports = cmPorts;
	for (i = 0; i < numHandles; i++)
	{	// Initialize IScomm instance, for serial reads / writes
		com_manager_port_t *port = &(cmInstance->ports[i]);
		is_comm_init(&(port->comm), port->comm_buffer, MEMBERSIZE(com_manager_port_t, comm_buffer));
		
		// Port status
		memset(&(port->status), 0, MEMBERSIZE(com_manager_port_t,status));	
			
#if ENABLE_PACKET_CONTINUATION			
		// Packet data continuation
		memset(&(port->con), 0, MEMBERSIZE(com_manager_port_t,con));
#endif
	}

	// Buffer: ensured packets
	if (cmInstance->maxEnsuredPackets > 0)
	{
		if (buffers->ensuredPackets == NULL || buffers->ensuredPacketsSize < COM_MANAGER_BUF_SIZE_ENSURED_PKTS(cmInstance->maxEnsuredPackets))
		{
			return -1;
		}
		cmInstance->ensuredPackets = (ensured_pkt_t*)buffers->ensuredPackets;
		memset(cmInstance->ensuredPackets, 0, buffers->ensuredPacketsSize);
		for (i = 0; i < cmInstance->maxEnsuredPackets; i++)
		{
			cmInstance->ensuredPackets[i].counter = -2; // indicates no retries are enabled
			cmInstance->ensuredPackets[i].pkt.body.ptr = cmInstance->ensuredPackets[i].pktBody;
		}
	}

	return 0;
}

int asciiMessageCompare(const void* elem1, const void* elem2)
{
	asciiMessageMap_t* e1 = (asciiMessageMap_t*)elem1;
	asciiMessageMap_t* e2 = (asciiMessageMap_t*)elem2;

	return memcmp(e1->messageId, e2->messageId, 4);
}

void comManagerRegister(uint32_t dataId, pfnComManagerPreSend txFnc, pfnComManagerPostRead pstRxFnc, const void* txDataPtr, void* rxDataPtr, int dataSize, uint8_t pktFlags)
{
	comManagerRegisterInstance(&g_cm, dataId, txFnc, pstRxFnc, txDataPtr, rxDataPtr, dataSize, pktFlags);
}

void comManagerRegisterInstance(CMHANDLE cmInstance_, uint32_t dataId, pfnComManagerPreSend txFnc, pfnComManagerPostRead pstRxFnc, const void* txDataPtr, void* rxDataPtr, int dataSize, uint8_t pktFlags)
{
	com_manager_t* cmInstance = (com_manager_t*)cmInstance_;

	// Validate ID and data pointer
	if (dataId >= DID_COUNT)
	{
		return;
	}

	// Function called to update struct before data is sent
	cmInstance->regData[dataId].preTxFnc = txFnc;

	// Function called after data is received and struct is updated
	cmInstance->regData[dataId].pstRxFnc = pstRxFnc;

	// Pointer to data struct for Tx
	cmInstance->regData[dataId].dataSet.txPtr = (unsigned char*)txDataPtr;

	// Pointer to data struct for Rx
	cmInstance->regData[dataId].dataSet.rxPtr = (unsigned char*)rxDataPtr;

	// Size of data struct
	cmInstance->regData[dataId].dataSet.size = dataSize;
	
	// Packet flags
	cmInstance->regData[dataId].pktFlags = pktFlags;
}

void comManagerStep(void)
{
	comManagerStepRxInstance(&g_cm);
	comManagerStepTxInstance(&g_cm);
}

void comManagerStepInstance(CMHANDLE cmInstance_)
{
	com_manager_t* cmInstance = (com_manager_t*)cmInstance_;
	comManagerStepRxInstance(cmInstance);
	comManagerStepTxInstance(cmInstance);
}

// pfnISCommRead 
// static int commRead(int pHandle, uint8_t *buffer, int numberOfBytes)
// {
// 	
// } 

void comManagerStepRxInstance(CMHANDLE cmInstance_)
{
	com_manager_t* cmInstance = (com_manager_t*)cmInstance_;
	int32_t pHandle;
	
	if (!cmInstance->readCallback)
	{
		return;
	}
		
		
	for (pHandle = 0; pHandle < cmInstance->numHandles; pHandle++)
	{
		com_manager_port_t *port = &(cmInstance->ports[pHandle]);
		is_comm_instance_t *comm = &(port->comm);
		protocol_type_t ptype;

#if 0	// Read one byte (simple method)
		uint8_t c;

		// Read from serial buffer until empty
		while (cmInstance->readCallback(cmInstance, pHandle, &c, 1))
		{
			if ((ptype = is_comm_parse_byte(comm, c)) != _PTYPE_NONE)
			{

#else	// Read a set of bytes (fast method)

		// Get available size of comm buffer
		int n = is_comm_free(comm);

		// Read data directly into comm buffer
		if ((n = cmInstance->readCallback(cmInstance, pHandle, comm->buf.tail, n)))
		{
			// Update comm buffer tail pointer
			comm->buf.tail += n;

			// Search comm buffer for valid packets
			while ((ptype = is_comm_parse(comm)) != _PTYPE_NONE)
			{
#endif					
				uint8_t error = 0;
				uint8_t *dataPtr = comm->dataPtr + comm->dataHdr.offset;
				uint32_t dataSize = comm->dataHdr.size;

				switch (ptype)
				{
				case _PTYPE_PARSE_ERROR:
					error = 1;
					break;

				case _PTYPE_INERTIAL_SENSE_DATA:
				case _PTYPE_INERTIAL_SENSE_CMD:
					error = (uint8_t)processBinaryRxPacket(cmInstance, pHandle, &(comm->pkt));
					break;

				case _PTYPE_UBLOX:
					if (cmInstance->cmMsgHandlerUblox)
					{
						error = (uint8_t)cmInstance->cmMsgHandlerUblox(cmInstance, pHandle, dataPtr, dataSize);
					}
					break;

				case _PTYPE_RTCM3:
					if (cmInstance->cmMsgHandlerRtcm3)
					{
						error = (uint8_t)cmInstance->cmMsgHandlerRtcm3(cmInstance, pHandle, dataPtr, dataSize);
					}
					break;

				case _PTYPE_ASCII_NMEA:
					if (cmInstance->cmMsgHandlerAscii)
					{
						error = (uint8_t)cmInstance->cmMsgHandlerAscii(cmInstance, pHandle, dataPtr, dataSize);
					}
					break;
					
				default:
					break;
				}

				if (error)
				{	// Error parsing packet
					port->status.readCounter += 32;
					port->status.rxError = (uint32_t)-1;
					port->status.communicationErrorCount++;
				}
			}
		}
			
		if ((port->status.flags & CM_PKT_FLAGS_RX_VALID_DATA) && port->status.readCounter > 128)
		{	// communication problem, clear communication received bit
			port->status.flags &= (~CM_PKT_FLAGS_RX_VALID_DATA);
		}
	}
}

void comManagerStepTxInstance(CMHANDLE cmInstance_)
{
	com_manager_t* cmInstance = (com_manager_t*)cmInstance_;
	stepComManagerSendMessagesInstance(cmInstance);
}

void stepComManagerSendMessages(void)
{
	stepComManagerSendMessagesInstance(&g_cm);
}

void stepComManagerSendMessagesInstance(CMHANDLE cmInstance_)
{
	com_manager_t* cmInstance = cmInstance_;
	
	// Send data (if necessary)
	for (broadcast_msg_t* bcPtr = cmInstance->broadcastMessages, *ptrEnd = (cmInstance->broadcastMessages + MAX_NUM_BCAST_MSGS); bcPtr < ptrEnd; bcPtr++)
	{
		// If send buffer does not have space, exit out
		if (cmInstance->txFreeCallback && (bcPtr->pkt.txData.size > (uint32_t)cmInstance->txFreeCallback(cmInstance, bcPtr->pHandle)))
		{
			break;
		}
		// Send once and remove from message queue
		else if (bcPtr->period == MSG_PERIOD_SEND_ONCE)
		{
			sendDataPacket(cmInstance, bcPtr->pHandle, &(bcPtr->pkt));
			disableBroadcastMsg(cmInstance, bcPtr);
		}
		// Broadcast messages
		else if (bcPtr->period > 0)
		{
			// Check if counter has expired
			if (++bcPtr->counter >= bcPtr->period)
			{
				bcPtr->counter = 0;    // reset counter

				// Prep data if callback exists
				if (cmInstance->regData[bcPtr->dataHdr.id].preTxFnc)
				{
					cmInstance->regData[bcPtr->dataHdr.id].preTxFnc(cmInstance, bcPtr->pHandle);
				}
				sendDataPacket(cmInstance, bcPtr->pHandle, &(bcPtr->pkt));
			}
		}
	}

	// Resend data (if necessary)
	stepPacketRetry(cmInstance);
}

void comManagerSetCallbacks(
	pfnComManagerAsapMsg handlerRmc,
	pfnComManagerGenMsgHandler handlerAscii,
	pfnComManagerGenMsgHandler handlerUblox, 
	pfnComManagerGenMsgHandler handlerRtcm3)
{
	comManagerSetCallbacksInstance(&g_cm, handlerRmc, handlerAscii, handlerUblox, handlerRtcm3);
}

void comManagerSetCallbacksInstance(CMHANDLE cmInstance, 
	pfnComManagerAsapMsg handlerRmc,
	pfnComManagerGenMsgHandler handlerAscii,
	pfnComManagerGenMsgHandler handlerUblox,
	pfnComManagerGenMsgHandler handlerRtcm3)
{
	if (cmInstance != 0)
	{
		((com_manager_t*)cmInstance)->cmMsgHandlerRmc = handlerRmc;
		((com_manager_t*)cmInstance)->cmMsgHandlerAscii = handlerAscii;
		((com_manager_t*)cmInstance)->cmMsgHandlerUblox = handlerUblox;
		((com_manager_t*)cmInstance)->cmMsgHandlerRtcm3 = handlerRtcm3;
	}
}

void comManagerAssignUserPointer(CMHANDLE cmInstance, void* userPointer)
{
	((com_manager_t*)cmInstance)->userPointer = userPointer;
}

void* comManagerGetUserPointer(CMHANDLE cmInstance)
{
	return ((com_manager_t*)cmInstance)->userPointer;
}

com_manager_status_t* comManagerGetStatus(int pHandle)
{
	return comManagerGetStatusInstance(&g_cm, pHandle);
}

com_manager_status_t* comManagerGetStatusInstance(CMHANDLE cmInstance, int pHandle)
{
	com_manager_t *cm = (com_manager_t*)cmInstance;
	
	if(cm->numHandles <= 0 || pHandle < 0 || pHandle >= cm->numHandles)
	{
		return NULL;
	}
	
	return &(cm->ports[pHandle].status);
}

/**
*   @brief Request data
*   This function requests the specified data w/ offset and length
*   for partial reads.
*
*	@param[in] dataId       Data structure ID
*	@param[in] offset   Byte offset into data structure.  0 = data start.
*	@param[in] length   Byte length of data.  0 = entire structure.
*	@param[in] periodMultiple Broadcast period of requested data.  0 = single request.
*
*	@return 0 on successful request.  -1 on failure.
*/
void comManagerGetData(int pHandle, uint32_t dataId, int offset, int size, int periodMultiple)
{
	comManagerGetDataInstance(&g_cm, pHandle, dataId, offset, size, periodMultiple);
}

void comManagerGetDataInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId, int offset, int size, int periodMultiple)
{
	p_data_get_t request;
	bufPtr_t data;

	// Create and Send request packet
	request.id = dataId;
	request.offset = offset;
	request.size = size;
	request.bc_period_multiple = periodMultiple;

	data.ptr = (uint8_t*)&request;
	data.size = sizeof(request);
	comManagerSendInstance(cmInstance, pHandle, PID_GET_DATA, 0, &data, 0);

	// comManagerSendEnsured(pHandle, PID_GET_DATA, (unsigned char*)&request, sizeof(request));
}

void comManagerGetDataRmc(int pHandle, uint64_t rmcBits, uint32_t rmcOptions)
{
	comManagerGetDataRmcInstance(&g_cm, pHandle, rmcBits, rmcOptions);
}

void comManagerGetDataRmcInstance(CMHANDLE cmInstance, int pHandle, uint64_t rmcBits, uint32_t rmcOptions)
{
	rmc_t rmc;
	rmc.bits = rmcBits;
	rmc.options = rmcOptions;

	comManagerSendDataInstance(cmInstance, pHandle, DID_RMC, &rmc, sizeof(rmc_t), 0);
}

int comManagerSendData(int pHandle, uint32_t dataId, void *dataPtr, int dataSize, int dataOffset)
{
	return comManagerSendDataInstance(&g_cm, pHandle, dataId, dataPtr, dataSize, dataOffset);
}

int comManagerSendDataInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset)
{
	p_data_hdr_t hdr;
	bufPtr_t bodyHdr, data;

	// Data Header
	hdr.id = dataId;
	hdr.size = dataSize;
	hdr.offset = dataOffset;

	// Packet Body
	bodyHdr.ptr = (uint8_t*)&hdr;
	bodyHdr.size = sizeof(hdr);
	data.ptr = (uint8_t*)dataPtr;
	data.size = dataSize;

	return comManagerSendInstance(cmInstance, pHandle, PID_SET_DATA, &bodyHdr, &data, 0);
}

int comManagerSendDataNoAck(int pHandle, uint32_t dataId, void *dataPtr, int dataSize, int dataOffset)
{
	return comManagerSendDataNoAckInstance(&g_cm, pHandle, dataId, dataPtr, dataSize, dataOffset);
}

int comManagerSendDataNoAckInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset)
{
	p_data_hdr_t hdr;
	bufPtr_t bodyHdr, data;

	// Data Header
	hdr.id = dataId;
	hdr.size = dataSize;
	hdr.offset = dataOffset;

	// Packet Body
	bodyHdr.ptr = (uint8_t*)&hdr;
	bodyHdr.size = sizeof(hdr);
	data.ptr = (uint8_t*)dataPtr;
	data.size = dataSize;

	return comManagerSendInstance((com_manager_t*)cmInstance, pHandle, PID_DATA, &bodyHdr, &data, 0);
}

int comManagerSendRawData(int pHandle, uint32_t dataId, void *dataPtr, int dataSize, int dataOffset)
{
	return comManagerSendRawDataInstance(&g_cm, pHandle, dataId, dataPtr, dataSize, dataOffset);
}

int comManagerSendRawDataInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset)
{
	p_data_hdr_t hdr;
	bufPtr_t bodyHdr, data;

	// Data Header
	hdr.id = dataId;
	hdr.size = dataSize;
	hdr.offset = dataOffset;

	// Packet Body
	bodyHdr.ptr = (uint8_t*)&hdr;
	bodyHdr.size = sizeof(hdr);
	data.ptr = (uint8_t*)dataPtr;
	data.size = dataSize;

	return comManagerSendInstance((com_manager_t*)cmInstance, pHandle, PID_SET_DATA, &bodyHdr, &data, CM_PKT_FLAGS_RAW_DATA_NO_SWAP);
}

int comManagerDisableData(int pHandle, uint32_t dataId)
{
	return comManagerDisableDataInstance(&g_cm, pHandle, dataId);
}

int comManagerDisableDataInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId)
{
	bufPtr_t data;
	data.ptr  = (uint8_t*)&dataId;
	data.size = 4;

	return comManagerSendInstance(cmInstance, pHandle, PID_STOP_DID_BROADCAST, 0, &data, 0);
}

int comManagerSend(int pHandle, uint8_t pktInfo, bufPtr_t *bodyHdr, bufPtr_t *txData, uint8_t pFlags)
{
	return comManagerSendInstance(&g_cm, pHandle, pktInfo, bodyHdr, txData, pFlags);
}

int comManagerSendInstance(CMHANDLE cmInstance, int pHandle, uint8_t pktInfo, bufPtr_t* bodyHdr, bufPtr_t* txData, uint8_t pktFlags)
{
	pkt_info_t pkt = { 0 };

	// Create Packet String (start to end byte)
	pkt.hdr.startByte = PSC_START_BYTE;
	pkt.hdr.pid = pktInfo;
	pkt.hdr.flags = pktFlags;

	if (bodyHdr)
	{
		pkt.bodyHdr = *bodyHdr;
	}
	if (txData)
	{
		pkt.txData = *txData;
	}

	return sendDataPacket(cmInstance, pHandle, &pkt);
}

int comManagerSendEnsured(int pHandle, uint8_t pktInfo, unsigned char* data, unsigned int dataSize)
{
	return comManagerSendEnsuredInstance(&g_cm, pHandle, pktInfo, data, dataSize);
}

int comManagerSendEnsuredInstance(CMHANDLE cmInstance, int pHandle, uint8_t pktInfo, unsigned char *data, unsigned int dataSize)
{
	packet_t *pkt;

	// Change retry "Ensured" packets to so that we encode packets first (including pkt counter)
	// and then ensure they are delivered.  Include packet checksum in ACK/NACK to validate delivery.
	// Then, if all the ensured slots are occupied because of bad comm, either allow
	// to clear ensured packets or just block until they are delivered.  We must
	// ensure NACKs are used to clear blocking ensured packets.

	// Create Packet String (start to end byte)
	if ((pkt = registerPacketRetry((com_manager_t*)cmInstance, pHandle, pktInfo, data, dataSize)) == 0)
	{
		return -1;
	}

	return sendPacket((com_manager_t*)cmInstance, pHandle, pkt, 0);
}

int findAsciiMessage(const void * a, const void * b)
{
	unsigned char* a1 = (unsigned char*)a;
	asciiMessageMap_t* a2 = (asciiMessageMap_t*)b;

	return memcmp(a1, a2->messageId, 4);
}

/**
*   @brief Process binary packet content:
*
*	@return 0 on success.  -1 on failure.
*/
int processBinaryRxPacket(com_manager_t* cmInstance, int pHandle, packet_t *pkt)
{
	p_data_t			*data = (p_data_t*)(pkt->body.ptr);
	p_data_hdr_t		*dataHdr;
	registered_data_t	*regd;
	uint8_t		pid = (uint8_t)(pkt->hdr.pid);

	com_manager_port_t *port = &(cmInstance->ports[pHandle]);
	port->status.flags |= CM_PKT_FLAGS_RX_VALID_DATA; // communication received
	port->status.readCounter = 0;

	// Packet read success
	port->status.rxPktCount++;

	switch (pid)
	{
	default:    // Data ID Unknown
		return -1;

	case PID_SET_DATA:
	case PID_DATA:
		dataHdr = &(data->hdr);

		// Validate Data
		if (dataHdr->id >= DID_COUNT)
		{
			return -1;
		}

		regd = &(cmInstance->regData[dataHdr->id]);

		// Validate and constrain Rx data size to fit within local data struct
		if (regd->dataSet.size && (dataHdr->offset + dataHdr->size) > regd->dataSet.size)
		{
			// trim the size down so it fits
			int size = (int)(regd->dataSet.size - dataHdr->offset);
			if (size < 4)
			{
				// we are completely out of bounds, we cannot process this message at all
				// the minimum data struct size is 4 bytes
				return -1;
			}

			// Update Rx data size
			dataHdr->size = _MIN(dataHdr->size, (uint8_t)size);
		}

#if ENABLE_PACKET_CONTINUATION

		// Consolidate datasets that were broken-up across multiple packets
		p_data_t* con = &cmInstance->ports[pHandle].con;
		if (additionalDataAvailable || (con->hdr.size != 0 && con->hdr.id == dataHdr->id))
		{
			// New dataset
			if (con->hdr.id == 0 || con->hdr.size == 0 || con->hdr.id != dataHdr->id || con->hdr.size > dataHdr->offset)
			{
				// Reset data consolidation
				con->hdr.id = dataHdr->id;
				con->hdr.offset = dataHdr->offset;
				con->hdr.size = 0;
			}

			// Ensure data will fit in buffer
			if ((con->hdr.size + dataHdr->size) < sizeof(con->buf))
			{
				// Add data to buffer
				memcpy(con->buf + con->hdr.size, data->buf, dataHdr->size);
				con->hdr.size += dataHdr->size;
			}
			else
			{
				// buffer overflow
			}

			// Wait for end of data
			if (additionalDataAvailable)
			{
				return 0;
			}

			// Use consolidated data
			data = con;
		}
		
#else
	
// 		unsigned char additionalDataAvailable // function parameter removed 
// 		(void)additionalDataAvailable;

#endif

		// Write to data structure if it was registered
		if (regd->dataSet.rxPtr)
		{
			copyDataPToStructP(regd->dataSet.rxPtr, data, regd->dataSet.size);
		}

		// Call data specific callback after data has been written to
		if (regd->pstRxFnc)
		{
			regd->pstRxFnc(cmInstance, pHandle, data);
		}

		// Call general/global callback
		if (cmInstance->pstRxFnc)
		{
			cmInstance->pstRxFnc(cmInstance, pHandle, data);
		}

		// Remove retry from linked list if necessary
		updatePacketRetryData(cmInstance, pkt);

#if ENABLE_PACKET_CONTINUATION

		// Clear dataset consolidation
		con->hdr.id = con->hdr.size = con->hdr.offset = 0;

#endif

		// Reply w/ ACK for PID_SET_DATA
		if (pid == PID_SET_DATA)
		{
			sendAck(cmInstance, pHandle, pkt, PID_ACK);
		}
		break;

	case PID_GET_DATA:
		if (comManagerGetDataRequestInstance(cmInstance, pHandle, (p_data_get_t *)(data)))
		{
			sendAck(cmInstance, pHandle, pkt, PID_NACK);
		}
		break;

	case PID_STOP_BROADCASTS_ALL_PORTS:
		comManagerDisableBroadcastsInstance(cmInstance, -1);

		// Call disable broadcasts callback if exists
		if (cmInstance->disableBcastFnc)
		{
			cmInstance->disableBcastFnc(cmInstance, -1);
		}
		sendAck(cmInstance, pHandle, pkt, PID_ACK);
		break;

	case PID_STOP_BROADCASTS_CURRENT_PORT:
		comManagerDisableBroadcastsInstance(cmInstance, pHandle);

		// Call disable broadcasts callback if exists
		if (cmInstance->disableBcastFnc)
		{
			cmInstance->disableBcastFnc(cmInstance, pHandle);
		}
		sendAck(cmInstance, pHandle, pkt, PID_ACK);
		break;

	case PID_STOP_DID_BROADCAST:
		disableDidBroadcast(cmInstance, pHandle, (p_data_disable_t *)(data));
		break;

	case PID_NACK:
	case PID_ACK:
		// Remove retry from linked list if necessary
		updatePacketRetryAck(cmInstance, pkt);

		// Call general ack callback
		if (cmInstance->pstAckFnc)
		{
			cmInstance->pstAckFnc(cmInstance, pHandle, (p_ack_t*)(pkt->body.ptr), pid);
		}
		break;
	}

	return 0;
}

bufTxRxPtr_t* comManagerGetRegisteredDataInfo(uint32_t dataId)
{
	return comManagerGetRegisteredDataInfoInstance(&g_cm, dataId);
}

bufTxRxPtr_t* comManagerGetRegisteredDataInfoInstance(CMHANDLE _cmInstance, uint32_t dataId)
{
	if (dataId >= DID_COUNT)
	{
		return 0;
	}

	com_manager_t* cmInstance = (com_manager_t*)_cmInstance;
	return &cmInstance->regData[dataId].dataSet;
}

// 0 on success. -1 on failure.
int comManagerGetDataRequest(int pHandle, p_data_get_t* req)
{
	return comManagerGetDataRequestInstance(&g_cm, pHandle, req);
}

int comManagerGetDataRequestInstance(CMHANDLE _cmInstance, int pHandle, p_data_get_t* req)
{
	com_manager_t* cmInstance = (com_manager_t*)_cmInstance;
	broadcast_msg_t* msg = 0;

	// Validate the request
	if (req->id >= DID_COUNT)
	{
		// invalid data id
		return -1;
	}
	// Call RealtimeMessageController (RMC) handler
	else if (cmInstance->cmMsgHandlerRmc && (cmInstance->cmMsgHandlerRmc(cmInstance, pHandle, req) == 0))
	{
		// Don't allow comManager broadcasts for messages handled by RealtimeMessageController. 
		return 0;
	}
	// if size is 0 and offset is 0, set size to full data struct size
	else if (req->size == 0 && req->offset == 0 && req->id < _ARRAY_ELEMENT_COUNT(cmInstance->regData))
	{
		req->size = cmInstance->regData[req->id].dataSet.size;
	}
	
	// Copy reference to source data
	bufTxRxPtr_t* dataSetPtr = &cmInstance->regData[req->id].dataSet;

	// Abort if no data pointer is registered or offset + size is out of bounds
	if (dataSetPtr->txPtr == 0 || dataSetPtr->size == 0)
	{
		return -1;
	}
	else if (req->offset + req->size > dataSetPtr->size)
	{
		req->offset = 0;
		req->size = dataSetPtr->size;
	}

	// Search for matching message (i.e. matches pHandle, id, size, and offset)...
	for (broadcast_msg_t* bcPtr = cmInstance->broadcastMessages, *ptrEnd = (cmInstance->broadcastMessages + MAX_NUM_BCAST_MSGS); bcPtr < ptrEnd; bcPtr++)
	{
		if (bcPtr->pHandle == pHandle && bcPtr->dataHdr.id == req->id && bcPtr->dataHdr.size == req->size && bcPtr->dataHdr.offset == req->offset)
		{
			msg = bcPtr;
			break;
		}
	}

	// otherwise use the first available (period=0) message.
	if (msg == 0)
	{
		for (broadcast_msg_t* bcPtr = cmInstance->broadcastMessages, *ptrEnd = (cmInstance->broadcastMessages + MAX_NUM_BCAST_MSGS); bcPtr < ptrEnd; bcPtr++)
		{
			if (bcPtr->period <= MSG_PERIOD_DISABLED)
			{
				msg = bcPtr;
				break;
			}
		}

		if (msg == 0)
		{
			// use last slot, force overwrite
			msg = cmInstance->broadcastMessages + MAX_NUM_BCAST_MSGS - 1;
		}
	}

	// Port handle
	msg->pHandle = pHandle;

	// Packet parameters
	msg->pkt.hdr.startByte = PSC_START_BYTE;
	msg->pkt.hdr.pid = PID_DATA;

	// Data Header
	msg->dataHdr.id = req->id;
	msg->dataHdr.size = req->size;
	msg->dataHdr.offset = req->offset;
	msg->pkt.hdr.flags = cmInstance->regData[msg->dataHdr.id].pktFlags;
	msg->pkt.bodyHdr.ptr = (uint8_t *)&msg->dataHdr;
	msg->pkt.bodyHdr.size = sizeof(msg->dataHdr);
	msg->pkt.txData.size = req->size;
	msg->pkt.txData.ptr = cmInstance->regData[req->id].dataSet.txPtr + req->offset;

	// Prep data if callback exists
	if (cmInstance->regData[msg->dataHdr.id].preTxFnc)
	{
		cmInstance->regData[msg->dataHdr.id].preTxFnc(cmInstance, pHandle);
	}
	
	// Constrain request broadcast period if necessary
	if (req->bc_period_multiple != 0)
	{
		_LIMIT2(req->bc_period_multiple, MIN_REQUEST_PERIOD_MS, MAX_REQUEST_PERIOD_MS);
	}

	// Send data
	if (req->bc_period_multiple > 0)
	{
		// ***  Request Broadcast  ***
		// Send data immediately if possible
		if (cmInstance->txFreeCallback == 0 || msg->pkt.txData.size <= (uint32_t)cmInstance->txFreeCallback(cmInstance, pHandle))
		{
			sendDataPacket(cmInstance, pHandle, &(msg->pkt));
		}

		// Enable broadcast message
		enableBroadcastMsg(cmInstance, msg, req->bc_period_multiple);
	}
	else
	{
		// ***  Request Single  ***
		// Send data immediately if possible
		if (cmInstance->txFreeCallback == 0 || msg->pkt.txData.size <= (uint32_t)cmInstance->txFreeCallback(cmInstance, pHandle))
		{
			sendDataPacket(cmInstance, pHandle, &(msg->pkt));
			disableBroadcastMsg(cmInstance, msg);
		}
		else
		{
			// Won't fit in queue, so send it later
			enableBroadcastMsg(cmInstance, msg, req->bc_period_multiple);
		}
	}

	return 0;
}

void enableBroadcastMsg(com_manager_t* cmInstance, broadcast_msg_t* msg, int periodMultiple)
{
	// Update broadcast period
	if (periodMultiple > 0)
	{
		msg->period = periodMultiple / cmInstance->stepPeriodMilliseconds;
	}
	else
	{
		msg->period = MSG_PERIOD_SEND_ONCE;
	}
	msg->counter = -1;   // Keeps broadcast from sending for at least one period
}

void disableBroadcastMsg(com_manager_t* cmInstance, broadcast_msg_t *msg)
{
	(void)cmInstance;

	// Remove item from linked list
	msg->period = MSG_PERIOD_DISABLED;
}

void comManagerDisableBroadcasts(int pHandle)
{
	comManagerDisableBroadcastsInstance(&g_cm, pHandle);
}

void comManagerDisableBroadcastsInstance(CMHANDLE cmInstance_, int pHandle)
{
	com_manager_t* cmInstance = (com_manager_t*)cmInstance_;
	for (broadcast_msg_t* bcPtr = cmInstance->broadcastMessages, *ptrEnd = (cmInstance->broadcastMessages + MAX_NUM_BCAST_MSGS); bcPtr < ptrEnd; bcPtr++)
	{
		if (pHandle < 0 || bcPtr->pHandle == pHandle)
		{
			bcPtr->period = MSG_PERIOD_DISABLED;
		}
	}
}

void disableDidBroadcast(com_manager_t* cmInstance, int pHandle, p_data_disable_t* disable)
{
	for (broadcast_msg_t* bcPtr = cmInstance->broadcastMessages, *ptrEnd = (cmInstance->broadcastMessages + MAX_NUM_BCAST_MSGS); bcPtr < ptrEnd; bcPtr++)
	{
		if ((pHandle < 0 || pHandle == bcPtr->pHandle) && bcPtr->dataHdr.id == disable->id)
		{
			bcPtr->period = MSG_PERIOD_DISABLED;
		}
	}
	
	// Call global broadcast handler to disable message control
	if (cmInstance->cmMsgHandlerRmc)
	{
		p_data_get_t req;
		req.id = disable->id;
		req.size = 0;
		req.offset = 0;
		req.bc_period_multiple = 0;
		cmInstance->cmMsgHandlerRmc(cmInstance, pHandle, &req);
	}
}

/**
*   @brief Encode and send out serial port the referenced packet structure.
*
*	@param[in/out] dPkt Packet structure containing packet info.
*
*	@return 0 on success.  -1 on failure.
*/
int sendPacket(com_manager_t* cmInstance, int pHandle, packet_t *dPkt, uint8_t additionalPktFlags)
{
	buffer_t buffer;

	if (encodeBinaryPacket(cmInstance, pHandle, &buffer, dPkt, additionalPktFlags))
	{
		return -1;
	}

	// Send Packet
	else if (cmInstance->sendPacketCallback)
	{
		cmInstance->sendPacketCallback(cmInstance, pHandle, buffer.buf, buffer.size);
	}

	return 0;
}

// Consolidate this with sendPacket() so that we break up packets into multiples that fit our buffer size.
int sendDataPacket(com_manager_t* cmInstance, int pHandle, pkt_info_t* msg)
{
	pfnComManagerSend sendCallback = cmInstance->sendPacketCallback;
	if (sendCallback == 0)
	{
		return -1;
	}

	buffer_t bufToSend;
	packet_t pkt;
	pkt.hdr = msg->hdr;

	switch (pkt.hdr.pid)
	{
		// Large data support - breaks data up into separate packets for Tx
		case PID_DATA:
		case PID_SET_DATA:
		{
			// Setup packet and encoding state
			buffer_t bufToEncode;
			p_data_hdr_t hdr = *(p_data_hdr_t*)msg->bodyHdr.ptr;
			p_data_hdr_t* hdrToSend = (p_data_hdr_t*)bufToEncode.buf;
			uint32_t size = hdr.size;
			uint32_t offset = 0;
			uint32_t id = hdr.id;
			pkt.body.ptr = bufToEncode.buf;

#if ENABLE_PACKET_CONTINUATION

			while (size > 0)
			{
				
#endif
				
				// Assign data header values
				hdrToSend->size = _MIN(size, MAX_P_DATA_BODY_SIZE);
				hdrToSend->offset = hdr.offset + offset;
				hdrToSend->id = id;

				// copy the data to send to bufToEncode, skipping the data header - since we had to create that data header, we now have to append the actual data
				memcpy(bufToEncode.buf + sizeof(p_data_hdr_t), msg->txData.ptr + offset, hdrToSend->size);
				
				// reduce size by the amount sent - if packet continuation is off, this must become 0 otherwise we fail
				size -= hdrToSend->size;
				
#if ENABLE_PACKET_CONTINUATION

				// increment offset for the next packet
				offset += hdrToSend->size;
				
#else

				if (size > 0)
				{
					// data was too large to fit in one packet, fail
					return -1;
				}
				
#endif

				// Set data body size
				pkt.body.size = sizeof(p_data_hdr_t) + hdrToSend->size;

				// Encode the packet, handling special characters, etc.
				if (encodeBinaryPacket(cmInstance, pHandle, &bufToSend, &pkt, CM_PKT_FLAGS_MORE_DATA_AVAILABLE * (size != 0)))
				{
					return -1;
				}

				// Send the packet using the specified callback
				sendCallback(cmInstance, pHandle, bufToSend.buf, bufToSend.size);
				
#if ENABLE_PACKET_CONTINUATION

			}
			
#endif

		} break;

		// Single packet commands/data sets. No data header, just body.
		default:
		{
			// Assign packet pointer and encode data as is
			pkt.body = msg->txData;
			if (encodeBinaryPacket(cmInstance, pHandle, &bufToSend, &pkt, 0))
			{
				return -1;
			}

			// Send the packet using the specified callback
			sendCallback(cmInstance, pHandle, bufToSend.buf, bufToSend.size);
		} break;
	}

	return 0;
}

void sendAck(com_manager_t* cmInstance, int pHandle, packet_t *pkt, unsigned char pid_ack)
{
	int ackSize;
	bufPtr_t data;

	// Create and Send request packet
	p_ack_t ack = { 0 };
	ack.hdr.pktInfo = pkt->hdr.pid;
	ack.hdr.pktCounter = pkt->hdr.counter;
	ackSize = sizeof(p_ack_hdr_t);

	// Set ack body
	switch (pkt->hdr.pid)
	{
	case PID_SET_DATA:
// 		memcpy(ack.body.buf, (p_data_hdr_t*)(pkt->body.ptr), sizeof(p_data_hdr_t));
		ack.body.dataHdr = *((p_data_hdr_t*)(pkt->body.ptr));
		ackSize += sizeof(p_data_hdr_t);
		break;
	}

	data.ptr = (unsigned char*)&ack;
	data.size = ackSize;

	comManagerSendInstance(cmInstance, pHandle, (uint8_t)pid_ack, 0, &data, 0);
}

//////////////////////////////////////////////////////////////////////////
//  Packet Composition
//////////////////////////////////////////////////////////////////////////
/**
*  @brief Adds data to a packet: adds start, info, data length, data, checksum, and stop bytes.
*  All data is communicated in the endianess of the sender, each packet has a bit that determines big or little endian.
*  Process for Creating Tx Packet:
*  1.) Add to packet
*      - pkt start byte
*      - pkt ID
*      - pkt counter
*      - pkt flags
*      - data length
*      - data ID
*      - data start...
*      - ...data end
*      - pkt reserved
*      - computed cksum (2 bytes)
*      - pkt end byte
*  2.) Tx encode extraneous special characters to remove them from packet
*
*	@return 0 on success, -1 on failure.
*/
int encodeBinaryPacket(com_manager_t* cmInstance, int pHandle, buffer_t *pkt, packet_t *dPkt, uint8_t additionalPktFlags)
{
	com_manager_port_t *port = &(cmInstance->ports[pHandle]);
	
	void* srcBuffer = dPkt->body.ptr;
	int srcBufferLength = dPkt->body.size;
	void* encodedPacket = pkt->buf;
	int encodedPacketLength = PKT_BUF_SIZE - 1;
	packet_hdr_t* hdr = &dPkt->hdr;
	hdr->counter = (uint8_t)(port->comm.txPktCount++);

	pkt->size = is_encode_binary_packet(srcBuffer, srcBufferLength, hdr, additionalPktFlags | port->status.flags, encodedPacket, encodedPacketLength);
	return (-1 * ((int)pkt->size < 8));
}


//////////////////////////////////////////////////////////////////////////
//  Packet Retry
//////////////////////////////////////////////////////////////////////////

/**
*   @brief stepPacketRetry - Resend the ensured packets after the ENSURE_RETRY_COUNT
*   period if the expected response was not received.
*/
void stepPacketRetry(com_manager_t* cmInstance)
{
	int32_t i;
	ensured_pkt_t* ePkt;

	for (i = 0; i < cmInstance->maxEnsuredPackets; i++)
	{
		ePkt = &(cmInstance->ensuredPackets[i]);

		// No more retries in list
		if (ePkt->counter == -2)
		{
			return;
		}

		// Check that retry is enabled
		if (ePkt->counter >= 0)
		{
			// Check if counter has expired
			if (--(ePkt->counter) == 0)
			{
				// Reset counter
				ePkt->counter = cmInstance->ensureRetryCount;

				// Reset packet
				sendPacket(cmInstance, ePkt->pHandle, &(ePkt->pkt), 0);
			}
		}
	}
}

/**
*   @brief registerPacketRetry - Saves data and packet header info
*   to a retry list that will be resent if the corresponding response
*   is not received (data or ack) within the given period.  The packet
*   header info must be populated following a call to this function.
*
*	@param[in] data[]   Pointer to data buffer.
*	@param[in] dataSize Size of the data buffer.
*
*	@return Pointer to retry packet.  The header info must be populated.
*/
packet_t* registerPacketRetry(com_manager_t* cmInstance, int pHandle, uint8_t pid, unsigned char data[], unsigned int dataSize)
{
	int32_t i;
	ensured_pkt_t *ePkt = 0;
	unsigned char searching = 1;

	#if ENABLE_FILTER_DUPLICATE_PACKETS

	#if ENABLE_FILTER_DUPLICATE_PACKETS_MATCH_ALL_CHARACTERS

	int32_t j;

	#endif

	// Filter out redundant retries (replace same type packets and pHandle with latest)
	p_data_get_t *getData1, *getData2;

	// Validate Data Size
	if (dataSize > MAX_P_DATA_BODY_SIZE)
	{
		return 0;
	}

	// Check for existing retry
	for (i = 0; searching && i < cmInstance->maxEnsuredPackets; i++)
	{
		ePkt = &(cmInstance->ensuredPackets[i]);

		// No more retries to search over.  Abort and look for first disabled slot.
		if (ePkt->counter == -2)
		{
			break;
		}

		// Found enabled retry w/ matching packet ID and data size
		if (ePkt->counter >= 0 &&
		ePkt->pkt.hdr.pid == pid		&&
		ePkt->pkt.body.size == dataSize &&
		ePkt->pHandle == pHandle)
		{
			switch (pid)
			{
			case PID_GET_DATA:
				getData1 = (p_data_get_t*)data;
				getData2 = (p_data_get_t*)ePkt->pktBody;

				// Match: all Get Data parameters
				if (getData1->id == getData2->id     &&
				getData1->size == getData2->size   &&
				getData1->offset == getData2->offset)
				searching = 0;
				break;

			case PID_STOP_BROADCASTS_ALL_PORTS:
				searching = 0;
				break;

			default:

#if !ENABLE_FILTER_DUPLICATE_PACKETS_MATCH_ALL_CHARACTERS

				// Match: first character
				if (ePkt->pkt.body.ptr[0] == data[0])
				{
					searching = 0;
				}

#else

				// Match: All character
				for (j = 0; j < dataSize; j++)
				{
					if (ePkt->pkt.body.ptr[j] == data[j])
					{
						searching = 0;
						break;
					}
				}

#endif

				break;
			}
		}
	}

	#endif

	// Find Empty Slot - either first available or tail if all used.
	for (i = 0; searching && i < cmInstance->maxEnsuredPackets; i++)
	{
		ePkt = &(cmInstance->ensuredPackets[i]);

		// Found empty slot
		if (ePkt->counter < 0)
		{
			searching = 0;
			break;
		}
	}

	// All slots enabled, so take the oldest (one after last used)
	if (searching && cmInstance->ensuredPackets != 0)
	{
		if (++cmInstance->lastEnsuredPacketIndex >= cmInstance->maxEnsuredPackets)
		{
			cmInstance->lastEnsuredPacketIndex = 0;
		}
		ePkt = &(cmInstance->ensuredPackets[cmInstance->lastEnsuredPacketIndex]);
	}
	else
	{
		cmInstance->lastEnsuredPacketIndex = i;
	}
	if (ePkt == 0)
	{
		return 0;
	}

	// Backup packet contents for retry if not already registered
	ePkt->counter = cmInstance->ensureRetryCount;
	memcpy(ePkt->pktBody, data, dataSize);

	// Update ePkt pkt header and body info
	ePkt->pkt.hdr.startByte = PSC_START_BYTE;
	ePkt->pkt.hdr.pid = pid;
	ePkt->pkt.body.ptr = ePkt->pktBody; // point to ePkt buffer "pktBody"
	ePkt->pkt.body.size = dataSize;
	ePkt->pHandle = pHandle;

	return &(ePkt->pkt);
}

/**
*   @brief Update packet retry.  If the specific data requested or acknowledge
*   is received, the retry list is updated as to no continue to resend the
*   corresponding message.
*
*	@param[in] *pkt        Pointer to pkt buffer.
*/
void updatePacketRetryData(com_manager_t* cmInstance, packet_t *pkt)
{
	int32_t i;
	ensured_pkt_t *ePkt;

	// Search for retries that match packet received.  If found, removed it from the retry list.
	for (i = 0; i < cmInstance->maxEnsuredPackets; i++)
	{
		ePkt = &(cmInstance->ensuredPackets[i]);

		if (ePkt->counter == -2)
		{
			// No more retries to search for
			return;
		}

		if (ePkt->counter < 0)
		{
			// This retry is disabled.  Skip it.
			continue;
		}

		// Found packet response expected.  Remove from retry list.
		if (ePkt->pktBody[0] == pkt->body.ptr[0])
		{
			// Indicate disabled retry
			ePkt->counter = -1;
		}
	}

	// Update last retry indicator
	for (i = cmInstance->maxEnsuredPackets - 1; i >= 0; i--)
	{
		// Current is enabled so stop
		if (cmInstance->ensuredPackets[i].counter >= 0)
		{
			break;
		}

		// Indicate no more retries in list
		cmInstance->ensuredPackets[i].counter = -2;
	}
}

void updatePacketRetryAck(com_manager_t* cmInstance, packet_t *pkt)
{
	int32_t i;
	ensured_pkt_t *ePkt;
	p_ack_t *ack;
	uint8_t ackInfo;

	ack = (p_ack_t*)(pkt->body.ptr);
	ackInfo = (uint8_t)(ack->hdr.pktInfo);

	// Search for retries that match packet received.  If found, removed it from the retry list.
	for (i = 0; i < cmInstance->maxEnsuredPackets; i++)
	{
		ePkt = &(cmInstance->ensuredPackets[i]);

		if (ePkt->counter == -2)
		{
			// No more retries to search for
			return;
		}

		if (ePkt->counter == -1)
		{
			// This retry is disabled.  Skip it.
			continue;
		}

		// Check packet info matches
		if (ack->hdr.pktInfo == ePkt->pkt.hdr.pid)
		{
			p_data_hdr_t *dHdr, *eHdr;

			switch (ackInfo)
			{
				default:
				// Custom/Specific Packets
				case PID_STOP_BROADCASTS_ALL_PORTS: // No body ID available
				ePkt->counter = -1;                 // indicate disabled retry
				break;

				case PID_SET_DATA:
				dHdr = &(ack->body.dataHdr);
				eHdr = (p_data_hdr_t*)(ePkt->pktBody);

				if (dHdr->id == eHdr->id &&
				dHdr->size == eHdr->size &&
				dHdr->offset == eHdr->offset)
				{
					ePkt->counter = -1;             // indicate disabled retry
				}
				break;
			}
		}
	}

	// Update last retry indicator
	for (i = cmInstance->maxEnsuredPackets - 1; i >= 0; i--)
	{
		// Current is enabled so stop
		if (cmInstance->ensuredPackets[i].counter >= 0)
		{
			break;
		}
		cmInstance->ensuredPackets[i].counter = -2;         // Indicate no more retries in list
	}
}

int comManagerValidateBaudRate(unsigned int baudRate)
{
	// Valid baudrates for InertialSense hardware
	for (size_t i = 0; i < _ARRAY_ELEMENT_COUNT(g_validBaudRates); i++)
	{
		if (g_validBaudRates[i] == baudRate)
		{
			return 0;
		}
	}
	return -1;
}

