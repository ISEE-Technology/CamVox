/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef DATA_CHUNK_H
#define DATA_CHUNK_H

#include "ISConstants.h"
#if PLATFORM_IS_EVB_2
#define DEFAULT_CHUNK_DATA_SIZE     16384			// 16 KB (EVB)
#else
#define DEFAULT_CHUNK_DATA_SIZE     131072          // 128 KB
#endif

#define DATA_CHUNK_MARKER           0xFC05EA32

#include <stdint.h>

#include "com_manager.h"
#include "ISLogFileBase.h"

#define LOG_DEBUG_WRITE		0		// Enable debug printout
#define LOG_DEBUG_READ		0
#define LOG_CHUNK_STATS		0		// 0 = disabled, 1 = summary, 2 = detailed

using namespace std;

void logStats(const char *format, ...);

//!< Chunk Header
PUSH_PACK_1

struct sChunkHeader 
{
	uint32_t	marker;				//!< Chunk marker (0xFC05EA32)
	uint16_t	version;			//!< Chunk Version
	uint16_t	classification;		//!< Chunk classification
	char		name[4];			//!< Chunk name
	char		invName[4];			//!< Bitwise inverse of chunk name
	uint32_t	dataSize;			//!< Chunk data length in bytes
	uint32_t	invDataSize;		//!< Bitwise inverse of chunk data length
	uint32_t	grpNum;				//!< Chunk Group Number: 0 = serial data, 1 = sorted data...
	uint32_t	devSerialNum;		//!< Device serial number
	uint32_t	pHandle;			//!< Device port handle
	uint32_t	reserved;			//!< Unused

#if LOG_CHUNK_STATS
	void print()
	{
		logStats( "Chunk Header\n" );
		logStats( "         marker:  %u (0x%x)\n", marker, marker );
		logStats( "        version:  %d\n", version );
		logStats( " classification:  %d\n", classification );
		logStats( "           name:  %c%c%c%c\n", name[0], name[1], name[2], name[3] );
		logStats( "        invName:  %c%c%c%c\n", invName[0], invName[1], invName[2], invName[3] );
		logStats( "       dataSize:  %d\n", dataSize );
		logStats( "    invDataSize:  %d\n", invDataSize );
		logStats( "         grpNum:  %d\n", grpNum );
		logStats( "   devSerialNum:  %d\n", devSerialNum );
		logStats( "        pHandle:  %d\n", pHandle );
		logStats( "       reserved:  %d\n", reserved );
	}
#endif
};

POP_PACK

class cDataChunk
{
public:
	cDataChunk();
    virtual ~cDataChunk();

    uint32_t GetBuffSize() { return (uint32_t)(m_buffTail - m_buffHead); }
    uint32_t GetBuffFree() { return (uint32_t)(m_buffTail - m_dataTail); }
    uint32_t GetDataSize() { return (uint32_t)(m_dataTail - m_dataHead); }
    void SetName(const char name[4]);
	uint8_t* GetDataPtr();
	bool PopFront(uint32_t size);
    int32_t WriteToFile(cISLogFileBase* pFile, int groupNumber = 0); // Returns number of bytes written to file and clears the chunk
	int32_t ReadFromFile(cISLogFileBase* pFile);
	bool PushBack(uint8_t* d1, uint32_t d1Size, uint8_t* d2 = NULL, uint32_t d2Size = 0);

	virtual void Clear();

	sChunkHeader m_hdr;

#if LOG_CHUNK_STATS
	struct
	{
		uint32_t count;		// Number of occurrences
		uint32_t size;		// Size of each data structure
		uint32_t total;		// Total bytes read
	} m_stats[DID_COUNT];
#endif

protected:
	virtual int32_t WriteAdditionalChunkHeader(cISLogFileBase* pFile);
	virtual int32_t ReadAdditionalChunkHeader(cISLogFileBase* pFile);
	virtual int32_t GetHeaderSize();

private:
    uint8_t m_buffHead[DEFAULT_CHUNK_DATA_SIZE];    // Start of buffer
    uint8_t* m_buffTail;    // End of buffer
    uint8_t* m_dataHead;    // Front of data in buffer.  This moves as data is read.
    uint8_t* m_dataTail;    // End of data in buffer.  This moves as data is written.
};


#endif // DATA_CHUNK_H
