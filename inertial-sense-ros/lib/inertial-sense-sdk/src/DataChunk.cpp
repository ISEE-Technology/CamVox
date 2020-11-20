/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "DataChunk.h"
#include "ISLogFileBase.h"
#include "ISLogFileFactory.h"

cDataChunk::cDataChunk()
{
	Clear();
	m_hdr.marker = DATA_CHUNK_MARKER;
	m_hdr.version = 1;
	m_hdr.classification = ' ' << 8 | 'U';
	m_hdr.grpNum = 0;				//!< Chunk group number
	m_hdr.devSerialNum = 0;			//!< Serial number
	m_hdr.reserved = 0;				//!< Reserved 
    m_buffTail = m_buffHead + DEFAULT_CHUNK_DATA_SIZE;
	m_dataHead = m_buffHead;
	m_dataTail = m_buffHead;

	SetName("PDAT");
}


cDataChunk::~cDataChunk()
{
	
}


void cDataChunk::SetName(const char name[4])
{
	if (name == NULL)
	{
		return;
	}

	memcpy(m_hdr.name, name, 4);
	m_hdr.invName[0] = ~m_hdr.name[0];
	m_hdr.invName[1] = ~m_hdr.name[1];
	m_hdr.invName[2] = ~m_hdr.name[2];
	m_hdr.invName[3] = ~m_hdr.name[3];
}


bool cDataChunk::PushBack(uint8_t* d1, uint32_t d1Size, uint8_t* d2, uint32_t d2Size)
{
	// Ensure data will fit
	uint32_t count = d1Size + d2Size;
	if (m_dataHead == NULLPTR ||
		count > GetBuffFree())
	{
		return false;
	}
	if (d1Size > 0)
	{
        memcpy(m_dataTail, d1, d1Size);
        m_dataTail += d1Size;
		m_hdr.dataSize += d1Size;
	}
	if (d2Size > 0)
	{
        memcpy(m_dataTail, d2, d2Size);
        m_dataTail += d2Size;
		m_hdr.dataSize += d2Size;
	}
	m_hdr.invDataSize = ~m_hdr.dataSize;
	return true;
}


uint8_t* cDataChunk::GetDataPtr()
{
	return (GetDataSize() == 0 ? NULLPTR : m_dataHead);
}


bool cDataChunk::PopFront(uint32_t size)
{
	assert(m_dataHead != NULLPTR);

	if (size <= GetDataSize())
	{
        m_dataHead += size;
		m_hdr.dataSize -= size;
		m_hdr.invDataSize = ~m_hdr.dataSize;
		return true;
	}
	else
	{
        Clear();
		return false;
	}
}


void cDataChunk::Clear()
{
	m_hdr.dataSize = 0; //!< Byte size of data in this chunk
	m_hdr.invDataSize = (uint32_t)~0; //!< Bitwise inverse of m_Size
    m_dataHead = m_buffHead;
    m_dataTail = m_buffHead;

#if LOG_CHUNK_STATS
	memset(m_stats, 0, sizeof(m_stats));
#endif

}

// Returns number of bytes written, or -1 for error
int32_t cDataChunk::WriteToFile(cISLogFileBase* pFile, int groupNumber)
{
	if (pFile == NULLPTR)
	{
		return -1;
	}

	assert(m_dataHead != NULLPTR);

	m_hdr.grpNum = groupNumber;

	// Write chunk header to file
	int32_t nBytes = static_cast<int32_t>(pFile->write(&m_hdr, sizeof(sChunkHeader)));

	// Write any additional chunk header
	nBytes += WriteAdditionalChunkHeader(pFile);

	// Write chunk data to file
	nBytes += static_cast<int32_t>(pFile->write(m_dataHead, m_hdr.dataSize));

#if LOG_DEBUG_WRITE
	static int totalBytes = 0;
	totalBytes += nBytes;
	printf("%d : %d  -  %x %d", totalBytes, nBytes, m_Hdr.marker, m_Hdr.dataSize);
	if (nBytes != (headerSize + (int)m_Hdr.dataSize))
		printf("ERROR WRITING!");
	printf("\n");
#endif

	// Error writing to file
	if (nBytes != GetHeaderSize() + (int)m_hdr.dataSize)
	{
		return -1;
	}

	// Clear chunk data
	Clear();

	return nBytes;
}


// Returns number of bytes read, or -1 for error
int32_t cDataChunk::ReadFromFile(cISLogFileBase* pFile)
{
	if (pFile == NULLPTR)
	{
		return -1;
	}

	// reset state, prepare to read from file
	Clear();

	// Read chunk header
	int32_t nBytes = static_cast<int32_t>(pFile->read(&m_hdr, sizeof(sChunkHeader)));

	// Error checking
	if (m_hdr.dataSize != ~(m_hdr.invDataSize) || nBytes <= 0)
	{
		return -1;
	}

	// Read additional chunk header
	nBytes += ReadAdditionalChunkHeader(pFile);

//     // Error check data size
//     if (m_hdr.dataSize > MAX_DATASET_SIZE)
//     {
//         return -1;
//     }

	// Read chunk data
	m_dataTail += static_cast<int32_t>(pFile->read(m_buffHead, m_hdr.dataSize));
	nBytes += GetDataSize();

#if LOG_DEBUG_READ
	static int totalBytes = 0;
	totalBytes += nBytes;
	printf("%d : %d  -  %x %d  ", totalBytes, nBytes, m_Hdr.marker, m_Hdr.dataSize);
	if ((nBytes > 0) && (m_Hdr.marker != DATA_CHUNK_MARKER))
	{
		printf("MARKER MISMATCH!");
}
	printf("\n");
#endif

	if (m_hdr.marker == DATA_CHUNK_MARKER && nBytes == static_cast<int>(GetHeaderSize() + m_hdr.dataSize))
	{

#if LOG_CHUNK_STATS
		static bool start = true;
		int totalBytes = 0;
		for (int id = 0; id < DID_COUNT; id++)
			if (m_stats[id].total)
			{
				if (start)
				{
					start = false;
					logStats("------------------------------------------------\n");
					logStats("Chunk Data Summary\n");
					logStats("   ID:  Count:  Sizeof:  Total:\n");
				}
				logStats(" %4d%8d    %5d %7d\n", id, m_stats[id].count, m_stats[id].size, m_stats[id].total);
				totalBytes += m_stats[id].total;
			}
		start = true;
		if (totalBytes)
		{
			logStats("------------------------------------------------\n");
			logStats("                      %8d Total bytes\n", totalBytes);
		}
		logStats("\n");
		logStats("================================================\n");
		logStats("            *.dat Data Log File\n");
		logStats("================================================\n");
		m_Hdr.print();
#if LOG_CHUNK_STATS==2
		logStats("------------------------------------------------\n");
		logStats("Chunk Data\n");
#endif
		memset(m_stats, 0, sizeof(m_stats));
#endif

		return nBytes;
	}
	else
	{
        Clear();

		// Error reading from file
		return -1;
	}
}


int32_t cDataChunk::WriteAdditionalChunkHeader(cISLogFileBase* /*pFile*/)
{
	return 0;
}


int32_t cDataChunk::ReadAdditionalChunkHeader(cISLogFileBase* /*pFile*/)
{
	return 0;
}


int32_t cDataChunk::GetHeaderSize()
{
	return (int32_t)sizeof(sChunkHeader);
}


// LOG_CHUNK_STATS
void logStats( const char *format, ... )
{
#if !defined(PLATFORM_IS_EVB_2) || !PLATFORM_IS_EVB_2
	static cISLogFileBase* pFile = NULL;
#endif

	va_list args;
	va_start( args, format );

#if PLATFORM_IS_EVB_2
	vprintf( format, args );			// print to terminal
#else
	if( pFile == NULL ) {
		pFile = CreateISLogFile("STATS_.txt", "w");
	}
	pFile->vprintf(format, args);	// print to file
#endif
	va_end( args );
}
