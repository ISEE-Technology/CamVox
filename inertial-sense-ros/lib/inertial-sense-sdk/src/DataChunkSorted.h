/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef DATA_CHUNK_SORTED_H
#define DATA_CHUNK_SORTED_H

#include <stdint.h>

#include "com_manager.h"
#include "DataChunk.h"
#include "ISLogFileBase.h"

#define LOG_DEBUG_WRITE		0
#define LOG_DEBUG_READ		0

PUSH_PACK_1

/** Represents the complete packet body of a PID_DATA and PID_DATA_SET packet */
typedef struct
{
	/** Serial order of data structure.  Used to re-serialize data.  */
	uint32_t dataSerNum;

	/** Data */
	uint8_t buf[MAX_P_DATA_BODY_SIZE];
} p_cnk_data_t;

// sub-header for sorted data chunks
typedef struct
{
	p_data_hdr_t dHdr;

	// Number of sorted data elements
	uint32_t dCount;
} sChunkSubHeader;

POP_PACK

class cSortedDataChunk : public cDataChunk
{
public:
    cSortedDataChunk(const char* name = "EMPT");
	void Clear() OVERRIDE;

	int32_t WriteAdditionalChunkHeader(cISLogFileBase* pFile) OVERRIDE;
	int32_t ReadAdditionalChunkHeader(cISLogFileBase* pFile) OVERRIDE;
	int32_t GetHeaderSize() OVERRIDE;

	sChunkSubHeader m_subHdr;
};


#endif // DATA_CHUNK_SORTED_H
