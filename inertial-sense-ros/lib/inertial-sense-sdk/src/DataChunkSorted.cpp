/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <string.h>
#include "DataChunkSorted.h"


cSortedDataChunk::cSortedDataChunk(const char* name) : cDataChunk()
{
	memset(&m_subHdr, 0, sizeof(sChunkSubHeader));
	SetName(name);
}


void cSortedDataChunk::Clear()
{
	cDataChunk::Clear();
	m_subHdr.dCount = 0;
}


int32_t cSortedDataChunk::WriteAdditionalChunkHeader(cISLogFileBase* pFile)
{
	// Write sub header to file
	return static_cast<int32_t>(pFile->write(&m_subHdr, sizeof(m_subHdr)));
}


int32_t cSortedDataChunk::ReadAdditionalChunkHeader(cISLogFileBase* pFile)
{
	// Read chunk header
	return static_cast<int32_t>(pFile->read(&m_subHdr, sizeof(m_subHdr)));
}


int32_t cSortedDataChunk::GetHeaderSize()
{
    return int32_t((sizeof(sChunkHeader) + sizeof(sChunkSubHeader)));
}
