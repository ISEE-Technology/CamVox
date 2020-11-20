/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <ctime>
#include <string>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <iomanip>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>

#include "DeviceLogSerial.h"
#include "ISLogger.h"
#include "ISLogFileFactory.h"

using namespace std;

void cDeviceLogSerial::InitDeviceForWriting(int pHandle, std::string timestamp, std::string directory, uint64_t maxDiskSpace, uint32_t maxFileSize)
{
//     m_chunk.Init(chunkSize);
	m_chunk.m_hdr.pHandle = pHandle;

	cDeviceLog::InitDeviceForWriting(pHandle, timestamp, directory, maxDiskSpace, maxFileSize);
}


bool cDeviceLogSerial::CloseAllFiles()
{
    cDeviceLog::CloseAllFiles();

	// Write any remaining chunk data to file
	WriteChunkToFile();

	// Close file
	CloseISLogFile(m_pFile);

	return true;
}


bool cDeviceLogSerial::SaveData(p_data_hdr_t* dataHdr, const uint8_t* dataBuf)
{
    cDeviceLog::SaveData(dataHdr, dataBuf);

	// Add serial number if available
	if (dataHdr->id == DID_DEV_INFO && !copyDataPToStructP2(&m_devInfo, dataHdr, dataBuf, sizeof(dev_info_t)))
	{
		int start = dataHdr->offset;
		int end = dataHdr->offset + dataHdr->size;
		int snOffset = offsetof(dev_info_t, serialNumber);

		// Did we really get the serial number?
		if (start <= snOffset && (int)(snOffset + sizeof(uint32_t)) <= end)
		{
			m_chunk.m_hdr.devSerialNum = m_devInfo.serialNumber;
		}
	}

	// Ensure data will fit in chunk.  If not, create new chunk
	uint32_t dataBytes = sizeof(p_data_hdr_t) + dataHdr->size;
	if (dataBytes > m_chunk.GetBuffFree())
	{
        // Save chunk to file and clear
		if (!WriteChunkToFile())
		{
			return false;
		}
		else if (m_fileSize >= m_maxFileSize)
		{
			// Close existing file
			CloseAllFiles();
		}
	}
	// Add data header and data buffer to chunk
	if (!m_chunk.PushBack((unsigned char*)dataHdr, sizeof(p_data_hdr_t), (unsigned char*)dataBuf, dataHdr->size))
	{
		return false;
	}

	return true;
}


bool cDeviceLogSerial::WriteChunkToFile()
{
	// Make sure we have data to write
	if (m_chunk.GetDataSize() == 0)
	{
		return false;
	}

	// Create first file if it doesn't exist
	if (m_pFile == NULLPTR)
	{
		OpenNewSaveFile();
	}

	// Validate file pointer
	if (m_pFile == NULLPTR)
	{
		return false;
	}

	// Write chunk to file
	int fileBytes = m_chunk.WriteToFile(m_pFile, 0);
	if (!m_pFile->good())
	{
		return false;
	}

	// File byte size
	m_fileSize += fileBytes;
	m_logSize += fileBytes;

	return true;
}


p_data_t* cDeviceLogSerial::ReadData()
{
	p_data_t* data = NULL;

	// Read data from chunk
	while (!(data = ReadDataFromChunk()))
	{
		// Read next chunk from file
		if (!ReadChunkFromFile())
		{
			return NULL;
		}
	}

	// Read is good
    cDeviceLog::OnReadData(data);
	return data;
}


p_data_t* cDeviceLogSerial::ReadDataFromChunk()
{
	// Ensure chunk has data
	if (m_chunk.GetDataSize() <= 0)
	{
		return NULL;
	}

	p_data_t* data = (p_data_t*)m_chunk.GetDataPtr();
	int size = data->hdr.size + sizeof(p_data_hdr_t);
	if (m_chunk.PopFront(size))
	{
		return data;
	}
	else
	{
		return NULL;
	}
}


bool cDeviceLogSerial::ReadChunkFromFile()
{
	// Read next chunk from file
	while (m_chunk.ReadFromFile(m_pFile) < 0)
	{
		if (!OpenNextReadFile())
		{
			// No more data or error opening next file
			return false;
		}
	}
	return true;
}


void cDeviceLogSerial::SetSerialNumber(uint32_t serialNumber)
{
	m_devInfo.serialNumber = serialNumber;
	m_chunk.m_hdr.devSerialNum = serialNumber;
}


void cDeviceLogSerial::Flush()
{
	if (WriteChunkToFile())
	{
		m_pFile->flush();
	}
}


