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
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <climits>

#include "ISPose.h"
#include "DeviceLogCSV.h"
#include "ISFileManager.h"
#include "ISDataMappings.h"

using namespace std;


void cDeviceLogCSV::InitDeviceForWriting(int pHandle, std::string timestamp, std::string directory, uint64_t maxDiskSpace, uint32_t maxFileSize)
{
	m_logs.clear();
	m_nextId = 0;
	cDeviceLog::InitDeviceForWriting(pHandle, timestamp, directory, maxDiskSpace, maxFileSize);
}


void cDeviceLogCSV::InitDeviceForReading()
{
	cDeviceLog::InitDeviceForReading();

	// open the first file for each possible data set
	m_logs.clear();
	for (uint32_t id = DID_NULL + 1; id < DID_COUNT; id++)
	{
		const char* dataSet = cISDataMappings::GetDataSetName(id);
		if (dataSet != NULL)
		{
			string dataSetRegex = string(dataSet) + "\\.csv$";
			vector<ISFileManager::file_info_t> infos;
			vector<string> files;
			ISFileManager::GetDirectorySpaceUsed(m_directory, dataSetRegex, infos, false, false);
			if (infos.size() != 0)
			{
				cCsvLog log;
				log.dataId = id;
				log.dataSize = cISDataMappings::GetSize(log.dataId);
				for (size_t i = 0; i < infos.size(); i++)
				{
					files.push_back(infos[i].name);
				}
				m_currentFiles[id] = files;
				m_currentFileIndex[id] = 0;
				while (OpenNewFile(log, true) && !GetNextLineForFile(log)) {}
				if (log.nextLine.length() != 0)
				{
					m_logs[id] = log;
				}
			}
		}
	}
}


bool cDeviceLogCSV::CloseAllFiles()
{
	cDeviceLog::CloseAllFiles();

	for (map<uint32_t, cCsvLog>::iterator i = m_logs.begin(); i != m_logs.end(); i++)
	{
		cCsvLog& log = i->second;
		if (log.pFile)
		{
			fclose(log.pFile);
			log.pFile = NULL;
		}
	}
	m_logs.clear();
	return true;
}


bool cDeviceLogCSV::OpenNewFile(cCsvLog& log, bool readonly)
{
	const char* dataSetName = cISDataMappings::GetDataSetName(log.dataId);
	if (dataSetName == NULL)
	{
		return false;
	}

	// Close existing file
	if (log.pFile)
	{
		fclose(log.pFile);
		log.pFile = NULL;
	}

	// Ensure directory exists
	if (m_directory.empty())
	{
		return false;
	}

	_MKDIR(m_directory.c_str());

	// Open new file
	uint32_t serNum = m_devInfo.serialNumber;
	if (!serNum)
	{
		serNum = m_pHandle;
	}

	if (readonly)
	{
tryNextFile:
		{
			uint32_t index = m_currentFileIndex[log.dataId];
			vector<string>& files = m_currentFiles[log.dataId];
			if (index >= files.size())
			{
				m_logs.erase(log.dataId);
				return false;
			}
			string currentFile = files[index++];
			m_currentFileIndex[log.dataId] = index;
			log.pFile = fopen(currentFile.c_str(), "r");
			log.fileCount++;
			struct stat st;
			stat(currentFile.c_str(), &st);
			log.fileSize = st.st_size;
			m_logSize += log.fileSize;
			if (m_csv.ReadHeaderFromFile(log.pFile, log.dataId, log.columnHeaders) == 0)
			{
				goto tryNextFile;
			}
		}
	}
	else
	{
		log.fileCount++;
		string fileName = GetNewFileName(serNum, log.fileCount, dataSetName);
		log.pFile = fopen(fileName.c_str(), "w");

		// Write Header
		int fileBytes = m_csv.WriteHeaderToFile(log.pFile, log.dataId);

		// File byte size
		log.fileSize = fileBytes;
		m_logSize += fileBytes;
	}

	if (log.pFile)
	{
#if LOG_DEBUG_WRITE
		printf("Opened save file: %s\n", filename.str().c_str());
#endif
		return true;
	}
	else
	{
#if LOG_DEBUG_WRITE
		printf("FAILED to open save file: %s\n", filename.str().c_str());
#endif
		m_logs.erase(log.dataId);
		return false;
	}
}


bool cDeviceLogCSV::GetNextLineForFile(cCsvLog& log)
{
	if (log.pFile == NULL)
	{
		return false;
	}
	char lineBuffer[8192];
	if (fgets(lineBuffer, _ARRAY_BYTE_COUNT(lineBuffer), log.pFile) == NULL)
	{
		return false;
	}
	log.nextLine = lineBuffer;
	size_t index = log.nextLine.find(',', 0);
	if (index < log.nextLine.size())
	{
		log.orderId = strtoull(log.nextLine.c_str(), NULL, 10);
	}
	else
	{
		return false;
	}
	return true;
}


bool cDeviceLogCSV::SaveData(p_data_hdr_t* dataHdr, const uint8_t* dataBuf)
{
    cDeviceLog::SaveData(dataHdr, dataBuf);

	// Reference current log
	cCsvLog& log = m_logs[dataHdr->id];
	log.dataId = dataHdr->id;

	// Create first file it it doesn't exist, return out if failure
	if (log.pFile == NULL && (!OpenNewFile(log, false) || log.pFile == NULL))
	{
		return false;
	}
	else if (dataHdr->id == DID_DEV_INFO)
	{
		memcpy(&m_devInfo, dataBuf, sizeof(dev_info_t));
	}

	// Write date to file
    int nBytes = m_csv.WriteDataToFile(++m_nextId, log.pFile, *dataHdr, dataBuf);
	if (ferror(log.pFile) != 0)
	{
		return false;
	}

	// File byte size
	log.fileSize += nBytes;
	m_logSize += nBytes;

	if (log.fileSize >= m_maxFileSize)
	{
		// Close existing file
		fclose(log.pFile);
		log.pFile = NULL;
		log.fileSize = 0;
	}

	return true;
}


p_data_t* cDeviceLogCSV::ReadData()
{

tryAgain:
	if (m_logs.size() == 0)
	{
		return NULL;
	}

	p_data_t* data = NULL;
	cCsvLog* nextLog = NULL;
	uint64_t nextId = ULLONG_MAX;
	for (map<uint32_t, cCsvLog>::iterator i = m_logs.begin(); i != m_logs.end(); )
	{
		if (i->second.orderId < nextId)
		{
			nextLog = &i->second;
			nextId = i->second.orderId;
		}
		i++;
	}
	if (nextLog == NULL)
	{
		assert(false);
		return NULL;
	}
	if ((data = ReadDataFromFile(*nextLog)) == NULL)
	{
		goto tryAgain;
	}

    cDeviceLog::OnReadData(data);
	return data;
}


p_data_t* cDeviceLogCSV::ReadDataFromFile(cCsvLog& log)
{
	if (log.pFile == NULL)
	{
		assert(false);
		return NULL;
	}
	m_dataBuffer.hdr.id = log.dataId;
	m_dataBuffer.hdr.size = log.dataSize;
	if (m_csv.StringCSVToData(log.nextLine, m_dataBuffer.hdr, m_dataBuffer.buf, _ARRAY_BYTE_COUNT(m_dataBuffer.buf), log.columnHeaders))
	{
		if (m_dataBuffer.hdr.id == DID_DEV_INFO)
		{
			memcpy(&m_devInfo, m_dataBuffer.buf, sizeof(dev_info_t));
		}
		log.nextLine.clear();
		while (!GetNextLineForFile(log) && OpenNewFile(log, true)) {}
		return &m_dataBuffer;
	}
	log.nextLine.clear();
	while (!GetNextLineForFile(log) && OpenNewFile(log, true)) {}
	return NULL;
}


void cDeviceLogCSV::SetSerialNumber(uint32_t serialNumber)
{
	m_devInfo.serialNumber = serialNumber;
}



