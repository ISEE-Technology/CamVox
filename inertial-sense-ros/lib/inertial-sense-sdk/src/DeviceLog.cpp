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

#include "DeviceLog.h"
#include "ISFileManager.h"
#include "ISLogger.h"
#include "ISConstants.h"
#include "ISDataMappings.h"
#include "ISLogFileFactory.h"

using namespace std;


cDeviceLog::cDeviceLog()
{
    m_pFile = NULL;
    m_pHandle = 0;
    m_fileSize = 0;
    m_logSize = 0;
    m_fileCount = 0;
    memset(&m_devInfo, 0, sizeof(dev_info_t));
    m_logStats = new cLogStats;
	m_altClampToGround = true;
	m_showTracks = true;
	m_showPointTimestamps = true;
	m_pointUpdatePeriodSec = 1.0f;
}

cDeviceLog::~cDeviceLog()
{
    // Close open files
    CloseISLogFile(m_pFile);
    CloseAllFiles();
    delete m_logStats;
}

void cDeviceLog::InitDeviceForWriting(int pHandle, std::string timestamp, std::string directory, uint64_t maxDiskSpace, uint32_t maxFileSize)
{
    m_pHandle = pHandle;
	m_timeStamp = timestamp;
	m_directory = directory;
	m_fileCount = 0;
	m_maxDiskSpace = maxDiskSpace;
	m_maxFileSize = maxFileSize;
	m_logSize = 0;
}


void cDeviceLog::InitDeviceForReading()
{
	m_fileSize = 0;
	m_logSize = 0;
	m_fileCount = 0;
}


bool cDeviceLog::CloseAllFiles()
{
#if 1
    string str = m_directory + "/stats_SN" + to_string(m_devInfo.serialNumber) + ".txt";
    m_logStats->WriteToFile(str);
#else   // stringstream not working on embedded platform
	ostringstream serialNumString;
	serialNumString << m_devInfo.serialNumber;
    m_logStats->WriteToFile(m_directory + "/stats_SN" + serialNumString.str() + ".txt");
#endif
    m_logStats->Clear();
    return true;
}

bool cDeviceLog::OpenWithSystemApp()
{

#if PLATFORM_IS_WINDOWS

	std::wstring stemp = std::wstring(m_fileName.begin(), m_fileName.end());
	LPCWSTR filename = stemp.c_str();
	ShellExecuteW(0, 0, filename, 0, 0, SW_SHOW);

#endif

	return true;
}


bool cDeviceLog::SaveData(p_data_hdr_t *dataHdr, const uint8_t* dataBuf)
{
    if (dataHdr != NULL)
    {
        double timestamp = cISDataMappings::GetTimestamp(dataHdr, dataBuf);
        m_logStats->LogDataAndTimestamp(dataHdr->id, timestamp);
    }
    return true;
}

bool cDeviceLog::SetupReadInfo(const string& directory, const string& serialNum, const string& timeStamp)
{
	m_directory = directory;
	m_fileCount = 0;
	m_timeStamp = timeStamp;
	m_fileNames.clear();
	vector<ISFileManager::file_info_t> fileInfos;
	SetSerialNumber((uint32_t)strtoul(serialNum.c_str(), NULL, 10));
	ISFileManager::GetDirectorySpaceUsed(directory, string("[\\/\\\\]" IS_LOG_FILE_PREFIX) + serialNum + "_", fileInfos, false, false);
	if (fileInfos.size() != 0)
	{
		m_fileName = fileInfos[0].name;
		for (size_t i = 0; i < fileInfos.size(); i++)
		{
			m_fileNames.push_back(fileInfos[i].name);
		}
	}
	return true;
}


bool cDeviceLog::OpenNewSaveFile()
{
	// Close existing file
	CloseISLogFile(m_pFile);

	// Ensure directory exists
	if (m_directory.empty())
	{
		return false;
	}

	// create directory
	_MKDIR(m_directory.c_str());

#if !PLATFORM_IS_EMBEDDED

	// clear out space if we need to
	if (m_maxDiskSpace != 0)
	{
		vector<ISFileManager::file_info_t> files;
		uint64_t spaceUsed = ISFileManager::GetDirectorySpaceUsed(m_directory.c_str(), files, true, false);
		unsigned int index = 0;

		// clear out old files until we have space
		while (spaceUsed > m_maxDiskSpace && index < files.size())
		{
			spaceUsed -= files[index].size;
			ISFileManager::DeleteFile(files[index++].name);
		}
	}
	
#endif

	// Open new file
	m_fileCount++;
	uint32_t serNum = m_devInfo.serialNumber;
	if (!serNum)
	{
		serNum = m_pHandle;
	}
	string fileName = GetNewFileName(serNum, m_fileCount, NULL);
	m_pFile = CreateISLogFile(fileName, "wb");
	m_fileSize = 0;

	if (m_pFile && m_pFile->isOpened())
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
		return false;
	}
}


bool cDeviceLog::OpenNextReadFile()
{
	// Close file if open
	CloseISLogFile(m_pFile);

	if (m_fileCount == m_fileNames.size())
	{
		return false;
	}
	
	m_fileName = m_fileNames[m_fileCount++];
	m_pFile = CreateISLogFile(m_fileName, "rb");

	if (m_pFile)
	{

#if LOG_DEBUG_READ
		printf("File opened: %s\n", m_fileName.c_str());
#endif
		return true;
	}
	else
	{
#if LOG_DEBUG_READ
		printf("FAILED to open file: %s\n", m_fileName.c_str());
#endif
		return false;
	}
}

string cDeviceLog::GetNewFileName(uint32_t serialNumber, uint32_t fileCount, const char* suffix)
{
	// file name 
#if 1
    char filename[200];
    SNPRINTF(filename, sizeof(filename), "%s/%s%d_%s_%04d%s%s", 
        m_directory.c_str(), 
        IS_LOG_FILE_PREFIX, 
        (int)serialNumber, 
        m_timeStamp.c_str(), 
        (int)(fileCount % 10000), 
        (suffix == NULL || *suffix == 0 ? "" : (string("_") + suffix).c_str()), 
        LogFileExtention().c_str());
    return filename;
#else
    // This code is not working on the embedded platform
	ostringstream filename;
	filename << m_directory << "/" << IS_LOG_FILE_PREFIX <<
		serialNumber << "_" <<
		m_timeStamp << "_" <<
		setfill('0') << setw(4) << (fileCount % 10000) <<
		(suffix == NULL || *suffix == 0 ? "" : string("_") + suffix) <<
		LogFileExtention();
	return filename.str();
#endif
}


void cDeviceLog::SetDeviceInfo(const dev_info_t *info)
{
	if (info == NULL)
	{
		return;
	}
	m_devInfo = *info;
	SetSerialNumber(info->serialNumber);
}


void cDeviceLog::OnReadData(p_data_t* data)
{
    if (data != NULL)
    {
        double timestamp = cISDataMappings::GetTimestamp(&data->hdr, data->buf);
        m_logStats->LogDataAndTimestamp(data->hdr.id, timestamp);
    }
}


