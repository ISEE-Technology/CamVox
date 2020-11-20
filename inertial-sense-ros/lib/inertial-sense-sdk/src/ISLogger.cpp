/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <ctime>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <regex>
#include <set>
#include <sstream>

#include "ISFileManager.h"
#include "ISLogger.h"
#include "ISDataMappings.h"
#include "ISLogFileFactory.h"

#include "convert_ins.h"


#if PLATFORM_IS_EVB_2
#include "globals.h"
#include "rtc.h"
#endif

#if PLATFORM_IS_LINUX || PLATFORM_IS_APPLE

#include <sys/statvfs.h>

#endif

// #define DONT_CHECK_LOG_DATA_SET_SIZE		// uncomment to allow reading in of new data logs into older code sets


const string cISLogger::g_emptyString;

bool cISLogger::LogHeaderIsCorrupt(const p_data_hdr_t* hdr)
{
    bool corrupt = (hdr != NULL &&
    (
        hdr->size == 0 ||
        hdr->offset + hdr->size > MAX_DATASET_SIZE ||
        hdr->id == 0 ||
        hdr->id >= DID_COUNT ||
        hdr->offset % 4 != 0 ||
        hdr->size % 4 != 0 
#if !defined(DONT_CHECK_LOG_DATA_SET_SIZE)
		|| (cISDataMappings::GetSize(hdr->id) > 0 && hdr->offset + hdr->size > cISDataMappings::GetSize(hdr->id))
#endif
    ));
	return corrupt;
}

bool cISLogger::LogDataIsCorrupt(const p_data_t* data)
{
    return (data != NULL && LogHeaderIsCorrupt(&data->hdr));
}

cISLogger::cISLogger()
{
	m_enabled = false;
	m_logStats = new cLogStats;
	m_logStats->Clear();
	m_errorFile = CreateISLogFile();
	m_lastCommTime = 0;
	m_timeoutFlushSeconds = 0;
}


cISLogger::~cISLogger()
{
	Cleanup();
	CloseISLogFile(m_errorFile);
	delete m_logStats;
}


void cISLogger::Cleanup()
{
	// Delete old devices
	for (unsigned int i = 0; i < m_devices.size(); i++)
	{
		if (m_devices[i] != NULLPTR)
		{
			delete m_devices[i];
		}
	}
	m_devices.clear();
	m_logStats->Clear();
}


void cISLogger::Update()
{
	// if we have a timeout param and the time has elapsed, shutdown
	if (m_lastCommTime == 0)
	{
		m_lastCommTime = GetTime();
	}
	else if (m_timeoutFlushSeconds > 0 && GetTime() - m_lastCommTime >= m_timeoutFlushSeconds)
	{
		for (unsigned int i = 0; i < m_devices.size(); i++)
		{
			m_devices[i]->Flush();
		}
	}
	ISFileManager::TouchFile(m_directory + "/stats.txt");
}


bool cISLogger::InitSaveCommon(eLogType logType, const string& directory, const string& subDirectory, int numDevices, float maxDiskSpacePercent, uint32_t maxFileSize, bool useSubFolderTimestamp)
{
	static const int minFileCount = 50;
	static const int maxFileCount = 10000;

	// Close any open files
	CloseAllFiles();

	m_logType = logType;
	m_directory = (directory.empty() ? DEFAULT_LOGS_DIRECTORY : directory);
    maxDiskSpacePercent = _CLAMP(maxDiskSpacePercent, 0.01f, 0.99f);
	uint64_t availableSpace = ISFileManager::GetDirectorySpaceAvailable(m_directory);
	m_maxDiskSpace = (maxDiskSpacePercent <= 0.0f ? availableSpace : (uint64_t)(availableSpace * maxDiskSpacePercent));

	// ensure there are between min and max file count
	if (maxFileSize > m_maxDiskSpace / minFileCount)
	{
		m_maxFileSize = (uint32_t)(m_maxDiskSpace / minFileCount);
	}
	else if (maxFileSize < m_maxDiskSpace / maxFileCount)
	{
		m_maxFileSize = (uint32_t)(m_maxDiskSpace / maxFileCount);
	}
	else
	{
		m_maxFileSize = maxFileSize;
	}

	// create root dir
	_MKDIR(m_directory.c_str());

	if (useSubFolderTimestamp)
	{
		// create time stamp dir
		m_directory = directory + "/" + m_timeStamp;
		_MKDIR(m_directory.c_str());

		if (!subDirectory.empty())
		{
			// create sub dir
			m_directory += "/" + subDirectory;
			_MKDIR(m_directory.c_str());
		}
	}

	// create empty stats file to track timestamps
    string str = m_directory + "/" + subDirectory + "/stats.txt";
	cISLogFileBase* statsFile = CreateISLogFile(str, "w");
	CloseISLogFile(statsFile);

	// Initialize devices
	return InitDevicesForWriting(numDevices);
}


string cISLogger::CreateCurrentTimestamp()
{
	char buf[80];

#if PLATFORM_IS_EVB_2
	date_time_t rtc;
#if USE_RTC_DATE_TIME   // use RTC
	rtc_get_date(RTC, &rtc.year, &rtc.month, &rtc.day, &rtc.week);
	rtc_get_time(RTC, &rtc.hour, &rtc.minute, &rtc.second);
#else   // use uINS GPS time
    rtc = g_gps_date_time;    
#endif
	SNPRINTF(buf, _ARRAY_BYTE_COUNT(buf), "%.4d%.2d%.2d_%.2d%.2d%.2d",
			(int)rtc.year, (int)rtc.month, (int)rtc.day, 
			(int)rtc.hour, (int)rtc.minute, (int)rtc.second);
#else
    // Create timestamp
	time_t rawtime = GetTime();
	struct tm * timeinfo = localtime(&rawtime);
	strftime(buf, 80, "%Y%m%d_%H%M%S", timeinfo);
#endif

	return string(buf);
}


bool cISLogger::InitSave(eLogType logType, const string& directory, int numDevices, float maxDiskSpacePercent, uint32_t maxFileSize, bool useSubFolderTimestamp)
{
	m_timeStamp = CreateCurrentTimestamp();
	return InitSaveCommon(logType, directory, g_emptyString, numDevices, maxDiskSpacePercent, maxFileSize, useSubFolderTimestamp);
}


bool cISLogger::InitSaveTimestamp(const string& timeStamp, const string& directory, const string& subDirectory, int numDevices, eLogType logType, float maxDiskSpacePercent, uint32_t maxFileSize, bool useSubFolderTimestamp)
{
	if (timeStamp.length() == 0)
	{
		m_timeStamp = CreateCurrentTimestamp();
	}
	else
	{
		// Only use first 15 characters for the timestamp
		m_timeStamp = timeStamp.substr(0, IS_LOG_TIMESTAMP_LENGTH);
	}

	return InitSaveCommon(logType, directory, subDirectory, numDevices, maxDiskSpacePercent, maxFileSize, useSubFolderTimestamp);
}


bool cISLogger::InitDevicesForWriting(int numDevices)
{
	Cleanup();

	// Add devices
	m_devices.resize(numDevices);

	// Create and init new devices
	for (int i = 0; i < numDevices; i++)
	{
		switch (m_logType)
		{
		default:
		case LOGTYPE_DAT:	m_devices[i] = new cDeviceLogSerial();	break;
#if !defined(PLATFORM_IS_EVB_2) || !PLATFORM_IS_EVB_2
		case LOGTYPE_SDAT:	m_devices[i] = new cDeviceLogSorted();	break;
		case LOGTYPE_CSV:	m_devices[i] = new cDeviceLogCSV();		break;
		case LOGTYPE_JSON:	m_devices[i] = new cDeviceLogJSON();	break;
		case LOGTYPE_KML:	m_devices[i] = new cDeviceLogKML();		break;
#endif
		}

		m_devices[i]->InitDeviceForWriting(i, m_timeStamp, m_directory, m_maxDiskSpace, m_maxFileSize);
	}

	m_errorFile = CreateISLogFile((m_directory + "/errors.txt"), "w");

    return ISFileManager::PathIsDir(m_directory);
}

bool cISLogger::LoadFromDirectory(const string& directory, eLogType logType, vector<string> serials)
{
	// Delete and clear prior devices
	Cleanup();
	m_logType = logType;
	string fileExtensionRegex;
	set<string> serialNumbers;

	switch (logType)
	{
	default:
	case cISLogger::LOGTYPE_DAT: fileExtensionRegex = "\\.dat$"; break;
	case cISLogger::LOGTYPE_SDAT: fileExtensionRegex = "\\.sdat$"; break;
	case cISLogger::LOGTYPE_CSV: fileExtensionRegex = "\\.csv$"; break;
	case cISLogger::LOGTYPE_JSON: fileExtensionRegex = "\\.json$"; break;
	case cISLogger::LOGTYPE_KML: return false; // fileExtensionRegex = "\\.kml$"; break; // kml read not supported
	}

	// get all files, sorted by name
	vector<ISFileManager::file_info_t> files;
	ISFileManager::GetDirectorySpaceUsed(directory, fileExtensionRegex, files, false, false);
	if (files.size() == 0)
	{
		return false;
	}

	for (size_t i = 0; i < files.size(); i++)
	{
		string name = ISFileManager::GetFileName(files[i].name);

		// check for log file prefix
		size_t endOfLogPrefixIndex = name.find(IS_LOG_FILE_PREFIX);
		if (endOfLogPrefixIndex != string::npos)
		{
			endOfLogPrefixIndex += IS_LOG_FILE_PREFIX_LENGTH;
			size_t serialNumberEndIndex = name.find('_', endOfLogPrefixIndex);
			if (serialNumberEndIndex != string::npos)
			{
				string serialNumber = name.substr(endOfLogPrefixIndex, serialNumberEndIndex - endOfLogPrefixIndex);

				// if we don't have a timestamp yet, see if we can parse it from the filename, i.e. IS_LOG_FILE_PREFIX 30013_20170103_151023_001
				if (m_timeStamp.length() == 0)
				{
					// find _ at end of timestamp
					size_t timestampIndex = name.find_last_of('_');
					if (timestampIndex == string::npos)
					{
						timestampIndex = name.find_last_of('.');
					}
					if (timestampIndex != string::npos && timestampIndex - ++serialNumberEndIndex == IS_LOG_TIMESTAMP_LENGTH)
					{
						m_timeStamp = name.substr(timestampIndex - IS_LOG_TIMESTAMP_LENGTH, IS_LOG_TIMESTAMP_LENGTH);
					}
				}

				// check for unique serial numbers
                if (serialNumbers.find(serialNumber) == serialNumbers.end())
                {
                    if (serials.size() == 0 || ((find(serials.begin(), serials.end(), "SN" + serialNumber) != serials.end() ||
                        find(serials.begin(), serials.end(), "ALL") != serials.end()))) // and that it is a serial number we want to use
                    {
                        serialNumbers.insert(serialNumber);

                        // Add devices
                        switch (logType)
                        {
                        default:
                        case cISLogger::LOGTYPE_DAT: m_devices.push_back(new cDeviceLogSerial()); break;
#if !defined(PLATFORM_IS_EVB_2) || !PLATFORM_IS_EVB_2
                        case cISLogger::LOGTYPE_SDAT: m_devices.push_back(new cDeviceLogSorted()); break;
                        case cISLogger::LOGTYPE_CSV: m_devices.push_back(new cDeviceLogCSV()); break;
                        case cISLogger::LOGTYPE_JSON: m_devices.push_back(new cDeviceLogJSON()); break;
#endif
                        }
                        m_devices.back()->SetupReadInfo(directory, serialNumber, m_timeStamp);
                    }
                }
			}
		}
	}

	for (uint32_t i = 0; i < m_devices.size(); i++)
	{
		m_devices[i]->InitDeviceForReading();
	}
	return (m_devices.size() != 0);
}


bool cISLogger::LogData(unsigned int device, p_data_hdr_t* dataHdr, const uint8_t* dataBuf)
{
	m_lastCommTime = GetTime();

	if (!m_enabled)
	{
        m_errorFile->lprintf("Logger is not enabled\r\n");
		return false;
	}
	else if (device >= m_devices.size() || dataHdr == NULL || dataBuf == NULL)
	{
        m_errorFile->lprintf("Invalid device handle or NULL data\r\n");
		return false;
	}
	else if (LogHeaderIsCorrupt(dataHdr))
	{
        m_errorFile->lprintf("Corrupt log header, id: %lu, offset: %lu, size: %lu\r\n", (unsigned long)dataHdr->id, (unsigned long)dataHdr->offset, (unsigned long)dataHdr->size);
		m_logStats->LogError(dataHdr);
	}
    else if (!m_devices[device]->SaveData(dataHdr, dataBuf))
    {
        m_errorFile->lprintf("Underlying log implementation failed to save\r\n");
        m_logStats->LogError(dataHdr);
    }
#if 1
    else
    {
        double timestamp = cISDataMappings::GetTimestamp(dataHdr, dataBuf);
        m_logStats->LogDataAndTimestamp(dataHdr->id, timestamp);

        if (dataHdr->id == DID_DIAGNOSTIC_MESSAGE)
        {
            // write to diagnostic text file
#if 0        
            std::ostringstream outFilePath;
            outFilePath << m_directory << "/diagnostic_" << m_devices[device]->GetDeviceInfo()->serialNumber << ".txt";
            cISLogFileBase *outfile = CreateISLogFile(outFilePath.str(), "a");
#else
            cISLogFileBase *outfile = CreateISLogFile(m_directory + "/diagnostic_" + std::to_string(m_devices[device]->GetDeviceInfo()->serialNumber) + ".txt", "a");
#endif            
            std::string msg = (((diag_msg_t*)dataBuf)->message);
            outfile->write(msg.c_str(), msg.length());
            if (msg.length() > 0 && *msg.end() != '\n')
            {
                outfile->putch('\n');
            }
            CloseISLogFile(outfile);
        }
    }
#endif
	return true;
}


p_data_t* cISLogger::ReadData(unsigned int device)
{
	if (device >= m_devices.size())
	{
		return NULL;
	}

	p_data_t* data = NULL;
	while (LogDataIsCorrupt(data = m_devices[device]->ReadData()))
	{
	    m_errorFile->lprintf("Corrupt log header, id: %lu, offset: %lu, size: %lu\r\n", (unsigned long)data->hdr.id, (unsigned long)data->hdr.offset, (unsigned long)data->hdr.size);
		m_logStats->LogError(&data->hdr);
		data = NULL;
	}
	if (data != NULL)
	{
        double timestamp = cISDataMappings::GetTimestamp(&data->hdr, data->buf);
		m_logStats->LogDataAndTimestamp(data->hdr.id, timestamp);
	}
	return data;
}


p_data_t* cISLogger::ReadNextData(unsigned int& device)
{
	while (device < m_devices.size())
	{
		p_data_t* data = ReadData(device);
		if (data == NULL)
		{
			++device;
		}
	}
	return NULL;
}


void cISLogger::CloseAllFiles()
{
	for( unsigned int i = 0; i < m_devices.size(); i++ )
	{
		m_devices[i]->CloseAllFiles();
	}

    m_logStats->WriteToFile(m_directory + "/stats.txt");
    CloseISLogFile(m_errorFile);
}


void cISLogger::OpenWithSystemApp()
{
	for (unsigned int i = 0; i < m_devices.size(); i++)
	{
		m_devices[i]->OpenWithSystemApp();
	}
}


uint64_t cISLogger::LogSizeAll()
{
    uint64_t size = 0;
    for (size_t i = 0; i < m_devices.size(); i++)
    {
        size += m_devices[i]->LogSize();
    }
    return size;
}


uint64_t cISLogger::LogSize( unsigned int device )
{
	if (device >= m_devices.size())
	{
		return 0;
	}

	return m_devices[device]->LogSize();
}


float cISLogger::LogSizeAllMB()
{
    return LogSizeAll() * 0.000001f;
}


float cISLogger::LogSizeMB( unsigned int device )
{
	if (device >= m_devices.size())
	{
		return 0;
	}

	return m_devices[device]->LogSize() * 0.000001f;
}


float cISLogger::FileSizeMB( unsigned int device )
{
    if (device >= m_devices.size())
    {
        return 0;
    }

    return m_devices[device]->FileSize() * 0.000001f;
}


uint32_t cISLogger::FileCount( unsigned int device )
{
	if (device >= m_devices.size())
	{
		return 0;
	}

	return m_devices[device]->FileCount();
}

bool cISLogger::SetDeviceInfo(const dev_info_t *info, unsigned int device )
{
	if (device >= m_devices.size() || info == NULL)
	{
		return false;
	}

	m_devices[device]->SetDeviceInfo( info );
	return true;
}

const dev_info_t* cISLogger::GetDeviceInfo( unsigned int device )
{
	if (device >= m_devices.size())
	{
		return NULL;
	}

	return m_devices[device]->GetDeviceInfo();
}

bool cISLogger::CopyLog(cISLogger& log, const string& timestamp, const string &outputDir, eLogType logType, float maxLogSpacePercent, uint32_t maxFileSize, bool useSubFolderTimestamp)
{
	m_logStats->Clear();
	if (!InitSaveTimestamp(timestamp, outputDir, g_emptyString, log.GetDeviceCount(), logType, maxLogSpacePercent, maxFileSize, useSubFolderTimestamp))
	{
		return false;
	}
	EnableLogging(true);
	p_data_t* data = NULL;
	for (unsigned int dev = 0; dev < log.GetDeviceCount(); dev++)
	{
		// Copy device info
		const dev_info_t* devInfo = log.GetDeviceInfo(dev);
		SetDeviceInfo(devInfo, dev);

		// Set KML configuration
		m_devices[dev]->SetKmlConfig(m_showPath, m_showSample, m_showTimeStamp, m_iconUpdatePeriodSec, m_altClampToGround);

		// Copy data
		while ((data = log.ReadData(dev)))
		{
			// CSV special cases 
			if (logType == eLogType::LOGTYPE_CSV)
			{
				if (data->hdr.id == DID_INS_2)
				{	// Convert INS2 to INS1 when creating .csv logs
					ins_1_t ins1;
					ins_2_t ins2;

					copyDataPToStructP(&ins2, data, sizeof(ins_2_t));
					convertIns2ToIns1(&ins2, &ins1);

					p_data_hdr_t hdr;
					hdr.id = DID_INS_1;
					hdr.size = sizeof(ins_1_t);
					hdr.offset = 0;
					LogData(dev, &hdr, (uint8_t*)&ins1);
				}
			}

			// Save data
            LogData(dev, &data->hdr, data->buf);
		}
	}
	CloseAllFiles();
	return true;
}

bool cISLogger::ReadAllLogDataIntoMemory(const string& directory, map<uint32_t, vector<vector<uint8_t>>>& data)
{
    cISLogger logger;
    if (!logger.LoadFromDirectory(directory))
    {
        return false;
    }
    unsigned int deviceId = 0;
    unsigned int lastDeviceId = 0xFFFFFEFE;
    p_data_t* p;
	vector<vector<uint8_t>>* currentDeviceData = NULL;
    const dev_info_t* info;
    uint8_t* ptr, *ptrEnd;
    data.clear();

    // read every piece of data out of every device log
    while ((p = logger.ReadNextData(deviceId)) != NULL)
    {
        // if this is a new device, we need to add the device to the map using the serial number
        if (deviceId != lastDeviceId)
        {
            lastDeviceId = deviceId;
            info = logger.GetDeviceInfo(deviceId);
            data[info->serialNumber] = vector<vector<uint8_t>>();
            currentDeviceData = &data[info->serialNumber];
        }

		assert(currentDeviceData != NULL);

        // add slots until we have slots at least equal to the data id
        while (currentDeviceData->size() < p->hdr.id)
        {
            currentDeviceData->push_back(vector<uint8_t>());
        }

        // append each byte of the packet to the slot
        ptrEnd = p->buf + p->hdr.size;
		vector<uint8_t>& stream = (*currentDeviceData)[p->hdr.id];
        for (ptr = p->buf; ptr != ptrEnd; ptr++)
        {
			stream.push_back(*ptr);
        }
    }
    return true;
}

cLogStatDataId::cLogStatDataId()
{
	count = 0;
	errorCount = 0;
	averageTimeDelta = 0.0;
	totalTimeDelta = 0.0;
	lastTimestamp = 0.0;
	lastTimestampDelta = 0.0;
	maxTimestampDelta = 0.0;
    minTimestampDelta = 1.0E6;
    timestampDeltaCount = 0;
	timestampDropCount = 0;
}

void cLogStatDataId::LogTimestamp(double timestamp)
{
    // check for corrupt data
    if (_ISNAN(timestamp) || timestamp < 0.0 || timestamp > 999999999999.0)
    {
        return;
    }
    else if (lastTimestamp > 0.0)
    {
        double delta = fabs(timestamp - lastTimestamp);
        minTimestampDelta = _MIN(delta, minTimestampDelta);
        maxTimestampDelta = _MAX(delta, maxTimestampDelta);
        totalTimeDelta += delta;
        averageTimeDelta = (totalTimeDelta / (double)++timestampDeltaCount);
        if (lastTimestampDelta != 0.0 && (fabs(delta - lastTimestampDelta) > (lastTimestampDelta * 0.5)))
        {
            timestampDropCount++;
        }
        lastTimestampDelta = delta;
	}
    lastTimestamp = timestamp;
}

void cLogStatDataId::Printf()
{
	
#if !PLATFORM_IS_EMBEDDED
	
	printf(" Count: %llu,   Errors: %llu\r\n", (unsigned long long)count, (unsigned long long)errorCount);
	printf(" Time delta: (ave, min, max) %f, %f, %f\r\n", averageTimeDelta, minTimestampDelta, maxTimestampDelta);
	printf(" Time delta drop: %llu\r\n", (unsigned long long)timestampDropCount);
	
#endif

}

cLogStats::cLogStats()
{
	Clear();
}

void cLogStats::Clear()
{
	memset(dataIdStats, 0, sizeof(dataIdStats));
    for (uint32_t id = 0; id < DID_COUNT; id++)
    {
        dataIdStats[id].minTimestampDelta = 1.0E6;
    }
	errorCount = 0;
	count = 0;
}

void cLogStats::LogError(const p_data_hdr_t* hdr)
{
	errorCount++;
	if (hdr != NULL && hdr->id < DID_COUNT)
	{
		cLogStatDataId& d = dataIdStats[hdr->id];
		d.errorCount++;
	}
}

void cLogStats::LogData(uint32_t dataId)
{
	if (dataId < DID_COUNT)
	{
		cLogStatDataId& d = dataIdStats[dataId];
		d.count++;
		count++;
	}
}

void cLogStats::LogDataAndTimestamp(uint32_t dataId, double timestamp)
{
	if (dataId < DID_COUNT)
	{
		cLogStatDataId& d = dataIdStats[dataId];
		d.count++;
		count++;
		if (timestamp != 0.0)
		{
			d.LogTimestamp(timestamp);
		}
	}
}

void cLogStats::Printf()
{
	
#if !PLATFORM_IS_EMBEDDED
	
	printf("LOG STATS\r\n");
	printf("----------");
	printf("Count: %llu,   Errors: %llu\r\n", (unsigned long long)count, (unsigned long long)errorCount);
	for (uint32_t id = 0; id < DID_COUNT; id++)
	{
		if (dataIdStats[id].count != 0)
		{
			printf(" DID: %d\r\n", id);
			dataIdStats[id].Printf();
			printf("\r\n");
		}
	}
	
#endif
	
}

void cLogStats::WriteToFile(const string &file_name)
{
    if (count != 0)
    {
        // flush log stats to disk
#if 1
        cISLogFileBase *statsFile = CreateISLogFile(file_name, "wb");        
        statsFile->lprintf("Total count: %d,   Total errors: \r\n\r\n", count, errorCount);
        for (uint32_t id = 0; id < DID_COUNT; id++)
        {
            cLogStatDataId& stat = dataIdStats[id];
            if (stat.count == 0 && stat.errorCount == 0)
            {   // Exclude zero count stats
                continue;
            }

            statsFile->lprintf("Data Id: %d (%s)\r\n", id, cISDataMappings::GetDataSetName(id));
            statsFile->lprintf("Count: %d,   Errors: %d\r\n", stat.count, stat.errorCount);
            statsFile->lprintf("Timestamp Delta (ave, min, max): %.4f, %.4f, %.4f\r\n", stat.averageTimeDelta, stat.minTimestampDelta, stat.maxTimestampDelta);
            statsFile->lprintf("Timestamp Drops: %d\r\n", stat.timestampDropCount);
            statsFile->lprintf("\r\n");
        }
        CloseISLogFile(statsFile);
#else
        // The following code doesn't work on the EVB-2.  Converting stringstream to a string causes the system to hang.
    	CONST_EXPRESSION char CRLF[]= "\r\n";
        stringstream stats;
        stats << fixed << setprecision(6);
        stats << "Total count: " << count << ",   Total errors: " << errorCount << CRLF << CRLF;
        for (uint32_t id = 0; id < DID_COUNT; id++)
        {
            cLogStatDataId& stat = dataIdStats[id];
            if (stat.count == 0 && stat.errorCount == 0)
            {   // Exclude zero count stats
                continue;
            }

            stats << "Data Id: " << id << " (" << cISDataMappings::GetDataSetName(id) << ")" << CRLF;
            stats << "Count: " << stat.count << ",   Errors: " << stat.errorCount << CRLF;
            stats << "Timestamp Delta (ave, min, max): " << stat.averageTimeDelta << ", " << stat.minTimestampDelta << ", " << stat.maxTimestampDelta << CRLF;
            stats << "Timestamp Drops: " << stat.timestampDropCount << CRLF;
            stats << CRLF;
        }
        cISLogFileBase *statsFile = CreateISLogFile(file_name, "wb");
        statsFile->puts(stats.str().c_str());
        CloseISLogFile(statsFile);
#endif
    }
}
