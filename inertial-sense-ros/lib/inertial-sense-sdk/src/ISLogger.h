/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef IS_LOGGER_H
#define IS_LOGGER_H

#define _FILE_OFFSET_BITS 64

#include <stdio.h>
#include <string>
#include <vector>
#include <map>

#include "DeviceLogSerial.h"

#if !defined(PLATFORM_IS_EVB_2) || !PLATFORM_IS_EVB_2
#include "DeviceLogSorted.h"
#include "DeviceLogCSV.h"
#include "DeviceLogJSON.h"
#include "DeviceLogKML.h"
#endif

#if PLATFORM_IS_EVB_2
#include "drivers/d_time.h"
#endif

#include "ISConstants.h"
#include "ISDisplay.h"
#include "ISLogFile.h"

using namespace std;

// default logging path if none specified
#define DEFAULT_LOGS_DIRECTORY "IS_logs"

class cLogStats;

class cISLogger
{
public:
	enum eLogType
	{
		LOGTYPE_DAT = 0,
		LOGTYPE_SDAT,
		LOGTYPE_CSV,
		LOGTYPE_KML,
		LOGTYPE_JSON
	};

	static const string g_emptyString;

	cISLogger();
	virtual ~cISLogger();

	// Setup logger to read from file.
	bool LoadFromDirectory(const string& directory, eLogType logType = LOGTYPE_DAT, vector<string> serials = {});

	// Setup logger for writing to file.
	bool InitSave(eLogType logType = LOGTYPE_DAT, const string& directory = g_emptyString, int numDevices = 1, float maxDiskSpacePercent = 0.5f, uint32_t maxFileSize = 1024 * 1024 * 5, bool useSubFolderTimestamp = true);
	bool InitSaveTimestamp(const string& timeStamp, const string& directory = g_emptyString, const string& subDirectory = g_emptyString, int numDevices = 1, eLogType logType = LOGTYPE_DAT, float maxDiskSpacePercent = 0.5f, uint32_t maxFileSize = 1024 * 1024 * 5, bool useSubFolderTimestamp = true);

	// update internal state, handle timeouts, etc.
	void Update();
	bool LogData(unsigned int device, p_data_hdr_t* dataHdr, const uint8_t* dataBuf);
	p_data_t* ReadData(unsigned int device = 0);
	p_data_t* ReadNextData(unsigned int& device);
	void EnableLogging(bool enabled) { m_enabled = enabled; }
	bool Enabled() { return m_enabled; }
	void CloseAllFiles();
	void OpenWithSystemApp();
	string TimeStamp() { return m_timeStamp; }
	string LogDirectory() { return m_directory; }
	uint64_t LogSizeAll();
	uint64_t LogSize(unsigned int device = 0);
	float LogSizeAllMB();
	float LogSizeMB(unsigned int device = 0);
	float FileSizeMB(unsigned int device = 0);
	uint32_t FileCount(unsigned int device = 0);
	uint32_t GetDeviceCount() { return (uint32_t)m_devices.size(); }
	bool SetDeviceInfo(const dev_info_t *info, unsigned int device = 0);
	const dev_info_t* GetDeviceInfo(unsigned int device = 0);
	bool CopyLog(cISLogger& log, const string& timestamp = g_emptyString, const string& outputDir = g_emptyString, eLogType logType = LOGTYPE_DAT, float maxDiskSpacePercent = 0.5f, uint32_t maxFileSize = 1024 * 1024 * 5, bool useSubFolderTimestamp = true);
	const cLogStats& GetStats() { return *m_logStats; }
	eLogType GetType() { return m_logType; }

	/**
	* Get the timeout flush parameter in seconds
	* @return the timeout flush parameter in seconds
	*/
	time_t GetTimeoutFlushSeconds() { return m_timeoutFlushSeconds; }

	/**
	* Set the timeout flush logger parameter in seconds
	* @param timeoutFlushSeconds the timeout flush logger parameter in seconds
	*/
	void SetTimeoutFlushSeconds(time_t timeoutFlushSeconds) { m_timeoutFlushSeconds = timeoutFlushSeconds; }

    // check if a data header is corrupt
    static bool LogHeaderIsCorrupt(const p_data_hdr_t* hdr);

	// create a timestamp
	static string CreateCurrentTimestamp();

    // check if a data packet is corrupt, NULL data is OK
    static bool LogDataIsCorrupt(const p_data_t* data);

    // read all log data into memory - if the log is over 1.5 GB this will fail on 32 bit processes
    // the map contains device id (serial number) key and a vector containing log data for each data id, which will be an empty vector if no log data for that id
    static bool ReadAllLogDataIntoMemory(const string& directory, map<uint32_t, vector<vector<uint8_t>>>& data);

	void SetKmlConfig(bool showPath = true, bool showSample = false, bool showTimeStamp = true, double updatePeriodSec = 1.0, bool altClampToGround = true)
	{
		m_showPath = showPath;
		m_showSample = showSample;
		m_showTimeStamp = showTimeStamp;
		m_iconUpdatePeriodSec = updatePeriodSec;
		m_altClampToGround = altClampToGround;

		for (unsigned int dev = 0; dev < GetDeviceCount(); dev++)
		{
			m_devices[dev]->SetKmlConfig(m_showPath, m_showSample, m_showTimeStamp, m_iconUpdatePeriodSec, m_altClampToGround);
		}
	}

	static eLogType ParseLogType(const string& logTypeString)
	{
		if (logTypeString == "csv")
		{
			return cISLogger::eLogType::LOGTYPE_CSV;
		}
		else if (logTypeString == "kml")
		{
			return cISLogger::eLogType::LOGTYPE_KML;
		}
		else if (logTypeString == "sdat")
		{
			return cISLogger::eLogType::LOGTYPE_SDAT;
		}
		else if (logTypeString == "json")
		{
			return cISLogger::eLogType::LOGTYPE_JSON;
		}
		return cISLogger::eLogType::LOGTYPE_DAT;
	}

private:
#if CPP11_IS_ENABLED
    cISLogger(const cISLogger& copy) = delete;
#else
	cISLogger(const cISLogger& copy); // Disable copy constructors
#endif

	bool InitSaveCommon(eLogType logType, const string& directory, const string& subDirectory, int numDevices, float maxDiskSpacePercent, uint32_t maxFileSize, bool useSubFolderTimestamp);
	bool InitDevicesForWriting(int numDevices = 1);
	void Cleanup();

	static time_t GetTime()
    {
#if PLATFORM_IS_EVB_2
        return static_cast<time_t>(time_msec() / 1000);
#else
        return time(NULLPTR);
#endif
    }

	eLogType				m_logType;
	bool					m_enabled;
	string					m_directory;
	string					m_timeStamp;
	vector<cDeviceLog*>		m_devices;
	uint64_t				m_maxDiskSpace;
	uint32_t				m_maxFileSize;
	cLogStats*				m_logStats;
	cISLogFileBase*         m_errorFile;

	bool					m_altClampToGround;
	bool					m_showSample;
	bool					m_showPath;
	bool					m_showTimeStamp;
	double					m_iconUpdatePeriodSec;
	time_t					m_lastCommTime;
	time_t					m_timeoutFlushSeconds;
};

class cLogStatDataId
{
public:
	uint64_t count; // count for this data id
	uint64_t errorCount; // error count for this data id
	double averageTimeDelta; // average time delta for the data id
	double totalTimeDelta; // sum of all time deltas
	double lastTimestamp;
	double lastTimestampDelta;
    double minTimestampDelta;
    double maxTimestampDelta;
	uint64_t timestampDeltaCount;
	uint64_t timestampDropCount; // count of delta timestamps > 50% different from previous delta timestamp

	cLogStatDataId();
	void LogTimestamp(double timestamp);
	void Printf();
};

class cLogStats
{
public:
	cLogStatDataId dataIdStats[DID_COUNT];
	uint64_t count; // count of all data ids
	uint64_t errorCount; // total error count

	cLogStats();
	void Clear();
	void LogError(const p_data_hdr_t* hdr);
	void LogData(uint32_t dataId);
	void LogDataAndTimestamp(uint32_t dataId, double timestamp);
	void Printf();
    void WriteToFile(const string& fileName);
};

#endif // IS_LOGGER_H
