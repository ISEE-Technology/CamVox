/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef DEVICE_LOG_H
#define DEVICE_LOG_H

#include <stdio.h>
#include <string.h>
#include <vector>
#include "ISLogFileBase.h"

extern "C"
{
	#include "com_manager.h"
}

using namespace std;

// never change these!
#define IS_LOG_FILE_PREFIX "LOG_SN"
#define IS_LOG_FILE_PREFIX_LENGTH 6
#define IS_LOG_TIMESTAMP_LENGTH 15

class cLogStats;

class cDeviceLog
{
public:
    cDeviceLog();
    virtual ~cDeviceLog();
	virtual void InitDeviceForWriting(int pHandle, string timestamp, string directory, uint64_t maxDiskSpace, uint32_t maxFileSize);
	virtual void InitDeviceForReading();
    virtual bool CloseAllFiles();
	virtual bool OpenWithSystemApp();
    virtual bool SaveData(p_data_hdr_t *dataHdr, const uint8_t* dataBuf);
    virtual p_data_t* ReadData() = 0;
	virtual void SetSerialNumber(uint32_t serialNumber) = 0;
	virtual string LogFileExtention() = 0;
	virtual void Flush() {}
    bool SetupReadInfo(const string& directory, const string& deviceName, const string& timeStamp);
    void SetDeviceInfo(const dev_info_t *info);
    const dev_info_t* GetDeviceInfo() { return &m_devInfo; }
	uint64_t FileSize() { return m_fileSize; }
	uint64_t LogSize() { return m_logSize; }
	uint32_t FileCount() { return m_fileCount; }
	void SetKmlConfig(bool showTracks =true, bool showPoints =true, bool showPointTimestamps =true, double pointUpdatePeriodSec=1.0, bool altClampToGround=true)
	{ 
		m_showTracks = showTracks;
		m_showPoints = showPoints;
		m_showPointTimestamps = showPointTimestamps;
		m_pointUpdatePeriodSec = pointUpdatePeriodSec;
		m_altClampToGround = altClampToGround; 
	}

protected:
	bool OpenNewSaveFile();
	bool OpenNextReadFile();
	string GetNewFileName(uint32_t serialNumber, uint32_t fileCount, const char* suffix);
    void OnReadData(p_data_t* data);

	vector<string>          m_fileNames;
	cISLogFileBase*         m_pFile;
	string                  m_directory;
	string                  m_timeStamp;
	string                  m_fileName;
	dev_info_t              m_devInfo;
	int                     m_pHandle;
	uint64_t                m_fileSize;
	uint64_t                m_logSize;
	uint32_t                m_fileCount;
	uint64_t                m_maxDiskSpace;
	uint32_t                m_maxFileSize;
	bool                    m_altClampToGround;
	bool                    m_showTracks;
	bool                    m_showPoints;
	bool                    m_showPointTimestamps;
	double                  m_pointUpdatePeriodSec;

private:
    cLogStats*              m_logStats;
};

#endif // DEVICE_LOG_H
