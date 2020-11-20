/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef DEVICE_LOG_CSV_H
#define DEVICE_LOG_CSV_H

#include <stdio.h>
#include <string>
#include <vector>
#include <map>

#include "DataCSV.h"
#include "DeviceLog.h"
#include "com_manager.h"

class cCsvLog
{
public:
	cCsvLog() : pFile(NULL), fileCount(0), fileSize(0), dataId(0), orderId(0) { }

	FILE* pFile;
	uint32_t fileCount;
	uint64_t fileSize;
	uint32_t dataId;
	uint32_t dataSize;
	uint64_t orderId;
	string nextLine;
	vector<data_info_t> columnHeaders;
};


class cDeviceLogCSV : public cDeviceLog
{
public:
	void InitDeviceForWriting(int pHandle, std::string timestamp, std::string directory, uint64_t maxDiskSpace, uint32_t maxFileSize) OVERRIDE;
	void InitDeviceForReading() OVERRIDE;
	bool CloseAllFiles() OVERRIDE;
    bool SaveData(p_data_hdr_t* dataHdr, const uint8_t* dataBuf) OVERRIDE;
	p_data_t* ReadData() OVERRIDE;
	void SetSerialNumber(uint32_t serialNumber) OVERRIDE;
	std::string LogFileExtention() OVERRIDE { return std::string(".csv"); }

private:
	bool OpenNewFile(cCsvLog& log, bool readOnly);
	bool GetNextLineForFile(cCsvLog& log);

	p_data_t* ReadDataFromFile(cCsvLog& log);
	map<uint32_t, cCsvLog> m_logs;
	cDataCSV m_csv;
	map<uint32_t, vector<string> > m_currentFiles; // all files for each data set
	map<uint32_t, uint32_t> m_currentFileIndex; // contains the current csv file index for each data set
	p_data_t m_dataBuffer;
	uint64_t m_nextId; // for writing the log, column 0 of csv is an incrementing id. This lets us read the log back in order.
};

#endif // DEVICE_LOG_CSV_H
