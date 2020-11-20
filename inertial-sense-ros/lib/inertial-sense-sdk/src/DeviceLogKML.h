/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef DEVICE_LOG_KML_H
#define DEVICE_LOG_KML_H

#include <stdio.h>
#include <string>
#include <vector>

#include "DataKML.h"
#include "DeviceLog.h"
#include "com_manager.h"

#if PLATFORM_IS_EVB_2
#include "ff.h"
#endif


struct sKmlLog
{
	std::vector<sKmlLogData> data;

	std::string				fileName;
	uint32_t				fileCount;
	uint32_t				fileDataSize;		// Byte size of chunk data.  Excludes chunk header.
	uint32_t				fileSize;
};



class cDeviceLogKML : public cDeviceLog
{
public:
	void InitDeviceForWriting(int pHandle, std::string timestamp, std::string directory, uint64_t maxDiskSpace, uint32_t maxFileSize) OVERRIDE;
	bool CloseAllFiles() OVERRIDE;
	bool CloseWriteFile(int kid, sKmlLog& log);
	bool OpenWithSystemApp(void) OVERRIDE;
    bool SaveData(p_data_hdr_t* dataHdr, const uint8_t* dataBuf) OVERRIDE;
	p_data_t* ReadData() OVERRIDE;
	void SetSerialNumber(uint32_t serialNumber) OVERRIDE;
	std::string LogFileExtention() OVERRIDE { return std::string(".kml"); }

private:
	bool OpenNewSaveFile(int kid, sKmlLog &log) { (void)kid; (void)log; return true; }
	p_data_t* ReadDataFromChunk();
	bool ReadChunkFromFile();
    bool WriteDateToFile(const p_data_hdr_t *dataHdr, const uint8_t *dataBuf);

	cDataKML                m_kml;
	sKmlLog                 m_Log[cDataKML::MAX_NUM_KID];
};

#endif // DEVICE_LOG_KML_H
