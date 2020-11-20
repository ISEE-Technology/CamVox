/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef DEVICE_LOG_SERIAL_H
#define DEVICE_LOG_SERIAL_H

#include <stdio.h>
#include <string>
#include <vector>

#include "DataChunk.h"
#include "DeviceLog.h"
#include "com_manager.h"




class cDeviceLogSerial : public cDeviceLog
{
public:
    cDeviceLogSerial(){}

	void InitDeviceForWriting(int pHandle, std::string timestamp, std::string directory, uint64_t maxDiskSpace, uint32_t maxFilesize) OVERRIDE;
	bool CloseAllFiles() OVERRIDE;
    bool SaveData(p_data_hdr_t* dataHdr, const uint8_t* dataBuf) OVERRIDE;
	p_data_t* ReadData() OVERRIDE;
	void SetSerialNumber(uint32_t serialNumber) OVERRIDE;
	std::string LogFileExtention() OVERRIDE { return std::string(".dat"); }
	void Flush() OVERRIDE;

	cDataChunk m_chunk;

private:
	p_data_t* ReadDataFromChunk();
	bool ReadChunkFromFile();
	bool WriteChunkToFile();
};

#endif // DEVICE_LOG_SERIAL_H
