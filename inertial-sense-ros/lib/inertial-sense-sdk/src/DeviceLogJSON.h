/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef DEVICE_LOG_JSON_H
#define DEVICE_LOG_JSON_H

#include <stdio.h>
#include <string>
#include <vector>
#include <map>

#include "DataJSON.h"
#include "DeviceLog.h"
#include "com_manager.h"

class cDeviceLogJSON : public cDeviceLog
{
public:
	bool CloseAllFiles() OVERRIDE;
    bool SaveData(p_data_hdr_t* dataHdr, const uint8_t* dataBuf) OVERRIDE;
	p_data_t* ReadData() OVERRIDE;
	void SetSerialNumber(uint32_t serialNumber) OVERRIDE;
	std::string LogFileExtention() OVERRIDE { return std::string(".json"); }

private:
	bool GetNextItemForFile();

	p_data_t* ReadDataFromFile();
	string m_jsonString;
	cDataJSON m_json;
	p_data_t m_dataBuffer;
};

#endif // DEVICE_LOG_CSV_H
