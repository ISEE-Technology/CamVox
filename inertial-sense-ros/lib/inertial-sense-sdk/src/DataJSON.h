/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef DATA_JSON_H
#define DATA_JSON_H

#include <string>
#include <map>
#include <regex>

#include "com_manager.h"
#include "ISLogFileBase.h"

using namespace std;

static inline bool IS_JSON_ESCAPE_CHAR(char c)
{
	switch (c)
	{
	case '"':
	case '\\':
	case '/':
	case '\b':
	case '\f':
	case '\n':
	case '\r':
	case '\t':
		return true;
	}
	return false;
}

class cDataJSON
{
public:
    /*
    * Write data to json file
    * pFile the file to write to
    * dataHdr data header
    * dataBuf data buffer
    * prefix prefix if data is written
    */
    int WriteDataToFile(cISLogFileBase* pFile, const p_data_hdr_t& dataHdr, const uint8_t* dataBuf, const char* prefix);

	/**
    * Parse a json string into a data packet
	* data needs the id set to the proper data id
	* buf memory to fill with data
	* bufSize size of available memory in buf
	* order id contains the value for ordering data
	* returns true if success, false if no map found
	*/
    bool StringJSONToData(string& s, p_data_hdr_t& hdr, uint8_t* buf, uint32_t bufSize);

	/**
    * Convert data to a json string
	* buf is assumed to be large enough to hold the data structure
    * json filled with json data
	* return true if success, false if no map found
	*/
    bool DataToStringJSON(const p_data_hdr_t& hdr, const uint8_t* buf, string& json);
};

#endif // DATA_JSON_H
