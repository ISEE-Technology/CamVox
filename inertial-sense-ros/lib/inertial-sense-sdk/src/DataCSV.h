/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef DATA_CVS_H
#define DATA_CVS_H

#include <string>
#include <map>
#include <regex>

#include "com_manager.h"

using namespace std;

#define IS_DATA_MAPPING_MAX_STRING_LENGTH 2048

typedef enum
{
	DataTypeInt8,
	DataTypeUInt8,
	DataTypeInt16,
	DataTypeUInt16,
	DataTypeInt32,
	DataTypeUInt32,
	DataTypeInt64,
	DataTypeUInt64,
	DataTypeFloat,
	DataTypeDouble,
	DataTypeString,
	DataTypeBinary,

	DataTypeCount
} eDataType;

/*
* Metadata about a specific field
*/
typedef struct
{
	uint32_t dataOffset;
	uint32_t dataSize;
	eDataType dataType;
	string name;
} data_info_t;

class cDataCSV
{
public:
	int WriteHeaderToFile(FILE* pFile, uint32_t id);
	int ReadHeaderFromFile(FILE* pFile, uint32_t id, vector<data_info_t>& columnHeaders);
    int WriteDataToFile(uint64_t orderId, FILE* pFile, const p_data_hdr_t& dataHdr, const uint8_t* dataBuf);

	/**
	* Parse a csv string into a data packet
	* data needs the id set to the proper data id
	* buf memory to fill with data
	* bufSize size of available memory in buf
	* order id contains the value for ordering data
	* returns true if success, false if no map found
	*/
	bool StringCSVToData(string& s, p_data_hdr_t& hdr, uint8_t* buf, uint32_t bufSize, const vector<data_info_t>& columnHeaders);

	/**
	* Convert data to a csv string
	* buf is assumed to be large enough to hold the data structure
	* csv filled with csv data
	* return true if success, false if no map found
	*/
    bool DataToStringCSV(const p_data_hdr_t& hdr, const uint8_t* buf, string& csv);
};

#endif // DATA_CVS_H
