/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <ctime>
#include <time.h>
#include <string>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <inttypes.h>
#include <math.h>
#include "DataCSV.h"
#include "ISLogger.h"
#include "data_sets.h"
#include "ISDataMappings.h"
#include "ISUtilities.h"
#include "ISConstants.h"

#ifdef USE_IS_INTERNAL
#	include "../../libs/IS_internal.h"
#endif


int cDataCSV::WriteHeaderToFile(FILE* pFile, uint32_t id)
{
	// Verify file pointer
	if (pFile == NULL || id >= DID_COUNT)
	{
		return 0;
	}
	//map_lookup_name_t::const_iterator offsetMap = cISDataMappings::GetMap().find(id);
	const map_name_to_info_t* offsetMap = cISDataMappings::GetMapInfo(id);
	//if (offsetMap == cISDataMappings::GetMap().end())
	if (offsetMap == NULLPTR)
	{
		return 0;
	}
	string header("_ID_");
	for (map_name_to_info_t::const_iterator offset = offsetMap->begin(); offset != offsetMap->end(); offset++)
	{
		header += ",";
		header += offset->first;
	}
	header += "\n";
	fputs(header.c_str(), pFile);
	return (int)header.length();
}


int cDataCSV::ReadHeaderFromFile(FILE* pFile, uint32_t id, vector<data_info_t>& columnHeaders)
{
#if PLATFORM_IS_EMBEDDED    

    return 0;

#else

	(void)id;

	char line[8192];
	line[0] = '\0';
	if (fgets(line, _ARRAY_BYTE_COUNT(line), pFile) == NULL || ferror(pFile) != 0)
	{
		return 0;
	}
	const map_name_to_info_t* offsetMap = cISDataMappings::GetMapInfo(id);
	stringstream stream(line);
	string columnHeader;
	columnHeaders.clear();
	while (stream.good())
	{
		getline(stream, columnHeader, ',');
		while (columnHeader.length() > 0 && (columnHeader[columnHeader.size() - 1] == '\r' || columnHeader[columnHeader.size() - 1] == '\n'))
		{
			columnHeader.resize(columnHeader.size() - 1);
		}
		if (columnHeader.length() == 0)
		{
			columnHeader = "UNKNOWN";
		}
		if (columnHeader == "_ID_")
		{
			columnHeaders.push_back({ 0xFFFFFFFF, 0xFFFFFFFF, DataTypeInt64, "_ID_" });
		}
		else
		{
			map_name_to_info_t::const_iterator offset = offsetMap->find(columnHeader);
			if (offset == offsetMap->end())
			{
				columnHeaders.push_back({ 0xFFFFFFFF, 0xFFFFFFFF, DataTypeBinary, columnHeader });
			}
			else
			{
				columnHeaders.push_back(offset->second);
			}
		}
	}
	return (int)strnlen(line, _ARRAY_BYTE_COUNT(line));
#endif
}

int cDataCSV::WriteDataToFile(uint64_t orderId, FILE* pFile, const p_data_hdr_t& dataHdr, const uint8_t* dataBuf)
{
	// Verify file pointer
	if (pFile == NULL || cISDataMappings::GetSize(dataHdr.id) == 0)
	{
		return 0;
	}
	string s;
    if (!DataToStringCSV(dataHdr, dataBuf, s))
	{
		return 0;
	}
	char tmp[64];
	SNPRINTF(tmp, 64, "%llu", (long long unsigned int)orderId);
	fputs(tmp, pFile);
	fputc(',', pFile);
	fputs(s.c_str(), pFile);

	// return the data string length plus comma plus order id string
    return (int)s.length() + 1 + (int)strnlen(tmp, sizeof(tmp));
}

bool cDataCSV::StringCSVToData(string& s, p_data_hdr_t& hdr, uint8_t* buf, uint32_t bufSize, const vector<data_info_t>& columnHeaders)
{
	const map_name_to_info_t* offsetMap = cISDataMappings::GetMapInfo(hdr.id);
	if (offsetMap == NULLPTR || hdr.offset != 0 || hdr.size == 0 || bufSize < hdr.size)
	{
		return false;
	}
	while (s.length() > 0 && (s[s.size() - 1] == '\r' || s[s.size() - 1] == '\n'))
	{
		s.resize(s.size() - 1);
	}
	// for parsing logic, last comma will allow last field to parse properly
	s += ",";
	string columnData;
	string::const_iterator start = s.begin();
	uint32_t index = 0;
	hdr.offset = 0;
	hdr.size = cISDataMappings::GetSize(hdr.id);
	bool inQuotes = false;
	uint32_t foundQuotes = 0;
	memset(buf, 0, hdr.size);
	for (string::const_iterator i = start; i < s.end() && index < columnHeaders.size(); i++)
	{
		if (*i == ',' && !inQuotes)
		{
			// end field
			columnData = string(start + foundQuotes, i - foundQuotes);
			start = i + 1;
			const data_info_t& data = columnHeaders[index++];
            if (data.dataOffset < MAX_DATASET_SIZE && !cISDataMappings::StringToData(columnData.c_str(), (int)columnData.length(), &hdr, buf, data))
			{
				return false;
			}
			foundQuotes = false;
		}
		else if (*i == '"')
		{
			inQuotes = !inQuotes;
			foundQuotes |= (uint32_t)inQuotes;
		}
	}
	return true;
}


bool cDataCSV::DataToStringCSV(const p_data_hdr_t& hdr, const uint8_t* buf, string& csv)
{
	csv.clear();
	string columnData;
	const map_name_to_info_t* offsetMap = cISDataMappings::GetMapInfo(hdr.id);
	if (offsetMap == NULLPTR)
	{
		return false;
	}
	char tmp[IS_DATA_MAPPING_MAX_STRING_LENGTH];
	const uint8_t* bufPtr = buf;
	uint8_t tmpBuffer[MAX_DATASET_SIZE];
	uint32_t size = cISDataMappings::GetSize(hdr.id);
	if (size > hdr.size)
	{
		// copy into temp buffer, zeroing out bytes that are not part of this packet
		memset(tmpBuffer, 0, hdr.offset);
		memcpy(tmpBuffer + hdr.offset, buf, hdr.size);
		uint32_t dataEnd = hdr.offset + hdr.size;
		memset(tmpBuffer + dataEnd, 0, size - dataEnd);
		bufPtr = tmpBuffer;
	}

	// copy header as we are now storing an entire struct, even if we got a partial hdr and buf
	p_data_hdr_t hdrCopy = hdr;
	hdrCopy.offset = 0;
	hdrCopy.size = size;

	for (map_name_to_info_t::const_iterator offset = offsetMap->begin(); offset != offsetMap->end(); offset++)
	{
		cISDataMappings::DataToString(offset->second, &hdrCopy, bufPtr, tmp);
		if (csv.length() != 0)
		{
			csv += ",";
		}
		csv += tmp;
	}
	csv += "\n";
	return true;
}

