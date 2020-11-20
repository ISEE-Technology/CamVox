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
#include "DataJSON.h"
#include "ISLogger.h"
#include "data_sets.h"
#include "ISDataMappings.h"
#include "ISUtilities.h"
#include "ISConstants.h"

#ifdef USE_IS_INTERNAL
#	include "../../libs/IS_internal.h"
#endif

int cDataJSON::WriteDataToFile(cISLogFileBase* pFile, const p_data_hdr_t& dataHdr, const uint8_t* dataBuf, const char* prefix)
{
	// Verify file pointer
	if (pFile == NULLPTR || cISDataMappings::GetSize(dataHdr.id) == 0)
	{
		return 0;
	}

	string s;
    if (!DataToStringJSON(dataHdr, dataBuf, s))
	{
		return 0;
	}
    if (prefix != NULLPTR)
    {
        pFile->puts(prefix);
    }
	pFile->puts(s.c_str());
    return (int)s.length();
}

bool cDataJSON::StringJSONToData(string& s, p_data_hdr_t& hdr, uint8_t* buf, uint32_t bufSize)
{
    (void)s;
    (void)buf;
    (void)bufSize;

    size_t pos = s.find("\"id\":");
    if (pos == string::npos)
    {
        return false;
    }
    uint32_t id = strtoul(s.c_str() + pos + 5, NULLPTR, 10);
    hdr.id = id;
	const map_name_to_info_t* offsetMap = cISDataMappings::GetMapInfo(hdr.id);
    if (offsetMap == NULLPTR)
	{
		return false;
	}
    hdr.size = cISDataMappings::GetSize(hdr.id);
	char c;
	char pc = 0;
	map_name_to_info_t::const_iterator offset;
	string fieldName;
	size_t fieldStart = 0;
	bool inName = true;
	bool inQuote = false;
	for (size_t i = 0; i < s.size(); i++)
	{
		c = s[i];
		if (c == '"' && pc != '\\')
		{
			if ((inQuote = !inQuote))
			{
				fieldStart = i + 1;
				pc = c;
				continue;
			}
		}

		if (inName)
		{
			if (c == ':')
			{
                fieldStart = i + 1;
				inName = inQuote = false;
				pc = c;
				continue;
			}
			else if (inQuote)
			{
				fieldName.append(1, c);
			}
		}
        else if ((c == '}' || c == '"' || (!inQuote && c == ',')) && pc != '\\')
		{
            if (fieldStart != 0)
			{
				offset = offsetMap->find(fieldName);
                string json = s.substr(fieldStart, i - fieldStart);
                if (offset != offsetMap->end() && !cISDataMappings::StringToData(json.c_str(), (int)json.size(), &hdr, buf, offset->second, 10, true))
				{
					return false;
				}
			}
            fieldName.clear();
            fieldStart = 0;
			inName = true;
		}
		pc = c;
	}

	return true;
}


bool cDataJSON::DataToStringJSON(const p_data_hdr_t& hdr, const uint8_t* buf, string& json)
{
    json.clear();
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

	SNPRINTF(tmp, sizeof(tmp), "{\"id\":%d", (int)hdr.id);
	json.append(tmp);
	for (map_name_to_info_t::const_iterator offset = offsetMap->begin(); offset != offsetMap->end(); offset++)
	{
		cISDataMappings::DataToString(offset->second, &hdrCopy, bufPtr, tmp, true);
		json.append(1, ',');
		json.append(1, '"');
		json.append(offset->second.name);
		json.append(1, '"');
		json.append(1, ':');
		json.append(tmp);
	}
	json.append(1, '}');
	return true;
}
