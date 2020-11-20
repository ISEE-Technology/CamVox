/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __ISDATAMAPPINGS_H_
#define __ISDATAMAPPINGS_H_

#include <string>
#include <map>
#include <inttypes.h>
#include "com_manager.h"
#include "DataCSV.h"

#ifndef CHAR_BIT
#define CHAR_BIT 8
#endif

using namespace std;

/**
* Get the size of an eDataType
* @param dataType the data type to get size for
* @return the size of the data type, or 0 if unknown or variable length (i.e. DataTypeString)
*/
uint32_t GetDataTypeSize(eDataType dataType);

#if !PLATFOM_IS_EMBEDDED

extern const unsigned char g_asciiToLowerMap[256];

#endif

/**
* Case-insensitive comparator for std::find and other functions
*/
struct sCaseInsensitiveCompare
{
	struct nocase_compare
	{
		bool operator() (const unsigned char& c1, const unsigned char& c2) const
		{
			return g_asciiToLowerMap[c1] < g_asciiToLowerMap[c2];
		}
	};

	/*
	size_t operator()(const std::string s) const
	{
		size_t hashCode = 5381;
		char c;
		const char* ptr = s.c_str();
		const char* ptrEnd = ptr + s.size();
		for (; ptr < ptrEnd; ptr++)
		{
			c = g_asciiToLowerMap[*ptr];
			hashCode = ((hashCode << 5) + hashCode) + c;
		}
		return hashCode;
	}
	*/

	// less than, not equal
	bool operator() (const std::string& s1, const std::string& s2) const
	{
		// return std::lexicographical_compare(s1.begin(), s1.end(), s2.begin(), s2.end(), nocase_compare());

		// we don't need unicode or fancy language handling here, and we do not want branching
		// so we have hand-coded a highly performant ASCII case insensitive compare here.
		// this custom code is 3x speed of lexicographical_compare
		char c1, c2;
		const char* ptr1 = s1.c_str();
		const char* ptr2 = s2.c_str();
		size_t size1 = s1.size();
		size_t size2 = s2.size();
		int sizeDiff = (int)size1 - (int)size2;

		// branchless min
		// y + ((x - y) & ((x - y) >> (sizeof(int) * CHAR_BIT - 1))); 
		size_t minSize = size2 + (sizeDiff & (sizeDiff >> (sizeof(int) * CHAR_BIT - 1)));

		for (size_t i = 0; i < minSize; i++)
		{
			c1 = g_asciiToLowerMap[(int)ptr1[i]];
			c2 = g_asciiToLowerMap[(int)ptr2[i]];
			if (c1 != c2)
			{
				return (c1 < c2);
			}
		}
		return (s1.size() < s2.size());
	}
};

/*
template <>
struct equal_to<std::string> : public unary_function<std::string, bool>
{
	bool operator()(const std::string& s1, const std::string& s2) const
	{
		// we don't need unicode or fancy language handling here, and we do not want branching
		// so we have hand-coded a highly performant ASCII case insensitive compare here.
		// this custom code is 3x speed of lexicographical_compare
		if (s1.size() != s2.size())
		{
			return false;
		}

		char c1, c2;
		const char* ptr1 = s1.c_str();
		const char* ptr2 = s2.c_str();

		for (size_t i = 0; i < s1.size(); i++)
		{
			c1 = g_asciiToLowerMap[ptr1[i]];
			c2 = g_asciiToLowerMap[ptr2[i]];
			if (c1 != c2)
			{
				return false;
			}
		}
		return true;
	}
};
*/

// map of field name to data info
typedef map<string, data_info_t, sCaseInsensitiveCompare> map_name_to_info_t;
typedef char data_mapping_string_t[IS_DATA_MAPPING_MAX_STRING_LENGTH];

class cISDataMappings
{
public:
	/**
	* Destructor
	*/
	virtual ~cISDataMappings();

	/**
	* Get a data set name from an id
	* @param dataId the data id to get a data set name from
	* @return data set name or NULL if not found
	*/
	static const char* GetDataSetName(uint32_t dataId);

	/**
	* Get the info for a data id
	* @return the info for the data id, or NULL if none found
	*/
	static const map_name_to_info_t* GetMapInfo(uint32_t dataId);

	/**
	* Get the size of a given data id
	* @param dataId the data id
	* @return the data id size or 0 if not found or unknown
	*/
	static uint32_t GetSize(uint32_t dataId);

	/**
	* Convert a string to a data field
	* @param stringBuffer the null terminated string to convert, must not be NULL
	* @param stringLength the number of chars in stringBuffer
	* @param hdr packet header, NULL means dataBuffer is the entire data structure
	* @param dataBuffer packet buffer
	* @param info metadata about the field to convert
	* @param radix (base 10, base 16, etc.) to use if the field is a number field, ignored otherwise
	* @param json true if json, false if csv
	* @return true if success, false if error
	*/
	static bool StringToData(const char* stringBuffer, int stringLength, const p_data_hdr_t* hdr, uint8_t* dataBuffer, const data_info_t& info, int radix = 10, bool json = false);

	/**
	* Convert data to a string
	* @param info metadata about the field to convert
	* @param hdr packet header, NULL means dataBuffer is the entire data structure
	* @param dataBuffer packet buffer
	* @param stringBuffer the buffer to hold the converted string
	* @param json true if json, false if csv
	* @return true if success, false if error
	*/
	static bool DataToString(const data_info_t& info, const p_data_hdr_t* hdr, const uint8_t* dataBuffer, data_mapping_string_t stringBuffer, bool json = false);

	/**
	* Get a timestamp from data if available
	* @param hdr data header
	* @param buf data buffer
	* @return timestamp, or 0.0 if no timestamp available
	*/
    static double GetTimestamp(const p_data_hdr_t* hdr, const uint8_t* buf);

	/**
	* Check whether field data can be retrieved given a data packet
	* @param info metadata for the field to get
	* @param hdr packet header
	* @param buf packet buffer
	* @param ptr receives the offset to get data at if the return value is true
	* @return true if the data can be retrieved, false otherwise
	*/
	static bool CanGetFieldData(const data_info_t& info, const p_data_hdr_t* hdr, const uint8_t* buf, const uint8_t*& ptr);

private:
	cISDataMappings();

	uint32_t m_lookupSize[DID_COUNT];
	const data_info_t* m_timestampFields[DID_COUNT];
	map_name_to_info_t m_lookupInfo[DID_COUNT];

#if PLATFORM_IS_EMBEDDED

	// on embedded we cannot new up C++ runtime until after free rtos has started
	static cISDataMappings* s_map;

#else

	static cISDataMappings s_map;

#endif

};

#endif // __ISDATAMAPPINGS_H_

