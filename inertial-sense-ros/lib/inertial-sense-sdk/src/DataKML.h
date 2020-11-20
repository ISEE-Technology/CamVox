/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef DATA_KML_H
#define DATA_KML_H

// #include <stdio.h>
#include <string>
#include <vector>

#include "tinyxml.h"
#include "com_manager.h"

#ifdef USE_IS_INTERNAL
#	include "../../libs/IS_internal.h"
#endif

using namespace std;

struct sKmlLogData
{
	double                  time;
	double                  lla[3];
	float                   theta[3];
    sKmlLogData() {}
	sKmlLogData(double _time, double _lla[3], float _theta[3])
	{
		time = _time;
        lla[0] = _lla[0];
        lla[1] = _lla[1];
        lla[2] = _lla[2];
        theta[0] = _theta[0];
        theta[1] = _theta[1];
        theta[2] = _theta[2];
	}

	sKmlLogData(unsigned int _timeMs, double _lla[3])
	{
		time = _timeMs*0.001;
        lla[0] = _lla[0];
        lla[1] = _lla[1];
        lla[2] = _lla[2];
        theta[0] = theta[1] = theta[2] = 0;
	}
};


class cDataKML
{
public:
	enum MyEnum
	{
		KID_INS = 0,
		KID_REF,
		KID_GPS,
		KID_GPS1,
		KID_GPS2,
        KID_RTK,
		MAX_NUM_KID,
	};

	static inline int DID_TO_KID(int did)
	{
		switch (did)
		{
		default:					return -1; // Unused
		case DID_INS_1:				return KID_INS;
		case DID_GPS1_POS:			return KID_GPS;
		case DID_GPS1_UBX_POS:			return KID_GPS1;
		case DID_GPS2_POS:			return KID_GPS2;
        case DID_GPS1_RTK_POS:       return KID_RTK;
		}
	}

	static inline int BYTES_PER_KID(int kid)
	{
		switch (kid)
		{
        default:
            return -1; // Unused
        case KID_INS:
            return 130;
		case KID_GPS:
		case KID_GPS1:
        case KID_GPS2:
        case KID_RTK:
            return 65;
		}
	}
	
	cDataKML();
	string GetDatasetName(int kid);
    int WriteDataToFile(vector<sKmlLogData>& list, const p_data_hdr_t* dataHdr, const uint8_t* dataBuf);
};

#endif // DATA_KML_H
