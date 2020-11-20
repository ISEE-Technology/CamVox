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

#include "tinyxml.h"
#include "DataKML.h"
#include "ISLogger.h"
#include "ISPose.h"
#include "data_sets.h"
#include "ISUtilities.h"
#include "ISConstants.h"

#ifdef USE_IS_INTERNAL
#	include "../../libs/IS_internal.h"
#endif


cDataKML::cDataKML()
{
// 	def[0].init( "devInfo", "Device information" );
// 	def[5].init( "ins2", "Inertial navigation data with quaternion attitude" );
}

string cDataKML::GetDatasetName(int kid)
{
	switch (kid)
	{
	default:                    return "";
	case KID_INS:               return "ins";
	case KID_REF:               return "ref";
	case KID_GPS:               return "gps";
	case KID_GPS1:				return "gps1";
	case KID_GPS2:				return "gps2";
    case KID_RTK:               return "rtk";
	}
}


#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <time.h>

int cDataKML::WriteDataToFile(vector<sKmlLogData>& list, const p_data_hdr_t* dataHdr, const uint8_t* dataBuf)
{
	uDatasets& d = (uDatasets&)(*dataBuf);
	Euler theta;
    sKmlLogData data;

#ifdef USE_IS_INTERNAL
// 	uInternalDatasets &i = (uInternalDatasets&)(*dataBuf);
#endif

	// Write date to file
	switch (dataHdr->id)
	{
	default:		// Unidentified dataset
        return 0;

	case DID_INS_1:
        data = sKmlLogData(d.ins1.timeOfWeek, d.ins1.lla, d.ins1.theta);
		break;
	case DID_INS_2:
		quat2euler(d.ins2.qn2b, theta);
        data = sKmlLogData(d.ins2.timeOfWeek, d.ins2.lla, theta);
		break;
	case DID_INS_3:
		quat2euler(d.ins3.qn2b, theta);
        data = sKmlLogData(d.ins3.timeOfWeek, d.ins3.lla, theta);
		break;
	case DID_GPS1_POS:
	case DID_GPS1_UBX_POS:
	case DID_GPS2_POS:
        data = sKmlLogData(d.gpsPos.timeOfWeekMs, d.gpsPos.lla);
		break;
    case DID_GPS1_RTK_POS:
        data = sKmlLogData(d.gpsPos.timeOfWeekMs, d.gpsPos.lla);
        break;
	}

    list.push_back(data);

    return 0;
}


