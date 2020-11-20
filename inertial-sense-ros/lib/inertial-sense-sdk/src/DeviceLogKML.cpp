/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <ctime>
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
#include "ISPose.h"
#include "DeviceLogKML.h"
#include "ISLogger.h"
#include "ISConstants.h"

using namespace std;

void cDeviceLogKML::InitDeviceForWriting(int pHandle, std::string timestamp, std::string directory, uint64_t maxDiskSpace, uint32_t maxFileSize)
{
	memset(&m_Log, 0, sizeof(m_Log));

	cDeviceLog::InitDeviceForWriting(pHandle, timestamp, directory, maxDiskSpace, maxFileSize);
}


bool cDeviceLogKML::CloseAllFiles()
{
    cDeviceLog::CloseAllFiles();

	for (int kid = 0; kid < cDataKML::MAX_NUM_KID; kid++)
	{
		// Close file
		CloseWriteFile(kid, m_Log[kid]);
	}

	return true;
}


bool cDeviceLogKML::CloseWriteFile(int kid, sKmlLog &log)
{
	// Ensure we have data
	int64_t dataSize = (int64_t)log.data.size();
	if (dataSize <= 0)
	{
		return false;
	}

	// Ensure directory exists
	if (m_directory.empty())
	{
		return false;
	}

	_MKDIR(m_directory.c_str());

	// Create filename
	log.fileCount++;
	int serNum = m_devInfo.serialNumber;
	if (!serNum)
	{
		serNum = m_pHandle;
	}

	log.fileName = GetNewFileName(serNum, log.fileCount, m_kml.GetDatasetName(kid).c_str());

#define BUF_SIZE	100
	char buf[BUF_SIZE];
	int styleCnt = 1;
	string altitudeMode;
	string iconScale = "0.5";
	string labelScale = "0.5";
	string iconUrl;

	if (m_showPointTimestamps)
		labelScale = "0.5";
	else
		labelScale = "0.01";

	// Altitude mode
	if(m_altClampToGround)
		altitudeMode = "clampToGround";
	else
		altitudeMode = "absolute";

	// Style
	string styleStr, colorStr;

	iconUrl = "http://maps.google.com/mapfiles/kml/shapes/shaded_dot.png";
	// 		iconUrl = "http://earth.google.com/images/kml-icons/track-directional/track-none.png";

	// colors are ABGR
	switch (kid)
	{
	case cDataKML::KID_INS:
		iconUrl = "http://earth.google.com/images/kml-icons/track-directional/track-0.png";
		colorStr = "ff00ffff";  // yellow
		break;
	case cDataKML::KID_GPS:
		colorStr = "ff0000ff";  // red
		break;
	case cDataKML::KID_GPS1:
		colorStr = "ffff0000";  // blue
		break;
    case cDataKML::KID_RTK:
		colorStr = "ffffff00";  // cyan
        break;
	case cDataKML::KID_REF:
		iconUrl = "http://earth.google.com/images/kml-icons/track-directional/track-0.png";
		colorStr = "ffff00ff";  // magenta
		break;
	}

	// Add XML version info and document
	TiXmlElement
		*Style, *LineStyle, *IconStyle, *LabelStyle,
		*Placemark, *Point, *LineString,
		*heading,
		*elem, *href;

	TiXmlDocument tDoc;

	TiXmlDeclaration* decl = new TiXmlDeclaration("1.0", "UTF-8", "");
	tDoc.LinkEndChild(decl);

	TiXmlElement* kml = new TiXmlElement("kml");
	tDoc.LinkEndChild(kml);

	TiXmlElement* Document = new TiXmlElement("Document");
	kml->LinkEndChild(Document);
	kml->SetAttribute("xmlns", "http://www.opengis.net/kml/2.2");
	kml->SetAttribute("xmlns:gx", "http://www.google.com/kml/ext/2.2");

	// Show sample (dot)
	if (m_showPoints)
	{
		// Style
		styleStr = string("stylesel_");

		Style = new TiXmlElement("Style");
		Style->SetAttribute("id", styleStr);
		Document->LinkEndChild(Style);

		// Icon style
		IconStyle = new TiXmlElement("IconStyle");
		Style->LinkEndChild(IconStyle);

		elem = new TiXmlElement("color");
		elem->LinkEndChild(new TiXmlText(colorStr));
		IconStyle->LinkEndChild(elem);

		elem = new TiXmlElement("colorMode");
		elem->LinkEndChild(new TiXmlText("normal"));
		IconStyle->LinkEndChild(elem);

		elem = new TiXmlElement("scale");
		elem->LinkEndChild(new TiXmlText(iconScale));
		IconStyle->LinkEndChild(elem);

		heading = new TiXmlElement("heading");
		heading->LinkEndChild(new TiXmlText("0"));
		IconStyle->LinkEndChild(heading);

		href = new TiXmlElement("href");
		href->LinkEndChild(new TiXmlText(iconUrl));
		elem = new TiXmlElement("Icon");
		elem->LinkEndChild(href);
		IconStyle->LinkEndChild(elem);

		// Label style
		LabelStyle = new TiXmlElement("LabelStyle");
		Style->LinkEndChild(LabelStyle);

		elem = new TiXmlElement("colorMode");
		elem->LinkEndChild(new TiXmlText("normal"));
		LabelStyle->LinkEndChild(elem);

		elem = new TiXmlElement("scale");
		elem->LinkEndChild(new TiXmlText(labelScale));
		LabelStyle->LinkEndChild(elem);


		double nextTime = log.data[0].time;
		for (size_t i = 0; i < log.data.size(); i++)
		{
			sKmlLogData& item = (log.data[i]);

			// Log only every iconUpdatePeriodSec
			if (item.time < nextTime)
			{
				continue;
			}
			nextTime = item.time - fmod(item.time, m_pointUpdatePeriodSec) + m_pointUpdatePeriodSec;

			// Placemark
			Placemark = new TiXmlElement("Placemark");
			Document->LinkEndChild(Placemark);

			// Name
			elem = new TiXmlElement("name");
			ostringstream timeStream;
			switch (kid)
			{
			case cDataKML::KID_GPS:
			case cDataKML::KID_GPS1:
			case cDataKML::KID_GPS2:
			case cDataKML::KID_RTK:
				timeStream << fixed << setprecision(1) << item.time;
				break;
			default:
				timeStream << fixed << setprecision(3) << item.time;
				break;
			}
			elem->LinkEndChild(new TiXmlText(timeStream.str()));
			Placemark->LinkEndChild(elem);

			// styleUrl
			elem = new TiXmlElement("styleUrl");
			if (item.theta[2] != 0.0f)
			{	// Style - to indicate heading
				string styleCntStr = styleStr + to_string(styleCnt++);
				TiXmlNode *StyleNode = Style->Clone();
				StyleNode->ToElement()->SetAttribute("id", styleCntStr);
				StyleNode->FirstChildElement("IconStyle")->FirstChildElement("heading")->FirstChild()->ToText()->SetValue(to_string(item.theta[2] * C_RAD2DEG_F));
				Document->LinkEndChild(StyleNode);

				elem->LinkEndChild(new TiXmlText("#" + styleCntStr));
			}
			else
			{
				elem->LinkEndChild(new TiXmlText("#" + styleStr));
			}
			Placemark->LinkEndChild(elem);

			Point = new TiXmlElement("Point");
			Placemark->LinkEndChild(Point);

			elem = new TiXmlElement("coordinates");
#if 1
			double lat = _CLAMP(item.lla[0] * DEG2RADMULT, -C_PIDIV2, C_PIDIV2) * RAD2DEGMULT;
			double lon = _CLAMP(item.lla[1] * DEG2RADMULT, -C_PI, C_PI) * RAD2DEGMULT;
			double alt = _CLAMP(item.lla[2], -1000, 100000);
			snprintf(buf, BUF_SIZE, "%.8lf,%.8lf,%.3lf", lon, lat, alt);
			elem->LinkEndChild(new TiXmlText(buf));
#else
			ostringstream coordinateStream;	// Not getting digits of precision
			coordinateStream
				<< fixed
				<< setprecision(8)
				<< item->lla[1] << ","  // Lat
				<< item->lla[0] << ","  // Lon
				<< setprecision(3)
				<< item->lla[2] << " "; // Alt
			elem->LinkEndChild(new TiXmlText(coordinateStream.str()));
#endif
			Point->LinkEndChild(elem);

			elem = new TiXmlElement("altitudeMode");
			elem->LinkEndChild(new TiXmlText(altitudeMode));
			Point->LinkEndChild(elem);
		}
	}// if (m_showSample)


	// Show path (track)
	if (m_showTracks)
	{
		// Track style
		styleStr = string("stylesel_") + to_string(styleCnt++);

		Style = new TiXmlElement("Style");
		Style->SetAttribute("id", styleStr);
		Document->LinkEndChild(Style);

		LineStyle = new TiXmlElement("LineStyle");
		Style->LinkEndChild(LineStyle);

		elem = new TiXmlElement("color");
		elem->LinkEndChild(new TiXmlText(colorStr));
		LineStyle->LinkEndChild(elem);

		elem = new TiXmlElement("colorMode");
		elem->LinkEndChild(new TiXmlText("normal"));
		LineStyle->LinkEndChild(elem);

		elem = new TiXmlElement("width");
		elem->LinkEndChild(new TiXmlText("2"));
		LineStyle->LinkEndChild(elem);

		// Track
		Placemark = new TiXmlElement("Placemark");
		Document->LinkEndChild(Placemark);

		elem = new TiXmlElement("name");
		elem->LinkEndChild(new TiXmlText("Tracks"));
		Placemark->LinkEndChild(elem);

		elem = new TiXmlElement("description");
		elem->LinkEndChild(new TiXmlText("SN tracks"));
		Placemark->LinkEndChild(elem);

		elem = new TiXmlElement("styleUrl");
		elem->LinkEndChild(new TiXmlText("#" + styleStr));
		Placemark->LinkEndChild(elem);

		LineString = new TiXmlElement("LineString");
		Placemark->LinkEndChild(LineString);

		ostringstream coordinateStream;
		int j = 0;
		for (size_t i = 0; i < log.data.size(); i++)
		{
            sKmlLogData& item = (log.data[i]);
            double lat = _CLAMP(item.lla[0] * DEG2RADMULT, -C_PIDIV2, C_PIDIV2) * RAD2DEGMULT;
            double lon = _CLAMP(item.lla[1] * DEG2RADMULT, -C_PI, C_PI) * RAD2DEGMULT;
            double alt = _CLAMP(item.lla[2], -1000, 100000);

			if (i >= log.data.size()-2)
			{
				j++;
			}

            snprintf(buf, BUF_SIZE, "%.8lf,%.8lf,%.3lf ", lon, lat, alt);
// 			if (strcmp("-111.65863637,40.05570543,1418.282 ", buf) == 0)
// 			{
// 				j++;
// 			}
// 			qDebug() << string(buf);
//			std::cout << "Value of str is : ";
			coordinateStream << string(buf);
		}

		elem = new TiXmlElement("coordinates");
		elem->LinkEndChild(new TiXmlText(coordinateStream.str()));
		LineString->LinkEndChild(elem);

		elem = new TiXmlElement("extrude");
		elem->LinkEndChild(new TiXmlText("1"));
		LineString->LinkEndChild(elem);

		elem = new TiXmlElement("altitudeMode");
		elem->LinkEndChild(new TiXmlText(altitudeMode));
		LineString->LinkEndChild(elem);
	}

	// Write XML to file
	if (!tDoc.SaveFile(log.fileName.c_str()))
	{
		return false;
	}

	// Remove data
	log.data.clear();

	return true;
}


bool cDeviceLogKML::OpenWithSystemApp(void)
{

#if PLATFORM_IS_WINDOWS

	for (int kid = 0; kid < cDataKML::MAX_NUM_KID; kid++)
	{
		sKmlLog &log = m_Log[kid];

		if (log.fileName.empty())
		{
			continue;
		}

		std::wstring stemp = std::wstring(log.fileName.begin(), log.fileName.end());
		LPCWSTR filename = stemp.c_str();
		ShellExecuteW(0, 0, filename, 0, 0, SW_SHOW);
	}

#endif

	return true;
}


bool cDeviceLogKML::SaveData(p_data_hdr_t *dataHdr, const uint8_t *dataBuf)
{
    cDeviceLog::SaveData(dataHdr, dataBuf);

	// Save data to file
    if (!WriteDateToFile(dataHdr, dataBuf))
	{
		return false;
	}

	// Convert formats
	uDatasets& d = (uDatasets&)(*dataBuf);
	switch (dataHdr->id)
	{
	case DID_INS_2:
		ins_1_t ins1;
		ins1.week = d.ins2.week;
		ins1.timeOfWeek = d.ins2.timeOfWeek;
		ins1.hdwStatus = d.ins2.hdwStatus;
		ins1.insStatus = d.ins2.insStatus;
		quat2euler(d.ins2.qn2b, ins1.theta);
		memcpy(ins1.uvw, d.ins2.uvw, 12);
        memcpy(ins1.lla, d.ins2.lla, 24);
		p_data_hdr_t dHdr = { DID_INS_1 , sizeof(ins_1_t), 0 };
		// Save data to file
        if (!WriteDateToFile(&dHdr, (uint8_t*)&ins1))
        {
            return false;
        }
		break;
	}

	return true;
}


bool cDeviceLogKML::WriteDateToFile(const p_data_hdr_t *dataHdr, const uint8_t* dataBuf)
{
	int kid = cDataKML::DID_TO_KID(dataHdr->id);

	// Only use selected data
	if (kid < 0)
	{
		return true;
	}

	// Reference current log
	sKmlLog &log = m_Log[kid];

	// Write date to file
    m_kml.WriteDataToFile(log.data, dataHdr, dataBuf);

	// File byte size
    int nBytes = (int)(log.data.size() * cDataKML::BYTES_PER_KID(kid));
	log.fileSize = nBytes;
	m_fileSize = _MAX(m_fileSize, log.fileSize);
	m_logSize = nBytes;

	if (log.fileSize >= m_maxFileSize)
	{
		// Close existing file
        if (!CloseWriteFile(kid, log))
        {
            return false;
        }
	}

	return true;
}


p_data_t* cDeviceLogKML::ReadData()
{
	p_data_t* data = NULL;

	// Read data from chunk
	while (!(data = ReadDataFromChunk()))
	{
		// Read next chunk from file
		if (!ReadChunkFromFile())
		{
			return NULL;
		}
	}

	// Read is good
    cDeviceLog::OnReadData(data);
	return data;
}


p_data_t* cDeviceLogKML::ReadDataFromChunk()
{
	return NULL;
}


bool cDeviceLogKML::ReadChunkFromFile()
{
	return true;
}


void cDeviceLogKML::SetSerialNumber(uint32_t serialNumber)
{
	m_devInfo.serialNumber = serialNumber;
}



