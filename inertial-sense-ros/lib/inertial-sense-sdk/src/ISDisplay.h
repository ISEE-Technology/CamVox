/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef IS_DISPLAY_H
#define IS_DISPLAY_H

#include <stdlib.h>
#include <inttypes.h>
#include <vector>
#include <string>

#include "com_manager.h"
#include "data_sets.h"
#include "ISConstants.h"

using namespace std;

/**
* Utility functions for displaying data
*/
class cInertialSenseDisplay
{
public:
	enum eDisplayMode
	{
		DMODE_PRETTY,
		DMODE_SCROLL,
		DMODE_STATS,
		DMODE_QUIET
	};

	cInertialSenseDisplay();

	void SetDisplayMode(eDisplayMode mode) { m_displayMode = mode; };
	eDisplayMode GetDisplayMode() { return m_displayMode; }
	void ShowCursor(bool visible);
	void ShutDown();
	void Clear(void);
	void Home(void);
	void GoToRow(int y);
	void GoToColumnAndRow(int x, int y);
	string Hello();
	string Connected();
	string Replay(double speed=1.0);
	string Goodbye();
	int ReadKey(); // non-block, returns -1 if no key available
	bool ControlCWasPressed();

	// for the binary protocol, this processes a packet of data
	void ProcessData(p_data_t *data, bool enableReplay = false, double replaySpeedX = 1.0);
	void DataToStats(const p_data_t* data);
	string DataToString(const p_data_t* data);
	char* StatusToString(char* ptr, char* ptrEnd, const uint32_t insStatus, const uint32_t hdwStatus);
	char* InsStatusToSolStatusString(char* ptr, char* ptrEnd, const uint32_t insStatus);
	string DataToStringINS1(const ins_1_t &ins1, const p_data_hdr_t& hdr);
	string DataToStringINS2(const ins_2_t &ins2, const p_data_hdr_t& hdr);
 	string DataToStringINS3(const ins_3_t &ins3, const p_data_hdr_t& hdr);
	string DataToStringINS4(const ins_4_t &ins4, const p_data_hdr_t& hdr);
	string DataToStringDualIMU(const dual_imu_t &imu, const p_data_hdr_t& hdr);
	string DataToStringIMU(const imu_t &imu, const p_data_hdr_t& hdr);
	string DataToStringPreintegratedImu(const preintegrated_imu_t &imu, const p_data_hdr_t& hdr);
	string DataToStringMag(const magnetometer_t &mag, const p_data_hdr_t& hdr);
	string DataToStringMagCal(const mag_cal_t &mag, const p_data_hdr_t& hdr);
	string DataToStringBaro(const barometer_t &baro, const p_data_hdr_t& hdr);
	string DataToStringGpsPos(const gps_pos_t &gps, const p_data_hdr_t& hdr, const string didName);
	string DataToStringRtkRel(const gps_rtk_rel_t &gps, const p_data_hdr_t& hdr, const string didName);
	string DataToStringRtkMisc(const gps_rtk_misc_t& sol, const p_data_hdr_t& hdr, const string didName);
	string DataToStringRawGPS(const gps_raw_t& raw, const p_data_hdr_t& hdr);
    string DataToStringSurveyIn(const survey_in_t &survey, const p_data_hdr_t& hdr);
	string DataToStringSysParams(const sys_params_t& sys, const p_data_hdr_t& hdr);
	string DataToStringSysSensors(const sys_sensors_t& sensors, const p_data_hdr_t& hdr);
	string DataToStringRTOS(const rtos_info_t& info, const p_data_hdr_t& hdr);
	string DataToStringDevInfo(const dev_info_t &info, const p_data_hdr_t& hdr);
	string DataToStringSensorsADC(const sys_sensors_adc_t &sensorsADC, const p_data_hdr_t& hdr);

private:
	string VectortoString();
	void DataToVector(const p_data_t* data);

	vector<string>	m_didMsgs;
	eDisplayMode m_displayMode;
	uint16_t m_rxCount;

	struct sDidStats
	{
		int lastTimeMs;
		int dtMs;
		int count;
	};

	vector<sDidStats> m_didStats;

#if PLATFORM_IS_WINDOWS

	HANDLE m_windowsConsoleIn;
	HANDLE m_windowsConsoleOut;

#endif

};

ostream& boldOn(ostream& os);
ostream& boldOff(ostream& os);
ostream& endlbOn(ostream& os);  // Bold on with newline
ostream& endlbOff(ostream& os); // Bold off with newline

#endif // IS_DISPLAY_H
