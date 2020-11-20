/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <math.h>

#include "ISConstants.h"
#include "ISUtilities.h"
#include "ISDisplay.h"
#include "ISPose.h"
#include "ISDataMappings.h"

#if PLATFORM_IS_WINDOWS

#include <conio.h>

#endif

#if PLATFORM_IS_LINUX || PLATFORM_IS_APPLE

#include <unistd.h>
#include <sys/time.h>
#include <termios.h>

#endif

#define PRINTV3_P1		"%8.1f,%8.1f,%8.1f\n"
#define PRINTV3_P2		" %8.2f,%8.2f,%8.2f\n"
#define PRINTV3_P3		"  %8.3f,%8.3f,%8.3f\n"
#define PRINTV4_P1		"%8.1f,%8.1f,%8.1f,%8.1f\n"
#define PRINTV4_P2		" %8.2f,%8.2f,%8.2f,%8.2f\n"
#define PRINTV4_P3		"  %8.3f,%8.3f,%8.3f,%8.3f\n"
#define PRINTV3_LLA		"%13.7f,%13.7f,%7.1f ellipsoid\n"
#define PRINTV3_LLA_MSL	"%13.7f,%13.7f,%7.1f MSL\n"
#define BUF_SIZE 8192

#define DISPLAY_DELTA_TIME	0	// show delta time instead of time

static bool s_controlCWasPressed;

#if PLATFORM_IS_WINDOWS

static bool ctrlHandler(DWORD fdwCtrlType)
{
	switch (fdwCtrlType)
	{
	case CTRL_C_EVENT:
	case CTRL_CLOSE_EVENT:
	case CTRL_BREAK_EVENT:
	case CTRL_LOGOFF_EVENT:
	case CTRL_SHUTDOWN_EVENT:
		s_controlCWasPressed = true;
		return true;
	default:
		return false;
	}
}

#else

#include <signal.h>

static void signalFunction(int sig)
{
    (void)sig;
	s_controlCWasPressed = true;
}

#endif

cInertialSenseDisplay::cInertialSenseDisplay()
{
	cout << endl << Hello() << endl;

	m_displayMode = DMODE_PRETTY;

#if PLATFORM_IS_WINDOWS

	// Hide cursor
	ShowCursor(false);

	if (!SetConsoleCtrlHandler((PHANDLER_ROUTINE)ctrlHandler, true))
	{
		std::cout << "Failed to set console ctrl handler!" << std::endl;
	}

#else

	signal(SIGINT, signalFunction);

#endif

}


void cInertialSenseDisplay::ShowCursor(bool visible)
{

#if PLATFORM_IS_WINDOWS

// 	m_windowsConsoleIn = GetStdHandle(STD_INPUT_HANDLE);
	m_windowsConsoleOut = GetStdHandle(STD_OUTPUT_HANDLE);
	CONSOLE_CURSOR_INFO cursorInfo;
	GetConsoleCursorInfo(m_windowsConsoleOut, &cursorInfo);
	cursorInfo.bVisible = visible;
	SetConsoleCursorInfo(m_windowsConsoleOut, &cursorInfo);

#endif

}


void cInertialSenseDisplay::ShutDown()
{
	ShowCursor(true);
// 	cout << "Shutting down..." << endl;
}


void cInertialSenseDisplay::Clear(void)
{

#if PLATFORM_IS_WINDOWS

	COORD topLeft = { 0, 0 };
	HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);
	CONSOLE_SCREEN_BUFFER_INFO screen;
	DWORD written;
	GetConsoleScreenBufferInfo(console, &screen);
	FillConsoleOutputCharacterA(console, ' ', screen.dwSize.X * screen.dwSize.Y, topLeft, &written);
	FillConsoleOutputAttribute(console, FOREGROUND_GREEN | FOREGROUND_RED | FOREGROUND_BLUE, screen.dwSize.X * screen.dwSize.Y, topLeft, &written);
	SetConsoleCursorPosition(console, topLeft);

#else

	printf( "\x1B[2J" ); // VT100 terminal command

#endif

}

void cInertialSenseDisplay::Home(void)
{

#if PLATFORM_IS_WINDOWS

	COORD topLeft = { 0, 0 };
	SetConsoleCursorPosition(m_windowsConsoleOut, topLeft);

#else

	printf( "\x1B[H" ); // VT100 terminal command

#endif

}

void cInertialSenseDisplay::GoToRow(int y)
{

#if PLATFORM_IS_WINDOWS

	COORD pos = { 0, (int16_t)y };
	SetConsoleCursorPosition(m_windowsConsoleOut, pos);

#else

	printf("\x1B[%dH", y); // VT100 terminal command

#endif

}

void cInertialSenseDisplay::GoToColumnAndRow(int x, int y)
{

#if PLATFORM_IS_WINDOWS

	COORD pos = { (int16_t)x, (int16_t)y };
	SetConsoleCursorPosition(m_windowsConsoleOut, pos);

#else

	printf("\x1B[%d;%df", y, x); // VT100 terminal command

#endif

}

string cInertialSenseDisplay::Hello()
{
	return "$ Inertial Sense.  Press CTRL-C to terminate.\n";
}

string cInertialSenseDisplay::Connected()
{
	return string("$ Inertial Sense.  Connected.  Press CTRL-C to terminate.  Rx ") + std::to_string(m_rxCount) + "\n";
}

string cInertialSenseDisplay::Replay(double speed)
{
	char buf[BUF_SIZE];

	SNPRINTF(buf, BUF_SIZE, "$ Inertial Sense.  Replay mode at %.1lfx speed.  Press CTRL-C to terminate.\n", speed);

	return buf;
}

string cInertialSenseDisplay::Goodbye()
{
	return "\nThanks for using Inertial Sense!\n";
}

int cInertialSenseDisplay::ReadKey()
{

#if PLATFORM_IS_WINDOWS

#if 0
    // This isn't working
    INPUT_RECORD ip;
	DWORD numRead = 0;
	PeekConsoleInputA(m_windowsConsoleIn, &ip, 1, &numRead);
	if (numRead == 1)
	{
		ReadConsoleInputA(m_windowsConsoleIn, &ip, 1, &numRead);
		if (numRead == 1 && ip.EventType == KEY_EVENT && ip.Event.KeyEvent.bKeyDown)
		{
			return ip.Event.KeyEvent.uChar.AsciiChar;
		}
	}
#else
    if (_kbhit())
    {
        return _getch();
    }
#endif

#else

	struct timeval tv;
	fd_set fds;
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	FD_ZERO(&fds);
	FD_SET(STDIN_FILENO, &fds);
	select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
	if (FD_ISSET(0, &fds))
	{
		struct termios oldt;
		struct termios newt;
		tcgetattr(STDIN_FILENO, &oldt); /* store old settings*/
		newt = oldt; /* copy old settings to new settings */
		newt.c_lflag &= ~(ICANON); /* change settings */
		tcsetattr(STDIN_FILENO, TCSANOW, &newt); /*apply the new settings immediatly */
		int ch = getchar(); /* standard getchar call */
		tcsetattr(STDIN_FILENO, TCSANOW, &oldt); /* reapply the old settings */
		return ch; /* return received char */
	}

#endif

	return -1;
}

bool cInertialSenseDisplay::ControlCWasPressed()
{
	return s_controlCWasPressed;
}

void cInertialSenseDisplay::ProcessData(p_data_t *data, bool enableReplay, double replaySpeedX)
{
	if (m_displayMode == DMODE_QUIET)
	{
		return;
	}

	int curTimeMs = current_weekMs();
	m_rxCount++;

	if (enableReplay)
	{
		static bool isTowMode = false;
		static int gpsTowMsOffset = 0;
		static int msgTimeMsOffset = 0;
		int msgTimeMs = 0;

		// Copy only new data
		uDatasets d = {};
		copyDataPToStructP(&d, data, sizeof(uDatasets));

		// Record message time.  In either ToW or time since boot.
		switch (data->hdr.id)
		{
			// Time of week - double
		case DID_INS_1:
		case DID_INS_2:				
			msgTimeMs = (int)(1000.0*d.ins1.timeOfWeek); 
			isTowMode = true;
			break;

			// Time of week - uint32 ms
		case DID_SYS_PARAMS:
			msgTimeMs = d.gpsPos.timeOfWeekMs;
			isTowMode = true;
			break;

		case DID_GPS1_POS:
        case DID_GPS1_RTK_POS:
            msgTimeMs = d.gpsPos.timeOfWeekMs;
			gpsTowMsOffset = (int)(1000.0*d.gpsPos.towOffset); 
			isTowMode = true; 
			break;

		case DID_GPS1_RTK_POS_REL:
			msgTimeMs = d.gpsRtkRel.timeOfWeekMs;
			isTowMode = true;
			break;

		case DID_GPS1_RTK_POS_MISC:
			msgTimeMs = d.gpsPos.timeOfWeekMs;
			gpsTowMsOffset = (int)(1000.0*d.gpsPos.towOffset);
			isTowMode = false;
			break;

			// Time since boot - double
		case DID_MAGNETOMETER_1:
		case DID_BAROMETER:
		case DID_SYS_SENSORS:
		case DID_PREINTEGRATED_IMU:
		case DID_DUAL_IMU:
		case DID_INL2_STATES:
		case DID_GPS_BASE_RAW:
			if( isTowMode )
				msgTimeMs = (int)(1000.0*d.imu.time) + gpsTowMsOffset; 
			else
				msgTimeMs = (int)(1000.0*d.imu.time);
			break;

			// Unidentified data type
// 		default: printf("Unknown DID %d\t", data->hdr.id);	return;			
		}


		// Control replay rate
		if (msgTimeMs != 0 && replaySpeedX>0.0)
		{
			for (;;)
			{
				curTimeMs = current_weekMs();

				// Replay speed
				int replayTimeMs = (int)(long)(((double)curTimeMs)*replaySpeedX);

				// Reinitialize message offset
				if (abs(msgTimeMs + msgTimeMsOffset - replayTimeMs) > 1500)
					msgTimeMsOffset = replayTimeMs - msgTimeMs;

				// Proceed if we're caught up
				if (replayTimeMs >= msgTimeMs + msgTimeMsOffset)
					break;

				// Add delay
// 				SLEEP_US(1000);
				SLEEP_MS(10);
			}
		}
	}


	static int timeSinceRefreshMs = 0;
	static int timeSinceClearMs = 0;
	static char idHist[DID_COUNT] = { 0 };
	if (m_displayMode != DMODE_SCROLL)
	{
		// Clear display every 2 seconds or if we start seeing new messages.
		if (curTimeMs - timeSinceClearMs > 2000 || curTimeMs < timeSinceClearMs || idHist[data->hdr.id] == 0)
		{
			Clear();
			idHist[data->hdr.id] = 1;
			timeSinceClearMs = curTimeMs;
		}
	}


	// Display Data
	switch (m_displayMode)
	{
	default:
		m_displayMode = DMODE_PRETTY;
		// fall through
	case DMODE_PRETTY:

		// Data stays at fixed location (no scroll history)
		DataToVector(data);

		// Limit printData display updates to 20Hz (50 ms)
		if (curTimeMs - timeSinceRefreshMs > 50 || curTimeMs < timeSinceRefreshMs)
		{
			Home();
			if (enableReplay)
				cout << Replay(replaySpeedX) << endl;
			else
				cout << Connected() << endl;

			cout << VectortoString();

			timeSinceRefreshMs = curTimeMs;
		}
		break;

	case DMODE_STATS:
		// Limit printData display updates to 20Hz (50 ms)
		if (curTimeMs - timeSinceRefreshMs > 50 || curTimeMs < timeSinceRefreshMs)
		{
			Home();
			cout << Connected() << endl;
			DataToStats(data);

			timeSinceRefreshMs = curTimeMs;
		}
		break;


	case DMODE_SCROLL:	// Scroll display 
		cout << DataToString(data) << endl;
		break;
	}



}

string cInertialSenseDisplay::VectortoString()
{
	stringstream ss;

	for (size_t i = 0; i < m_didMsgs.size(); i++)
	{
		if (m_didMsgs[i].size())
		{
			ss << m_didMsgs[i];
		}
	}

	return ss.str();
}

void cInertialSenseDisplay::DataToVector(const p_data_t* data)
{
	size_t id = data->hdr.id;
	if (m_didMsgs.size() <= id)
	{	// Resize vector if necessary
		m_didMsgs.resize(id + 1);
	}
	// Add string to vector
	m_didMsgs[id] = DataToString(data);
}

void cInertialSenseDisplay::DataToStats(const p_data_t* data)
{
	size_t id = data->hdr.id;
	if (m_didStats.size() <= id)
	{	// Resize vector if necessary
		m_didStats.resize(id + 1);
	}

	// Update stats
	int curTimeMs = current_weekMs();
	sDidStats &s = m_didStats[id];
	s.count++;
	if(s.lastTimeMs)
		s.dtMs = curTimeMs - s.lastTimeMs;
	s.lastTimeMs = curTimeMs;

	// Display stats
	printf("                Name  DID    Count        dt\n");
	for (int i = 0; i < (int)m_didStats.size(); i++)
	{
		s = m_didStats[i];
		if (s.count)
		{
			printf("%20s %4d %9d %9.3lf\n", cISDataMappings::GetDataSetName(i), i, s.count, s.dtMs*0.001);
		}
	}
}

string cInertialSenseDisplay::DataToString(const p_data_t* data)
{
	uDatasets d = {};

	// Copy only new data
	copyDataPToStructP(&d, data, sizeof(uDatasets));

	string str;
	switch (data->hdr.id)
	{
	case DID_DEV_INFO:          str = DataToStringDevInfo(d.devInfo, data->hdr);        break;
	case DID_DUAL_IMU:          str = DataToStringDualIMU(d.dualImu, data->hdr);        break;
	case DID_PREINTEGRATED_IMU: str = DataToStringPreintegratedImu(d.pImu, data->hdr);  break;
	case DID_INS_1:             str = DataToStringINS1(d.ins1, data->hdr);              break;
	case DID_INS_2:             str = DataToStringINS2(d.ins2, data->hdr);              break;
	case DID_INS_3:             str = DataToStringINS3(d.ins3, data->hdr);              break;
	case DID_INS_4:             str = DataToStringINS4(d.ins4, data->hdr);              break;
	case DID_MAGNETOMETER_1:
	case DID_MAGNETOMETER_2:    str = DataToStringMag(d.mag, data->hdr);                break;
	case DID_MAG_CAL:           str = DataToStringMagCal(d.magCal, data->hdr);          break;
	case DID_BAROMETER:         str = DataToStringBaro(d.baro, data->hdr);              break;
	case DID_GPS1_POS:          str = DataToStringGpsPos(d.gpsPos, data->hdr, "DID_GPS1_POS");				break;
	case DID_GPS2_POS:          str = DataToStringGpsPos(d.gpsPos, data->hdr, "DID_GPS2_POS");				break;
	case DID_GPS1_RTK_POS:      str = DataToStringGpsPos(d.gpsPos, data->hdr, "DID_GPS1_RTK_POS");			break;
	case DID_GPS1_RTK_POS_REL:  str = DataToStringRtkRel(d.gpsRtkRel, data->hdr, "DID_GPS1_RTK_POS_REL");	break;
	case DID_GPS1_RTK_POS_MISC: str = DataToStringRtkMisc(d.gpsRtkMisc, data->hdr, "RTK_POS_MISC");			break;
	case DID_GPS2_RTK_CMP_REL:  str = DataToStringRtkRel(d.gpsRtkRel, data->hdr, "DID_GPS2_RTK_CMP_REL");	break;
	case DID_GPS2_RTK_CMP_MISC: str = DataToStringRtkMisc(d.gpsRtkMisc, data->hdr, "RTK_CMP_MISC");			break;
	case DID_GPS1_RAW:
	case DID_GPS2_RAW:
	case DID_GPS_BASE_RAW:      str = DataToStringRawGPS(d.gpsRaw, data->hdr);          break;
    case DID_SURVEY_IN:         str = DataToStringSurveyIn(d.surveyIn, data->hdr);      break;
	case DID_SYS_PARAMS:        str = DataToStringSysParams(d.sysParams, data->hdr);    break;
	case DID_SYS_SENSORS:       str = DataToStringSysSensors(d.sysSensors, data->hdr);  break;
	case DID_RTOS_INFO:         str = DataToStringRTOS(d.rtosInfo, data->hdr);          break;
	case DID_SENSORS_ADC:       str = DataToStringSensorsADC(d.sensorsAdc, data->hdr);  break;
	default:
#if 0	// List all DIDs 
		char buf[128];
		SNPRINTF(buf, 128, "DID: %d\n", data->hdr.id);
		str = buf;
#endif
		break;
	}

	return str;
}

char* cInertialSenseDisplay::StatusToString(char* ptr, char* ptrEnd, const uint32_t insStatus, const uint32_t hdwStatus)
{
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tSTATUS\n");
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "\t\tSatellite Rx %d     Aiding: Mag %d, GPS (Hdg %d, Pos %d)\n",
		(hdwStatus & HDW_STATUS_GPS_SATELLITE_RX) != 0,
        (insStatus & INS_STATUS_MAG_AIDING_HEADING) != 0,
        (insStatus & INS_STATUS_GPS_AIDING_HEADING) != 0,
        (insStatus & INS_STATUS_GPS_AIDING_POS_VEL) != 0);
	if (insStatus & INS_STATUS_NAV_MODE)
	{
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\t\tMode: NAV ");
	}
	else
	{
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\t\tMode: AHRS");
	}
	switch (INS_STATUS_SOLUTION(insStatus))
	{
	default:
	case INS_STATUS_SOLUTION_OFF:                   ptr += SNPRINTF(ptr, ptrEnd - ptr, "         Solution: OFF\n");                 break;
	case INS_STATUS_SOLUTION_ALIGNING:              ptr += SNPRINTF(ptr, ptrEnd - ptr, "         Solution: ALIGNING\n");            break;
	case INS_STATUS_SOLUTION_ALIGNMENT_COMPLETE:    ptr += SNPRINTF(ptr, ptrEnd - ptr, "         Solution: ALIGNMENT COMPLETE\n");  break;
	case INS_STATUS_SOLUTION_NAV:                   ptr += SNPRINTF(ptr, ptrEnd - ptr, "         Solution: NAV\n");                 break;
	case INS_STATUS_SOLUTION_NAV_HIGH_VARIANCE:     ptr += SNPRINTF(ptr, ptrEnd - ptr, "         Solution: NAV HIGH VARIANCE\n");   break;
	case INS_STATUS_SOLUTION_AHRS:                  ptr += SNPRINTF(ptr, ptrEnd - ptr, "         Solution: AHRS\n");                break;
	case INS_STATUS_SOLUTION_AHRS_HIGH_VARIANCE:    ptr += SNPRINTF(ptr, ptrEnd - ptr, "         Solution: AHRS HIGH VARIANCE\n");  break;
	}
// 	ptr += SNPRINTF(ptr, ptrEnd - ptr, "         Align Good: Att %d, Vel %d, Pos %d\n",
// 		(insStatus & INS_STATUS_ATT_ALIGN_GOOD) != 0,
// 		(insStatus & INS_STATUS_VEL_ALIGN_GOOD) != 0,
// 		(insStatus & INS_STATUS_POS_ALIGN_GOOD) != 0);
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "\t\tErrors    Rx parse %d, temperature %d, self-test %d\n",
		HDW_STATUS_COM_PARSE_ERROR_COUNT(hdwStatus),
		(hdwStatus & HDW_STATUS_ERR_TEMPERATURE) != 0,
		(hdwStatus & HDW_STATUS_BIT_FAULT) != 0);

	return ptr;
}

char* cInertialSenseDisplay::InsStatusToSolStatusString(char* ptr, char* ptrEnd, const uint32_t insStatus)
{
	switch (INS_STATUS_SOLUTION(insStatus))
	{
	default:
	case INS_STATUS_SOLUTION_OFF:                   ptr += SNPRINTF(ptr, ptrEnd - ptr, ", OFF      ");	break;
	case INS_STATUS_SOLUTION_ALIGNING:              ptr += SNPRINTF(ptr, ptrEnd - ptr, ", ALIGNING ");	break;
	case INS_STATUS_SOLUTION_ALIGNMENT_COMPLETE:    ptr += SNPRINTF(ptr, ptrEnd - ptr, ", COMPLETE ");	break;
	case INS_STATUS_SOLUTION_NAV:                   ptr += SNPRINTF(ptr, ptrEnd - ptr, ", NAV      ");	break;
	case INS_STATUS_SOLUTION_NAV_HIGH_VARIANCE:		ptr += SNPRINTF(ptr, ptrEnd - ptr, ", NAV VARIA");	break;
	case INS_STATUS_SOLUTION_AHRS:                  ptr += SNPRINTF(ptr, ptrEnd - ptr, ", AHRS     ");	break;
	case INS_STATUS_SOLUTION_AHRS_HIGH_VARIANCE:    ptr += SNPRINTF(ptr, ptrEnd - ptr, ", AHRS VARI");	break;
	}

	return ptr;
}

string cInertialSenseDisplay::DataToStringINS1(const ins_1_t &ins1, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "DID_INS_1:");

#if DISPLAY_DELTA_TIME==1
	static double lastTime = 0;
	double dtMs = 1000.0*(ins1.timeOfWeek - lastTime);
	lastTime = ins1.timeOfWeek;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %.3lfs", ins1.timeOfWeek);
#endif

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		ptr = InsStatusToSolStatusString(ptr, ptrEnd, ins1.insStatus);
		ptr += SNPRINTF(ptr, ptrEnd - ptr, " theta[%6.2f,%6.2f,%7.2f], uvw[%6.2f,%6.2f,%6.2f], lla[%12.7f,%12.7f,%7.1f], ned[%6.3f,%6.3f,%6.3f]",
			ins1.theta[0] * C_RAD2DEG_F,
			ins1.theta[1] * C_RAD2DEG_F,
			ins1.theta[2] * C_RAD2DEG_F,
			ins1.uvw[0], ins1.uvw[1], ins1.uvw[2],
			ins1.lla[0], ins1.lla[1], ins1.lla[2],
			ins1.ned[0], ins1.ned[1], ins1.ned[2]);
	}
	else
	{	// Spacious format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n\tEuler\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P2,
			ins1.theta[0] * C_RAD2DEG_F,	// Roll
			ins1.theta[1] * C_RAD2DEG_F,	// Pitch
			ins1.theta[2] * C_RAD2DEG_F);	// Yaw
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tUWV\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P1,
			ins1.uvw[0],					// U body velocity
			ins1.uvw[1],					// V body velocity
			ins1.uvw[2]);					// W body velocity
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tLLA\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_LLA,
			ins1.lla[0],					// INS Latitude
			ins1.lla[1],					// INS Longitude
			ins1.lla[2]);					// INS Ellipsoid altitude (meters)
		ptr = StatusToString(ptr, ptrEnd, ins1.insStatus, ins1.hdwStatus);
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringINS2(const ins_2_t &ins2, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "DID_INS_2:");

#if DISPLAY_DELTA_TIME==1
	static double lastTime = 0;
	double dtMs = 1000.0*(ins2.timeOfWeek - lastTime);
	lastTime = ins2.timeOfWeek;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %.3lfs", ins2.timeOfWeek);
#endif

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		ptr = InsStatusToSolStatusString(ptr, ptrEnd, ins2.insStatus);
		ptr += SNPRINTF(ptr, ptrEnd - ptr, " qn2b[%6.3f,%6.3f,%6.3f,%6.3f], uvw[%6.2f,%6.2f,%6.2f], lla[%12.7f,%12.7f,%7.1f]",
			ins2.qn2b[0], ins2.qn2b[1], ins2.qn2b[2], ins2.qn2b[3],
			ins2.uvw[0], ins2.uvw[1], ins2.uvw[2],
			ins2.lla[0], ins2.lla[1], ins2.lla[2]);
	}
	else
	{	// Spacious format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n\tQn2b\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV4_P3,					// Quaternion attitude rotation
			ins2.qn2b[0],					// W
			ins2.qn2b[1],					// X
			ins2.qn2b[2],					// Y
			ins2.qn2b[3]);					// Z
		float theta[3];
		quat2euler(ins2.qn2b, theta);
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\t(Euler)\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P2,					// Convert quaternion to euler rotation
			theta[0] * C_RAD2DEG_F,			// Roll
			theta[1] * C_RAD2DEG_F,			// Pitch
			theta[2] * C_RAD2DEG_F);		// Yaw
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tUWV\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P1,
			ins2.uvw[0],					// U body velocity
			ins2.uvw[1],					// V body velocity
			ins2.uvw[2]);					// W body velocity
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tLLA\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_LLA,
			ins2.lla[0],					// INS Latitude
			ins2.lla[1],					// INS Longitude
			ins2.lla[2]);					// INS Ellipsoid altitude (meters)
		ptr = StatusToString(ptr, ptrEnd, ins2.insStatus, ins2.hdwStatus);
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringINS3(const ins_3_t &ins3, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "DID_INS_3:");

#if DISPLAY_DELTA_TIME==1
	static double lastTime = 0;
	double dtMs = 1000.0*(ins3.timeOfWeek - lastTime);
	lastTime = ins3.timeOfWeek;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %.3lfs", ins3.timeOfWeek);
#endif

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		ptr = InsStatusToSolStatusString(ptr, ptrEnd, ins3.insStatus);
		ptr += SNPRINTF(ptr, ptrEnd - ptr, " qn2b[%6.3f,%6.3f,%6.3f,%6.3f], uvw[%6.2f,%6.2f,%6.2f], lla[%12.7f,%12.7f,%7.1f]",
			ins3.qn2b[0], ins3.qn2b[1], ins3.qn2b[2], ins3.qn2b[3],
			ins3.uvw[0], ins3.uvw[1], ins3.uvw[2],
			ins3.lla[0], ins3.lla[1], ins3.lla[2]);
	}
	else
	{	// Spacious format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n\tQn2b\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV4_P3,					// Quaternion attitude rotation
			ins3.qn2b[0],					// W
			ins3.qn2b[1],					// X
			ins3.qn2b[2],					// Y
			ins3.qn2b[3]);					// Z
		float theta[3];
		quat2euler(ins3.qn2b, theta);
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\t(Euler)\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P2,					// Convert quaternion to euler rotation
			theta[0] * C_RAD2DEG_F,			// Roll
			theta[1] * C_RAD2DEG_F,			// Pitch
			theta[2] * C_RAD2DEG_F);		// Yaw
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tUWV\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P1,
			ins3.uvw[0],					// U body velocity
			ins3.uvw[1],					// V body velocity
			ins3.uvw[2]);					// W body velocity
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tLLA\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_LLA_MSL,
			ins3.lla[0],					// INS Latitude
			ins3.lla[1],					// INS Longitude
			ins3.lla[2]);					// INS Ellipsoid altitude (meters)
		ptr = StatusToString(ptr, ptrEnd, ins3.insStatus, ins3.hdwStatus);
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringINS4(const ins_4_t &ins4, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "DID_INS_4:");

#if DISPLAY_DELTA_TIME==1
	static double lastTime = 0;
	double dtMs = 1000.0*(ins4.timeOfWeek - lastTime);
	lastTime = ins4.timeOfWeek;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %.3lfs", ins4.timeOfWeek);
#endif

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		ptr = InsStatusToSolStatusString(ptr, ptrEnd, ins4.insStatus);
		ptr += SNPRINTF(ptr, ptrEnd - ptr, " qe2b[%6.3f,%6.3f,%6.3f,%6.3f], ve[%6.2f,%6.2f,%6.2f], ecef[%12.7f,%12.7f,%7.1f]",
			ins4.qe2b[0], ins4.qe2b[1], ins4.qe2b[2], ins4.qe2b[3],
			ins4.ve[0], ins4.ve[1], ins4.ve[2],
			ins4.ecef[0], ins4.ecef[1], ins4.ecef[2]);
	}
	else
	{	// Spacious format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n\tQe2b\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV4_P3,					// Quaternion attitude rotation
			ins4.qe2b[0],					// W
			ins4.qe2b[1],					// X
			ins4.qe2b[2],					// Y
			ins4.qe2b[3]);					// Z
		float theta[3];
		qe2b2EulerNedEcef(theta, (float*)ins4.qe2b, (double*)ins4.ecef);
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\t(Euler)\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P2,					// Convert quaternion to euler rotation
			theta[0] * C_RAD2DEG_F,			// Roll
			theta[1] * C_RAD2DEG_F,			// Pitch
			theta[2] * C_RAD2DEG_F);		// Yaw
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tVE\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P3,
			ins4.ve[0],						// X ECEF velocity
			ins4.ve[1],						// Y ECEF velocity
			ins4.ve[2]);					// Z ECEF velocity
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tECEF\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P3,
			ins4.ecef[0],					// X ECEF position
			ins4.ecef[1],					// Y ECEF position
			ins4.ecef[2]);					// Z ECEF position
		ptr = StatusToString(ptr, ptrEnd, ins4.insStatus, ins4.hdwStatus);
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringDualIMU(const dual_imu_t &imu, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "DID_DUAL_IMU:");

#if DISPLAY_DELTA_TIME==1
	static double lastTime = 0;
	double dtMs = 1000.0*(imu.time - lastTime);
	lastTime = imu.time;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %.3lfs", imu.time);
#endif

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		for (int i = 0; i < 2; i++)
		{
			ptr += SNPRINTF(ptr, ptrEnd - ptr, ", pqr[%5.1f,%5.1f,%5.1f]",
				imu.I[i].pqr[0] * C_RAD2DEG_F,
				imu.I[i].pqr[1] * C_RAD2DEG_F,
				imu.I[i].pqr[2] * C_RAD2DEG_F);
		}
		for (int i = 0; i < 2; i++)
		{
			ptr += SNPRINTF(ptr, ptrEnd - ptr, ", acc[%5.1f,%5.1f,%5.1f]",
				imu.I[i].acc[0], imu.I[i].acc[1], imu.I[i].acc[2]);
		}
	}
	else
	{	// Spacious format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n");
		for (int i = 0; i < 2; i++)
		{
			ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tPQR\t");
			ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P1,
				imu.I[i].pqr[0] * C_RAD2DEG_F,		// P angular rate
				imu.I[i].pqr[1] * C_RAD2DEG_F,		// Q angular rate
				imu.I[i].pqr[2] * C_RAD2DEG_F);		// R angular rate
			ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tAcc\t");
			ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P1,
				imu.I[i].acc[0],					// X acceleration
				imu.I[i].acc[1],					// Y acceleration
				imu.I[i].acc[2]);					// Z acceleration
		}
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringPreintegratedImu(const preintegrated_imu_t &imu, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "DID_PREINTEGRATED_IMU:");

#if DISPLAY_DELTA_TIME==1
	static double lastTime = 0;
	double dtMs = 1000.0*(imu.time - lastTime);
	lastTime = imu.time;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %.3lfs, dt:%6.3f", imu.time, imu.dt);
#endif

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, ", theta1[%6.3f,%6.3f,%6.3f], vel1[%6.3f,%6.3f,%6.3f]",
			imu.theta1[0] * C_RAD2DEG_F,
			imu.theta1[1] * C_RAD2DEG_F,
			imu.theta1[2] * C_RAD2DEG_F,
			imu.vel1[0], imu.vel1[1], imu.vel1[2]);
		ptr += SNPRINTF(ptr, ptrEnd - ptr, ", theta2[%6.3f,%6.3f,%6.3f], vel2[%6.3f,%6.3f,%6.3f]",
			imu.theta2[0] * C_RAD2DEG_F,
			imu.theta2[1] * C_RAD2DEG_F,
			imu.theta2[2] * C_RAD2DEG_F,
			imu.vel2[0], imu.vel2[1], imu.vel2[2]);
	}
	else
	{	// Spacious format
        ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n\tIMU1 theta1\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P3,
			imu.theta1[0] * C_RAD2DEG_F,		// IMU1 P angular rate
			imu.theta1[1] * C_RAD2DEG_F,		// IMU1 Q angular rate
			imu.theta1[2] * C_RAD2DEG_F);		// IMU1 R angular rate
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tIMU2 theta2\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P3,
			imu.theta2[0] * C_RAD2DEG_F,		// IMU2 P angular rate
			imu.theta2[1] * C_RAD2DEG_F,		// IMU2 Q angular rate
			imu.theta2[2] * C_RAD2DEG_F);		// IMU2 R angular rate
        ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tIMU1 vel1\t");
        ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P3,
            imu.vel1[0],						// IMU1 X acceleration
            imu.vel1[1],						// IMU1 Y acceleration
            imu.vel1[2]);						// IMU1 Z acceleration
        ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tIMU2 vel2\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P3,
			imu.vel2[0],						// IMU2 X acceleration
			imu.vel2[1],						// IMU2 Y acceleration
			imu.vel2[2]);						// IMU2 Z acceleration
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringMag(const magnetometer_t &mag, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	int i = 0;
	if (hdr.id == DID_MAGNETOMETER_2) i = 1;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "DID_MAGNETOMETER_%d:", i + 1);

#if DISPLAY_DELTA_TIME==1
	static double lastTime[2] = { 0 };
	double dtMs = 1000.0*(mag.time - lastTime[i]);
	lastTime[i] = mag.time;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %.3lfs", mag.time);
#endif

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, ", mag[%6.2f,%6.2f,%6.2f]",
			mag.mag[0],					// X magnetometer
			mag.mag[1],					// Y magnetometer
			mag.mag[2]);				// Z magnetometer
	}
	else
	{	// Spacious format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n\tmag\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P2,
			mag.mag[0],					// X magnetometer
			mag.mag[1],					// Y magnetometer
			mag.mag[2]);				// Z magnetometer
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringMagCal(const mag_cal_t &mag, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "DID_MAG_CAL:");

	switch (mag.recalCmd)
	{
	default:							ptr += SNPRINTF(ptr, ptrEnd - ptr, "  recalCmd %3d,               ", mag.recalCmd);	break;
	case MAG_RECAL_CMD_MULTI_AXIS:		ptr += SNPRINTF(ptr, ptrEnd - ptr, "  recalCmd %3d (MULTI-AXIS ), ", mag.recalCmd);	break;
	case MAG_RECAL_CMD_SINGLE_AXIS:		ptr += SNPRINTF(ptr, ptrEnd - ptr, "  recalCmd %3d (SINGLE-AXIS), ", mag.recalCmd);	break;
	case MAG_RECAL_CMD_ABORT:			ptr += SNPRINTF(ptr, ptrEnd - ptr, "  recalCmd %3d (ABORT      ), ", mag.recalCmd);	break;
	}

	{	// Single line format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "   progress: %3.0f %%,   declination: %4.1f",
			mag.progress,
			mag.declination * C_RAD2DEG_F);
	}

	if (m_displayMode == DMODE_PRETTY)
	{
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n");
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringBaro(const barometer_t &baro, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "DID_BAROMETER:");

#if DISPLAY_DELTA_TIME==1
	static double lastTime = 0;
	double dtMs = 1000.0*(baro.time - lastTime);
	lastTime = baro.time;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %.3lfs", baro.time);
#endif

	ptr += SNPRINTF(ptr, ptrEnd - ptr, ", %.2fkPa", baro.bar);
	ptr += SNPRINTF(ptr, ptrEnd - ptr, ", %.1fm", baro.mslBar);
	ptr += SNPRINTF(ptr, ptrEnd - ptr, ", %.2fC", baro.barTemp);
	ptr += SNPRINTF(ptr, ptrEnd - ptr, ", Humid. %.1f%%", baro.humidity);

	if (m_displayMode == DMODE_PRETTY)
	{
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n");
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringGpsPos(const gps_pos_t &gps, const p_data_hdr_t& hdr, const string didName)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;

	ptr += SNPRINTF(ptr, ptrEnd - ptr, "%s:", didName.c_str());

#if DISPLAY_DELTA_TIME==1
	static int lastTimeMs = 0;
	int dtMs = gps.timeOfWeekMs - lastTimeMs;
	lastTimeMs = gps.timeOfWeekMs;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %3dms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %dms", gps.timeOfWeekMs);
#endif

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, ", LLA[%12.7f,%12.7f,%7.1f], %d sats, %4.1f cno, %4.3f hAcc, %4.3f vAcc, %4.3f pDop",
			gps.lla[0], gps.lla[1], gps.lla[2],
			gps.status&GPS_STATUS_NUM_SATS_USED_MASK, gps.cnoMean,
			gps.hAcc, gps.vAcc, gps.pDop);
	}
	else
	{	// Spacious format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n\tSats: %2d,  ",
			gps.status&GPS_STATUS_NUM_SATS_USED_MASK);	// Satellites used in solution
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "Status: 0x%08x (", gps.status);
		switch (gps.status&GPS_STATUS_FIX_MASK)
		{
		default: 
		case GPS_STATUS_FIX_NONE:               ptr += SNPRINTF(ptr, ptrEnd - ptr, "%d", (gps.status&GPS_STATUS_FIX_MASK)>>GPS_STATUS_FIX_BIT_OFFSET);	break;
		case GPS_STATUS_FIX_2D:                 ptr += SNPRINTF(ptr, ptrEnd - ptr, "2D");           break;
		case GPS_STATUS_FIX_3D:                 ptr += SNPRINTF(ptr, ptrEnd - ptr, "3D");           break;
		case GPS_STATUS_FIX_RTK_SINGLE:         ptr += SNPRINTF(ptr, ptrEnd - ptr, "RTK Single");   break;
		case GPS_STATUS_FIX_RTK_FLOAT:          ptr += SNPRINTF(ptr, ptrEnd - ptr, "RTK Float");    break;
        case GPS_STATUS_FIX_RTK_FIX:            ptr += SNPRINTF(ptr, ptrEnd - ptr, "RTK FIX");      break;
        }
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "),\thAcc: %.3f m     cno: %3.1f dBHz\n", gps.hAcc, gps.cnoMean);	// Position accuracy
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tLLA: ");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_LLA,
			gps.lla[0],					// GPS Latitude
			gps.lla[1],					// GPS Longitude
			gps.lla[2]);				// GPS Ellipsoid altitude (meters)
		if (gps.status&GPS_STATUS_FLAGS_RTK_POSITION_ENABLED)
		{
			if (gps.status&GPS_STATUS_FLAGS_RTK_COMPASSING_ENABLED) 
			{ 
				ptr += SNPRINTF(ptr, ptrEnd - ptr, "Compassing, "); 
			}
			if (gps.status&GPS_STATUS_FLAGS_RTK_RAW_GPS_DATA_ERROR)		{ ptr += SNPRINTF(ptr, ptrEnd - ptr, "Raw error, "); }
			switch (gps.status&GPS_STATUS_FLAGS_ERROR_MASK)
			{
			case GPS_STATUS_FLAGS_RTK_BASE_DATA_MISSING:		ptr += SNPRINTF(ptr, ptrEnd - ptr, "Base missing, ");	break;
			case GPS_STATUS_FLAGS_RTK_BASE_POSITION_MOVING:		ptr += SNPRINTF(ptr, ptrEnd - ptr, "Moving base, ");	break;
			case GPS_STATUS_FLAGS_RTK_BASE_POSITION_INVALID:	ptr += SNPRINTF(ptr, ptrEnd - ptr, "Moving invalid, ");	break;
			}
		}
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringRtkRel(const gps_rtk_rel_t &rel, const p_data_hdr_t& hdr, const string didName)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;

	ptr += SNPRINTF(ptr, ptrEnd - ptr, "%s:", didName.c_str());

#if DISPLAY_DELTA_TIME==1
	static int lastTimeMs = 0;
	int dtMs = rel.timeOfWeekMs - lastTimeMs;
	lastTimeMs = rel.timeOfWeekMs;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %3dms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %dms", rel.timeOfWeekMs);
#endif

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, ", V2B[%10.3f,%10.3f,%9.2f], %4.1f age, %4.1f arRatio, %4.3f dist, %4.2f bear",
			rel.baseToRoverVector[0], rel.baseToRoverVector[1], rel.baseToRoverVector[2],
			rel.differentialAge, rel.arRatio, rel.baseToRoverDistance, rel.baseToRoverHeading);
	}
	else
	{	// Spacious format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tvectorToRover: ");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P3,
			rel.baseToRoverVector[0],				// Vector to base in ECEF
			rel.baseToRoverVector[1],				// Vector to base in ECEF
			rel.baseToRoverVector[2]);				// Vector to base in ECEF
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tRTK:\tdiffAge:%5.1fs  arRatio: %4.1f  dist:%7.2fm  bear:%6.1f\n", 
			rel.differentialAge, rel.arRatio, rel.baseToRoverDistance, rel.baseToRoverHeading*C_RAD2DEG_F);
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringRtkMisc(const gps_rtk_misc_t& rtk, const p_data_hdr_t& hdr, const string didName)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	const char* terminator = (m_displayMode != DMODE_SCROLL ? "\n" : "");
	ptr += SNPRINTF(buf, ptrEnd - ptr, "%s: T=%d, lla[%4.7f,%4.7f,%7.3f], A[%3.3f,%3.3f,%3.3f], AR:%3.3f, dop(g,h,v)[%3.3f,%3.3f,%3.3f] %s",
		didName.c_str(),
		rtk.timeOfWeekMs, rtk.baseLla[0], rtk.baseLla[1], rtk.baseLla[2],
		rtk.accuracyPos[0], rtk.accuracyPos[1], rtk.accuracyPos[2],
		rtk.arThreshold,
		rtk.gDop, rtk.hDop, rtk.vDop,
		terminator);

	if (m_displayMode != DMODE_SCROLL)
	{
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n");
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringRawGPS(const gps_raw_t& raw, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	const char* terminator = (m_displayMode != DMODE_SCROLL ? "\n" : "");
	ptr += SNPRINTF(buf, ptrEnd - ptr, "RAW GPS: receiverIndex=%d, type=%d, count=%d   %s",
		raw.receiverIndex, raw.dataType, raw.obsCount, terminator);

	if (m_displayMode != DMODE_SCROLL)
	{
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n");
	}
	return buf;
}

string cInertialSenseDisplay::DataToStringSurveyIn(const survey_in_t &survey, const p_data_hdr_t& hdr)
{
    (void)hdr;
    char buf[BUF_SIZE];
    char* ptr = buf;
    char* ptrEnd = buf + BUF_SIZE;
//     int i = 0;
    ptr += SNPRINTF(ptr, ptrEnd - ptr, "DID_SURVEY_IN:");

    ptr += SNPRINTF(ptr, ptrEnd - ptr, " state: %d ", survey.state);
    switch (survey.state)
    {
    case SURVEY_IN_STATE_OFF:           ptr += SNPRINTF(ptr, ptrEnd - ptr, "(off)");           break;
    case SURVEY_IN_STATE_RUNNING_3D:    ptr += SNPRINTF(ptr, ptrEnd - ptr, "(running 3D)");    break;
    case SURVEY_IN_STATE_RUNNING_FLOAT: ptr += SNPRINTF(ptr, ptrEnd - ptr, "(running Float)"); break;
    case SURVEY_IN_STATE_RUNNING_FIX:   ptr += SNPRINTF(ptr, ptrEnd - ptr, "(running Fix)");   break;
    case SURVEY_IN_STATE_SAVE_POS:      ptr += SNPRINTF(ptr, ptrEnd - ptr, "(saving pos)");    break;
    case SURVEY_IN_STATE_DONE:          ptr += SNPRINTF(ptr, ptrEnd - ptr, "(done)");          break;
    }

    int elapsedTimeMin = survey.elapsedTimeSec / 60;
    int elapsedTimeSec = survey.elapsedTimeSec - (elapsedTimeMin * 60);
    int maxDurationMin = survey.maxDurationSec / 60;
    int maxDurationSec = survey.maxDurationSec - (maxDurationMin * 60);

    ptr += SNPRINTF(ptr, ptrEnd - ptr, ", elapsed: %d:%02d of %2d:%02d", 
        elapsedTimeMin, elapsedTimeSec, maxDurationMin, maxDurationSec );
    if (m_displayMode != DMODE_SCROLL)
    {
        ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n\thAcc: %4.3f\tlla:", survey.hAccuracy);
        ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_LLA,
            survey.lla[0],					// latitude
            survey.lla[1],					// longitude
            survey.lla[2]);					// altitude
    }
    else
    {   // Single line format
        ptr += SNPRINTF(ptr, ptrEnd - ptr, ", hAcc: %4.3f ", survey.hAccuracy);
        ptr += SNPRINTF(ptr, ptrEnd - ptr, " lla[%12.7f,%12.7f,%7.1f]",
            survey.lla[0],					// latitude
            survey.lla[1],					// longitude
            survey.lla[2]);					// altitude
    }
    return buf;
}

string cInertialSenseDisplay::DataToStringSysParams(const sys_params_t& sys, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "DID_SYS_PARAMS:");

#if DISPLAY_DELTA_TIME==1
	static int lastTimeMs = 0;
	int dtMs = sys.timeOfWeekMs - lastTimeMs;
	lastTimeMs = sys.timeOfWeekMs;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %3dms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %dms", sys.timeOfWeekMs);
#endif

	ptr += SNPRINTF(ptr, ptrEnd - ptr, ",%d,%d,%d\n", sys.imuPeriodMs, sys.navPeriodMs, sys.genFaultCode);

	if (m_displayMode == DMODE_PRETTY)
	{
		ptr = StatusToString(ptr, ptrEnd, sys.insStatus, sys.hdwStatus);
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tTemp:  IMU %4.1f C\tBaro %4.1f C\tMCU %4.1f C\n", sys.imuTemp, sys.baroTemp, sys.mcuTemp);
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringSysSensors(const sys_sensors_t& sensors, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "DID_SYS_SENSORS:");

#if DISPLAY_DELTA_TIME==1
	static double lastTime = 0;
	double dtMs = 1000.0*(sensors.time - lastTime);
	lastTime = sensors.time;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %.3lfs", sensors.time);
#endif

	// Single line format
	ptr += SNPRINTF(ptr, ptrEnd - ptr, ", %4.1fC, pqr[%5.1f,%5.1f,%5.1f], acc[%5.1f,%5.1f,%5.1f], mag[%6.2f,%6.2f,%6.2f]",
		sensors.temp,
		sensors.pqr[0] * C_RAD2DEG_F,
		sensors.pqr[1] * C_RAD2DEG_F,
		sensors.pqr[2] * C_RAD2DEG_F,
		sensors.acc[0], sensors.acc[1], sensors.acc[2],
		sensors.mag[0], sensors.mag[1], sensors.mag[2]
	);

	ptr += SNPRINTF(ptr, ptrEnd - ptr, ", baro[%5.2fkPa, %4.1fC, %7.2fm, %3.1f%% humidity], adc[%3.1fV, %3.1fV, %3.1fV, %3.1fV]",
		sensors.bar, sensors.barTemp, sensors.mslBar, sensors.humidity,
		sensors.vin, sensors.ana1, sensors.ana3, sensors.ana4
	);

	if (m_displayMode != DMODE_SCROLL)
	{
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n");
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringRTOS(const rtos_info_t& info, const p_data_hdr_t& hdr)
{
	cDataCSV csv;
	string csvString;
	csv.DataToStringCSV(hdr, (const uint8_t*)&info, csvString);
	const char* terminator = (m_displayMode != DMODE_SCROLL ? "\n" : "");
	return string("RTOS: ") + csvString + terminator;
}

string cInertialSenseDisplay::DataToStringDevInfo(const dev_info_t &info, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "DID_DEV_INFO:");

	// Single line format
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " SN%d, Fw %d.%d.%d.%d %c%d, %04d-%02d-%02d",
		info.serialNumber,
		info.firmwareVer[0],
		info.firmwareVer[1],
		info.firmwareVer[2],
		info.firmwareVer[3],
		info.buildDate[0],
		info.buildNumber,
		info.buildDate[1] + 2000,
		info.buildDate[2],
		info.buildDate[3]
	);

	if (m_displayMode != DMODE_SCROLL)
	{	// Spacious format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, " %02d:%02d:%02d, Proto %d.%d.%d.%d\n",
			info.buildTime[0],
			info.buildTime[1],
			info.buildTime[2],
			info.protocolVer[0],
			info.protocolVer[1],
			info.protocolVer[2],
			info.protocolVer[3]
		);
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringSensorsADC(const sys_sensors_adc_t &sensorsADC, const p_data_hdr_t &hdr) {
    (void) hdr; // hdr is not used

	stringstream ss;
	ss << "DID_SENSORS_ADC:";
	ss << fixed;
    ss << "time " << setprecision(3) << sensorsADC.time << ", ";
    ss << "bar " << setprecision(2) << sensorsADC.bar << ", ";
    ss << "barTemp " << setprecision(2) << sensorsADC.barTemp << ", ";
    ss << "humidity " << setprecision(2) << sensorsADC.humidity << ", ";

//     ss << " ana[" << setprecision(2);
//     for (size_t i = 0; i < NUM_ANA_CHANNELS; ++i)
//     {
//         if (i != 0) { ss << ", "; }
//         ss << sensorsADC.ana[i];
//     }
//     ss << "]";

	if (m_displayMode != DMODE_SCROLL)
	{    // Spacious format
		ss << "\n";
#define SADC_WIDTH	5
		for (size_t i = 0; i < NUM_IMU_DEVICES; ++i)
		{
			auto &mpu = sensorsADC.mpu[i];
			ss << "\tmpu[" << i << "]: " << setprecision(0);
			ss << "pqr[" << setw(SADC_WIDTH) << mpu.pqr[0] << "," << setw(SADC_WIDTH) << mpu.pqr[1] << "," << setw(SADC_WIDTH) << mpu.pqr[2] << "], ";
			ss << "acc[" << setw(SADC_WIDTH) << mpu.acc[0] << "," << setw(SADC_WIDTH) << mpu.acc[1] << "," << setw(SADC_WIDTH) << mpu.acc[2] << "], ";
			ss << "mag[" << setw(SADC_WIDTH) << mpu.mag[0] << "," << setw(SADC_WIDTH) << mpu.mag[1] << "," << setw(SADC_WIDTH) << mpu.mag[2] << "], ";
			ss << "temp " << setprecision(3) << mpu.temp << ",";
			ss << "\n";
		}
	}
	else
	{
		for (size_t i = 0; i < NUM_IMU_DEVICES; ++i)
		{
			auto &mpu = sensorsADC.mpu[i];
			ss << "mpu[" << i << "]: " << setprecision(0);
			ss << "pqr[" << mpu.pqr[0] << "," << mpu.pqr[1] << "," << mpu.pqr[2] << "], ";
			ss << "acc[" << mpu.acc[0] << "," << mpu.acc[1] << "," << mpu.acc[2] << "], ";
			ss << "mag[" << mpu.mag[0] << "," << mpu.mag[1] << "," << mpu.mag[2] << "], ";
			ss << "temp " << setprecision(3) << mpu.temp << ",";
		}
	}

	return ss.str();
}


ostream& boldOn(ostream& os)
{

#if PLATFORM_IS_WINDOWS

	return os;

#else

	return os << "\033[1m";

#endif

}

ostream& boldOff(ostream& os)
{

#if PLATFORM_IS_WINDOWS

	return os;

#else

	return os << "\033[0m";

#endif

}

// Bold on with newline
ostream& endlbOn(ostream& os)
{

#if PLATFORM_IS_WINDOWS

	return os << endl;

#else

	return os << endl << boldOn;

#endif

}

// Bold off with newline
ostream& endlbOff(ostream& os)
{

#if PLATFORM_IS_WINDOWS

	return os << endl;

#else

	return os << endl << boldOff;

#endif

}
