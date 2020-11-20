#include <stdint.h>
#include "protocol_nmea.h"
#include "ISPose.h"
#include "ISEarth.h"


//////////////////////////////////////////////////////////////////////////
// Utility functions
//////////////////////////////////////////////////////////////////////////

uint32_t ASCII_compute_checksum(uint8_t* str, int size)
{
	uint32_t checksum = 0;
	
	uint8_t *end = str + size;
	for(uint8_t *ptr=str; ptr<end; ptr++)
	{
		checksum ^= *ptr;
	}

	return checksum;
}

// All strings must be NULL terminated!
char *ASCII_find_next_field(char *str)
{
	while(*str != 0 && *str != ',') //move down looking for end of string.
	++str;

	if(*str == ',') //move past comma (if not at end of string)
	++str;

	return str;
}

double ddmm2deg(double ddmm)
{
	double deg = (int)ddmm / 100 ;
	ddmm -= deg * 100 ;
	return deg + (ddmm / 60) ;
}

void set_gpsPos_status_mask(uint32_t *status, uint32_t state, uint32_t mask)
{
	*status &= ~mask;
	*status |= state & mask;
}

/* convert calendar day/time to time -------------------------------------------
* convert calendar day/time to gtime_t struct
* args   : double *ep       I   day/time {year,month,day,hour,min,sec}
* return : gtime_t struct
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
gtime_t epoch2time(const double *ep)
{
	const int doy[]={1,32,60,91,121,152,182,213,244,274,305,335};
	gtime_t time={0};
	int days,sec,year=(int)ep[0],mon=(int)ep[1],day=(int)ep[2];
	
	if (year<1970||2099<year||mon<1||12<mon) return time;
	
	/* leap year if year%4==0 in 1901-2099 */
	days=(year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+(year%4==0&&mon>=3?1:0);
	sec=(int)floor(ep[5]);
	time.time=(time_t)days*86400+(int)ep[3]*3600+(int)ep[4]*60+sec;
	time.sec=ep[5]-sec;
	return time;
}

static const double gpst0[]={1980,1, 6,0,0,0}; /* gps time reference */

/* time to gps time ------------------------------------------------------------
* convert gtime_t struct to week and tow in gps time
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gps time (NULL: no output)
* return : time of week in gps time (s)
*-----------------------------------------------------------------------------*/
double time2gpst(gtime_t t, int *week)
{
	gtime_t t0=epoch2time(gpst0);
	time_t sec=t.time-t0.time;
	time_t w=(time_t)(sec/(86400*7));
	
	if (week) *week=(int)w;
	return (double)(sec-(double)w*86400*7)+t.sec;
}


//////////////////////////////////////////////////////////////////////////
// DID to NMEA
//////////////////////////////////////////////////////////////////////////

int dev_info_to_nmea_info(char a[], const int aSize, dev_info_t &info)
{
//     unsigned int checkSum = 0;
//     serWrite(portNum, (unsigned char*)"$", 1);
//     ASCII_PORT_WRITE_NO_FORMAT(portNum, "INFO", checkSum, 4);
//     ASCII_PORT_WRITE(portNum, a, checkSum, ",%d", devInfo.serialNumber);      // 1
//     ASCII_PORT_WRITE(portNum, a, checkSum, ",%d", devInfo.hardwareVer[0]);    // 2
//     ASCII_PORT_WRITE(portNum, a, checkSum, ".%d", devInfo.hardwareVer[1]);
//     ASCII_PORT_WRITE(portNum, a, checkSum, ".%d", devInfo.hardwareVer[2]);
//     ASCII_PORT_WRITE(portNum, a, checkSum, ".%d", devInfo.hardwareVer[3]);
//     ASCII_PORT_WRITE(portNum, a, checkSum, ",%d", devInfo.firmwareVer[0]);    // 3
//     ASCII_PORT_WRITE(portNum, a, checkSum, ".%d", devInfo.firmwareVer[1]);
//     ASCII_PORT_WRITE(portNum, a, checkSum, ".%d", devInfo.firmwareVer[2]);
//     ASCII_PORT_WRITE(portNum, a, checkSum, ".%d", devInfo.firmwareVer[3]);
//     ASCII_PORT_WRITE(portNum, a, checkSum, ",%d", devInfo.buildNumber);       // 4
//     ASCII_PORT_WRITE(portNum, a, checkSum, ",%d", devInfo.protocolVer[0]);    // 5
//     ASCII_PORT_WRITE(portNum, a, checkSum, ".%d", devInfo.protocolVer[1]);
//     ASCII_PORT_WRITE(portNum, a, checkSum, ".%d", devInfo.protocolVer[2]);
//     ASCII_PORT_WRITE(portNum, a, checkSum, ".%d", devInfo.protocolVer[3]);
//     ASCII_PORT_WRITE(portNum, a, checkSum, ",%d", devInfo.repoRevision);      // 6
//     ASCII_PORT_WRITE(portNum, a, checkSum, ",%s", devInfo.manufacturer);      // 7
//     ASCII_PORT_WRITE(portNum, a, checkSum, ",%04d", devInfo.buildDate[1]);    // 8
//     ASCII_PORT_WRITE(portNum, a, checkSum, "-%02d", devInfo.buildDate[2]);
//     ASCII_PORT_WRITE(portNum, a, checkSum, "-%02d", devInfo.buildDate[3]);
//     ASCII_PORT_WRITE(portNum, a, checkSum, ",%02d", devInfo.buildTime[0]);    // 9
//     ASCII_PORT_WRITE(portNum, a, checkSum, ":%02d", devInfo.buildTime[1]);
//     ASCII_PORT_WRITE(portNum, a, checkSum, ":%02d", devInfo.buildTime[2]);
//     ASCII_PORT_WRITE(portNum, a, checkSum, ".%02d", devInfo.buildTime[3]);
//     ASCII_PORT_WRITE(portNum, a, checkSum, ",%s", devInfo.addInfo);           // 10    
//     ASCII_PORT_WRITE_NO_CHECKSUM(portNum, a, "*%.2x\r\n", checkSum);

	int n = SNPRINTF(a, aSize, "$INFO"
		",%d"			// 1
		",%d.%d.%d.%d"	// 2
		",%d.%d.%d.%d"	// 3
		",%d"			// 4
		",%d.%d.%d.%d"	// 5
		",%d"			// 6
		",%s"			// 7
		",%04d-%02d-%02d"		// 8
		",%02d:%02d:02%d.02%d"	// 9
		",%s",			// 10
		(int)info.serialNumber,	// 1
		info.hardwareVer[0], info.hardwareVer[1], info.hardwareVer[2], info.hardwareVer[3], // 2
		info.firmwareVer[0], info.firmwareVer[1], info.firmwareVer[2], info.firmwareVer[3], // 3
		(int)info.buildNumber,	// 4
		info.protocolVer[0], info.protocolVer[1], info.protocolVer[2], info.protocolVer[3], // 5
		(int)info.repoRevision,	// 6
		info.manufacturer,		// 7
		info.buildDate[1], info.buildDate[2], info.buildDate[3], // 8
		info.buildTime[0], info.buildTime[1], info.buildTime[2], info.buildTime[3], // 9
		info.addInfo);			// 10
		
	unsigned int checkSum = ASCII_compute_checksum((uint8_t*)(a+1), n);
	n += SNPRINTF(a+n, aSize-n, "*%.2x\r\n", checkSum);
	return n;	
}

int tow_to_nmea_ptow(char a[], const int aSize, double imuTow, double insTow, unsigned int gpsWeek)
{
	int n = SNPRINTF(a, aSize, "$PTOW");
	n += SNPRINTF(a+n, aSize-n, ",%.6lf", imuTow);		// 1
	n += SNPRINTF(a+n, aSize-n, ",%.6lf", insTow);		// 2
	n += SNPRINTF(a+n, aSize-n, ",%u", gpsWeek);		// 3
	
	unsigned int checkSum = ASCII_compute_checksum((uint8_t*)(a+1), n);
	n += SNPRINTF(a+n, aSize-n, "*%.2x\r\n", checkSum);
	return n;	
}

int dimu_to_nmea_pimu(char a[], const int aSize, dual_imu_t &dimu)
{
	int n = SNPRINTF(a, aSize, "$PIMU");
	n += SNPRINTF(a+n, aSize-n, ",%.3lf", dimu.time);		// 1
	
	n += SNPRINTF(a+n, aSize-n, ",%.4f", dimu.I[0].pqr[0]);	// 2
	n += SNPRINTF(a+n, aSize-n, ",%.4f", dimu.I[0].pqr[1]);	// 3
	n += SNPRINTF(a+n, aSize-n, ",%.4f", dimu.I[0].pqr[2]);	// 4

	n += SNPRINTF(a+n, aSize-n, ",%.3f", dimu.I[0].acc[0]);	// 5
	n += SNPRINTF(a+n, aSize-n, ",%.3f", dimu.I[0].acc[1]);	// 6
	n += SNPRINTF(a+n, aSize-n, ",%.3f", dimu.I[0].acc[2]);	// 7

	n += SNPRINTF(a+n, aSize-n, ",%.4f", dimu.I[1].pqr[0]);	// 8
	n += SNPRINTF(a+n, aSize-n, ",%.4f", dimu.I[1].pqr[1]);	// 9
	n += SNPRINTF(a+n, aSize-n, ",%.4f", dimu.I[1].pqr[2]);	// 10

	n += SNPRINTF(a+n, aSize-n, ",%.3f", dimu.I[1].acc[0]);	// 11
	n += SNPRINTF(a+n, aSize-n, ",%.3f", dimu.I[1].acc[1]);	// 12
	n += SNPRINTF(a+n, aSize-n, ",%.3f", dimu.I[1].acc[2]);	// 13
	
	unsigned int checkSum = ASCII_compute_checksum((uint8_t*)(a+1), n);
	n += SNPRINTF(a+n, aSize-n, "*%.2x\r\n", checkSum);
	return n;	
}

int pimu_to_nmea_ppimu(char a[], const int aSize, preintegrated_imu_t &pimu)
{
	int n = SNPRINTF(a, aSize, "$PPIMU");
	n += SNPRINTF(a+n, aSize-n, ",%.3lf", pimu.time);		// 1
	
	n += SNPRINTF(a+n, aSize-n, ",%.4f", pimu.theta1[0]);	// 2
	n += SNPRINTF(a+n, aSize-n, ",%.4f", pimu.theta1[1]);	// 3
	n += SNPRINTF(a+n, aSize-n, ",%.4f", pimu.theta1[2]);	// 4

	n += SNPRINTF(a+n, aSize-n, ",%.4f", pimu.theta2[0]);	// 5
	n += SNPRINTF(a+n, aSize-n, ",%.4f", pimu.theta2[1]);	// 6
	n += SNPRINTF(a+n, aSize-n, ",%.4f", pimu.theta2[2]);	// 7

	n += SNPRINTF(a+n, aSize-n, ",%.4f", pimu.vel1[0]);		// 8
	n += SNPRINTF(a+n, aSize-n, ",%.4f", pimu.vel1[1]);		// 9
	n += SNPRINTF(a+n, aSize-n, ",%.4f", pimu.vel1[2]);		// 10

	n += SNPRINTF(a+n, aSize-n, ",%.4f", pimu.vel2[0]);		// 11
	n += SNPRINTF(a+n, aSize-n, ",%.4f", pimu.vel2[1]);		// 12
	n += SNPRINTF(a+n, aSize-n, ",%.4f", pimu.vel2[2]);		// 13

	n += SNPRINTF(a+n, aSize-n, ",%.3f", pimu.dt);			// 14
	
	unsigned int checkSum = ASCII_compute_checksum((uint8_t*)(a+1), n);
	n += SNPRINTF(a+n, aSize-n, "*%.2x\r\n", checkSum);
	return n;	
}

int ins1_to_nmea_pins1(char a[], const int aSize, ins_1_t &ins1)
{
	int n = SNPRINTF(a, aSize, "$PINS1");
	n += SNPRINTF(a+n, aSize-n, ",%.3lf", ins1.timeOfWeek);	// 1

	n += SNPRINTF(a+n, aSize-n, ",%u", (unsigned int)ins1.week);		// 2
	n += SNPRINTF(a+n, aSize-n, ",%u", (unsigned int)ins1.insStatus);	// 3
	n += SNPRINTF(a+n, aSize-n, ",%u", (unsigned int)ins1.hdwStatus);	// 4

	n += SNPRINTF(a+n, aSize-n, ",%.4f", ins1.theta[0]);				// 5
	n += SNPRINTF(a+n, aSize-n, ",%.4f", ins1.theta[1]);				// 6
	n += SNPRINTF(a+n, aSize-n, ",%.4f", ins1.theta[2]);				// 7

	n += SNPRINTF(a+n, aSize-n, ",%.3f", ins1.uvw[0]);					// 8
	n += SNPRINTF(a+n, aSize-n, ",%.3f", ins1.uvw[1]);					// 9
	n += SNPRINTF(a+n, aSize-n, ",%.3f", ins1.uvw[2]);					// 10

	n += SNPRINTF(a+n, aSize-n, ",%.8lf", ins1.lla[0]);					// 11
	n += SNPRINTF(a+n, aSize-n, ",%.8lf", ins1.lla[1]);					// 12
	n += SNPRINTF(a+n, aSize-n, ",%.3lf", ins1.lla[2]);					// 13

	n += SNPRINTF(a+n, aSize-n, ",%.3f", ins1.ned[0]);					// 14
	n += SNPRINTF(a+n, aSize-n, ",%.3f", ins1.ned[1]);					// 15
	n += SNPRINTF(a+n, aSize-n, ",%.3f", ins1.ned[2]);					// 16
	
	unsigned int checkSum = ASCII_compute_checksum((uint8_t*)(a+1), n);
	n += SNPRINTF(a+n, aSize-n, "*%.2x\r\n", checkSum);
	return n;	
}

int ins2_to_nmea_pins2(char a[], const int aSize, ins_2_t &ins2)
{
	int n = SNPRINTF(a, aSize, "$PINS2");
	n += SNPRINTF(a+n, aSize-n, ",%.3lf", ins2.timeOfWeek);				// 1

	n += SNPRINTF(a+n, aSize-n, ",%u", (unsigned int)ins2.week);		// 2
	n += SNPRINTF(a+n, aSize-n, ",%u", (unsigned int)ins2.insStatus);	// 3
	n += SNPRINTF(a+n, aSize-n, ",%u", (unsigned int)ins2.hdwStatus);	// 4
	
	n += SNPRINTF(a+n, aSize-n, ",%.4f", ins2.qn2b[0]);					// 5
	n += SNPRINTF(a+n, aSize-n, ",%.4f", ins2.qn2b[1]);					// 6
	n += SNPRINTF(a+n, aSize-n, ",%.4f", ins2.qn2b[2]);					// 7
	n += SNPRINTF(a+n, aSize-n, ",%.4f", ins2.qn2b[3]);					// 8

	n += SNPRINTF(a+n, aSize-n, ",%.3f", ins2.uvw[0]);					// 9
	n += SNPRINTF(a+n, aSize-n, ",%.3f", ins2.uvw[1]);					// 10
	n += SNPRINTF(a+n, aSize-n, ",%.3f", ins2.uvw[2]);					// 11

	n += SNPRINTF(a+n, aSize-n, ",%.8lf", ins2.lla[0]);					// 12
	n += SNPRINTF(a+n, aSize-n, ",%.8lf", ins2.lla[1]);					// 13
	n += SNPRINTF(a+n, aSize-n, ",%.3lf", ins2.lla[2]);					// 14
	
	unsigned int checkSum = ASCII_compute_checksum((uint8_t*)(a+1), n);
	n += SNPRINTF(a+n, aSize-n, "*%.2x\r\n", checkSum);		
	return n;	
}

int strobe_to_nmea_pstrb(char a[], const int aSize, strobe_in_time_t &strobe)
{
	int n = SNPRINTF(a, aSize, "$PSTRB");
	n += SNPRINTF(a+n, aSize-n, ",%u", (unsigned int)strobe.week);			// 1
	n += SNPRINTF(a+n, aSize-n, ",%u", (unsigned int)strobe.timeOfWeekMs);	// 2
	n += SNPRINTF(a+n, aSize-n, ",%u", (unsigned int)strobe.pin);			// 3
	n += SNPRINTF(a+n, aSize-n, ",%u", (unsigned int)strobe.count);			// 4
	
	unsigned int checkSum = ASCII_compute_checksum((uint8_t*)(a+1), n);
	n += SNPRINTF(a+n, aSize-n, "*%.2x\r\n", checkSum);
	return n;
}

int gps_to_nmea_pgpsp(char a[], const int aSize, gps_pos_t &pos, gps_vel_t &vel)
{
	int n = SNPRINTF(a, aSize, "$PGPSP");
	n += SNPRINTF(a+n, aSize-n, ",%u", (unsigned int)pos.timeOfWeekMs);	// 1
	n += SNPRINTF(a+n, aSize-n, ",%u", (unsigned int)pos.week);			// 2
	n += SNPRINTF(a+n, aSize-n, ",%u", (unsigned int)pos.status);		// 3

	n += SNPRINTF(a+n, aSize-n, ",%.8lf", pos.lla[0]);					// 4
	n += SNPRINTF(a+n, aSize-n, ",%.8lf", pos.lla[1]);					// 5
	n += SNPRINTF(a+n, aSize-n, ",%.2lf", pos.lla[2]);					// 6
	
	n += SNPRINTF(a+n, aSize-n, ",%.2f", pos.hMSL);						// 7
	n += SNPRINTF(a+n, aSize-n, ",%.2f", pos.pDop);						// 8
	n += SNPRINTF(a+n, aSize-n, ",%.2f", pos.hAcc);						// 9
	n += SNPRINTF(a+n, aSize-n, ",%.2f", pos.vAcc);						// 10

	n += SNPRINTF(a+n, aSize-n, ",%.2f", vel.vel[0]);					// 11
	n += SNPRINTF(a+n, aSize-n, ",%.2f", vel.vel[1]);					// 12
	n += SNPRINTF(a+n, aSize-n, ",%.2f", vel.vel[2]);					// 13
	n += SNPRINTF(a+n, aSize-n, ",%.2f", vel.sAcc);						// 14

	n += SNPRINTF(a+n, aSize-n, ",%.1f", pos.cnoMean);					// 15
	n += SNPRINTF(a+n, aSize-n, ",%.4lf", pos.towOffset);				// 16
	n += SNPRINTF(a+n, aSize-n, ",%u", (unsigned int)pos.leapS);		// 17
	
	unsigned int checkSum = ASCII_compute_checksum((uint8_t*)(a+1), n);
	n += SNPRINTF(a+n, aSize-n, "*%.2x\r\n", checkSum);
	return n;	
}

static int asciiSnprintfLatToDegMin(char* a, size_t aSize, double v)
{
	int degrees = (int)(v);
	double minutes = (v-((double)degrees))*60.0;
	
	return SNPRINTF(a, aSize, ",%02d%07.4lf,%c", abs(degrees), fabs(minutes), (degrees >= 0 ? 'N' : 'S'));
}

static int asciiSnprintfLonToDegMin(char* a, size_t aSize, double v)
{
	int degrees = (int)(v);
	double minutes = (v-((double)degrees))*60.0;
	
	return SNPRINTF(a, aSize, ",%03d%07.4lf,%c", abs(degrees), fabs(minutes), (degrees >= 0 ? 'E' : 'W'));
}

static int asciiSnprintfGPSTimeOfLastFix(char* a, size_t aSize, uint32_t timeOfWeekMs)
{
	unsigned int millisecondsToday = timeOfWeekMs % 86400000;
	unsigned int hours = millisecondsToday / 1000 / 60 / 60;
	unsigned int minutes = (millisecondsToday / (1000 * 60)) % 60;
	unsigned int seconds = (millisecondsToday / 1000) % 60;
	
	return SNPRINTF(a, aSize, ",%02u%02u%02u", hours, minutes, seconds);
}

static int asciiSnprintfGPSTimeOfLastFixMilliseconds(char* a, size_t aSize, uint32_t timeOfWeekMs)
{
	unsigned int millisecondsToday = timeOfWeekMs % 86400000;
	unsigned int hours = millisecondsToday / 1000 / 60 / 60;
	unsigned int minutes = (millisecondsToday / (1000 * 60)) % 60;
	unsigned int seconds = (millisecondsToday / 1000) % 60;
	unsigned int milliseconds = millisecondsToday % 1000;
	
	return SNPRINTF(a, aSize, ",%02u%02u%02u.%03u", hours, minutes, seconds, milliseconds);
}

static int asciiSnprintfGPSDateOfLastFix(char* a, size_t aSize, gps_pos_t &pos)
{
	double julian = gpsToJulian(pos.week, pos.timeOfWeekMs, pos.leapS);
	int32_t year, month, day, hours, minutes, seconds, milliseconds;
	julianToDate(julian, &year, &month, &day, &hours, &minutes, &seconds, &milliseconds);
	
	return SNPRINTF(a, aSize, ",%02u%02u%02u", (unsigned int)day, (unsigned int)month, (unsigned int)year);
}

static int asciiSnprintfGPSDateOfLastFixCSV(char* a, size_t aSize, gps_pos_t &pos)	//Comma Separated Values
{
	double julian = gpsToJulian(pos.week, pos.timeOfWeekMs, pos.leapS);
	int32_t year, month, day, hours, minutes, seconds, milliseconds;
	julianToDate(julian, &year, &month, &day, &hours, &minutes, &seconds, &milliseconds);
	
	return SNPRINTF(a, aSize, ",%02u,%02u,%02u", (unsigned int)day, (unsigned int)month, (unsigned int)year);
}

int gps_to_nmea_gga(char a[], const int aSize, gps_pos_t &pos)
{
	int fixQuality;
	switch((pos.status&GPS_STATUS_FIX_MASK))
	{
	default:
	case GPS_STATUS_FIX_NONE:                   fixQuality = 0;	break;
	case GPS_STATUS_FIX_SBAS:
	case GPS_STATUS_FIX_2D:
	case GPS_STATUS_FIX_RTK_SINGLE:
	case GPS_STATUS_FIX_3D:                     fixQuality = 1;	break;
	case GPS_STATUS_FIX_DGPS:                   fixQuality = 2;	break;
	case GPS_STATUS_FIX_TIME_ONLY:              fixQuality = 3;	break;   
    case GPS_STATUS_FIX_RTK_FIX:                fixQuality = 4;	break;
	case GPS_STATUS_FIX_RTK_FLOAT:              fixQuality = 5;	break;	
	case GPS_STATUS_FIX_DEAD_RECKONING_ONLY:
	case GPS_STATUS_FIX_GPS_PLUS_DEAD_RECK:     fixQuality = 6;	break;
	}
		
	// NMEA GGA line - http://www.gpsinformation.org/dale/nmea.htm#GGA
	/*
	GGA          Global Positioning System Fix Data
	123519       Fix taken at 12:35:19 UTC
	4807.038,N   Latitude 48 deg 07.038' N
	01131.000,E  Longitude 11 deg 31.000' E
	.            Fix quality:	0 = invalid
	.							1 = GPS fix (SPS)
	.							2 = DGPS fix
	.							3 = PPS fix
	.							4 = Real Time Kinematic
	.							5 = Float RTK
	.							6 = estimated (dead reckoning) (2.3 feature)
	.							7 = Manual input mode
	.							8 = Simulation mode
	08           Number of satellites being tracked
	0.9          Horizontal dilution of position
	545.4,M      MSL altitude in meters 
	46.9,M       HAE altitude (above geoid / WGS84 ellipsoid)
	ellipsoid
	(empty field) time in seconds since last DGPS update
	(empty field) DGPS station ID number
	*47          the checksum data, always begins with *
	*/

// 	unsigned int checkSum = 0;
// 	serWrite( portNum, (unsigned char*)"$", 1);
// 	ASCII_PORT_WRITE_NO_FORMAT(portNum, "GPGGA", checkSum, 5);
// 	asciiPortWriteGPSTimeOfLastFix(portNum, a, checkSum);										// 1
// 	asciiPortWriteCoordDegMin(portNum, a, checkSum, pos.lla[0], ",%02d", 'N', 'S');		// 2,3
// 	asciiPortWriteCoordDegMin(portNum, a, checkSum, pos.lla[1], ",%03d", 'E', 'W');		// 4,5	
// 	ASCII_PORT_WRITE(portNum, a, checkSum, ",%u", (unsigned)(fixQuality));						// 6
// 	ASCII_PORT_WRITE(portNum, a, checkSum, ",%02u", (unsigned)(pos.status&GPS_STATUS_NUM_SATS_USED_MASK));	// 7
// 	ASCII_PORT_WRITE(portNum, a, checkSum, ",%.2f", pos.pDop);								// 8
// 	ASCII_PORT_WRITE(portNum, a, checkSum, ",%.2f,M", pos.hMSL);							// 9,10
// 	ASCII_PORT_WRITE(portNum, a, checkSum, ",%.2f,M", pos.hMSL - pos.lla[2]);			// 11,12
// 	ASCII_PORT_WRITE_NO_FORMAT(portNum, ",,", checkSum, 2);										// 13,14
// 	ASCII_PORT_WRITE_NO_CHECKSUM(portNum, a, "*%.2x\r\n", checkSum);

	int n = SNPRINTF(a, aSize, "$GPGGA");
	n += asciiSnprintfGPSTimeOfLastFix(a+n, aSize-n, pos.timeOfWeekMs);	// 1
	n += asciiSnprintfLatToDegMin(a+n, aSize-n, pos.lla[0]);			// 2,3
	n += asciiSnprintfLonToDegMin(a+n, aSize-n, pos.lla[1]);			// 4,5
	n += SNPRINTF(a+n, aSize-n, 
		",%u"		// 6
		",%02u"		// 7
		",%.2f"		// 8
		",%.2f,M"	// 9,10
		",%.2f,M"	// 11,12
		",,", 		// 13,14
		(unsigned int)fixQuality,											// 6
		(unsigned int)(pos.status&GPS_STATUS_NUM_SATS_USED_MASK),	// 7
		pos.pDop,	// 8
		pos.hMSL,	// 9,10
		pos.hMSL - pos.lla[2]);	// 11,12 13,14
	
	unsigned int checkSum = ASCII_compute_checksum((uint8_t*)(a+1), n);	
	n += SNPRINTF(a+n, aSize-n, "*%.2x\r\n", checkSum);
	return n;
}

int gps_to_nmea_gll(char a[], const int aSize, gps_pos_t &pos)
{
	// NMEA GLL line - http://www.gpsinformation.org/dale/nmea.htm#GLL
	/*
	     GLL          Geographic position, Latitude and Longitude
	     4916.46,N    Latitude 49 deg. 16.45 min. North
	     12311.12,W   Longitude 123 deg. 11.12 min. West
	     225444       Fix taken at 22:54:44 UTC
	     A            Data Active or V (void)
	     *iD          checksum data
	*/

// 	unsigned int checkSum = 0;
// 	serWrite( portNum, (unsigned char*)"$", 1);
// 	ASCII_PORT_WRITE_NO_FORMAT(portNum, "GPGLL", checkSum, 5);
// 	asciiPortWriteCoordDegMin(portNum, a, checkSum, pos.lla[0], ",%02d", 'N', 'S');		// 1,2
// 	asciiPortWriteCoordDegMin(portNum, a, checkSum, pos.lla[1], ",%03d", 'E', 'W');		// 3,4
// 	asciiPortWriteGPSTimeOfLastFix(portNum, a, checkSum);										// 5
// 	ASCII_PORT_WRITE_NO_FORMAT(portNum, ",A", checkSum, 2);										// 6
// 	ASCII_PORT_WRITE_NO_CHECKSUM(portNum, a, "*%.2x\r\n", checkSum);

	int n = SNPRINTF(a, aSize, "$GPGLL");
	n += asciiSnprintfLatToDegMin(a+n, aSize-n, pos.lla[0]);			// 1,2
	n += asciiSnprintfLonToDegMin(a+n, aSize-n, pos.lla[1]);			// 3,4
	n += asciiSnprintfGPSTimeOfLastFix(a+n, aSize-n, pos.timeOfWeekMs);	// 5
	n += SNPRINTF(a+n, aSize-n, ",A");	// 6
	
	unsigned int checkSum = ASCII_compute_checksum((uint8_t*)(a+1), n);
	n += SNPRINTF(a+n, aSize-n, "*%.2x\r\n", checkSum);
	return n;	
}

int gps_to_nmea_gsa(char a[], const int aSize, gps_pos_t &pos, gps_sat_t &sat)
{
	int fixQuality;
	switch((pos.status&GPS_STATUS_FIX_MASK))
	{
	default:
		fixQuality = 0;	break;
	case GPS_STATUS_FIX_2D:					
		fixQuality = 2;	break;

	case GPS_STATUS_FIX_3D:					
	case GPS_STATUS_FIX_SBAS:
	case GPS_STATUS_FIX_DGPS:					
	case GPS_STATUS_FIX_RTK_FIX:
	case GPS_STATUS_FIX_RTK_SINGLE:
	case GPS_STATUS_FIX_RTK_FLOAT:				
		fixQuality = 3;	break;
	}
		
	// NMEA GSA line - http://www.gpsinformation.org/dale/nmea.htm#GSA
	/*
		eg1. $GPGSA,A,3,,,,,,16,18,,22,24,,,3.6,2.1,2.2*3C
		eg2. $GPGSA,A,3,19,28,14,18,27,22,31,39,,,,,1.7,1.0,1.3*35

		1    = Mode:
		.		M=Manual, forced to operate in 2D or 3D
		.		A=Automatic, 3D/2D
		2    = Mode:
		.		1=Fix not available
		.		2=2D
		.		3=3D
		3-14 = IDs of SVs used in position fix (null for unused fields)
		15   = PDOP
		16   = HDOP
		17   = VDOP
	*/

// 	unsigned int checkSum = 0;
// 	serWrite( portNum, (unsigned char*)"$", 1);
// 	ASCII_PORT_WRITE_NO_FORMAT(portNum, "GPGSA", checkSum, 5);
// 	ASCII_PORT_WRITE_NO_FORMAT(portNum, ",A", checkSum, 2);							// 1
// 	ASCII_PORT_WRITE(portNum, a, checkSum, ",%02u", (unsigned)(fixQuality));		// 2
// 	for (uint32_t i = 0; i < 12; i++)												// 3-14
// 	{
// 		if(g_gps1Sat.sat[i].svId)
// 		{
// 			ASCII_PORT_WRITE(portNum, a, checkSum, ",%02u", (unsigned)(g_gps1Sat.sat[i].svId));
// 		}
// 		else
// 		{
// 			ASCII_PORT_WRITE_NO_FORMAT(portNum, ",", checkSum, 1);
// 		}
// 	}
// 	ASCII_PORT_WRITE(portNum, a, checkSum, ",%.1f", pos.pDop);					// 15
// 	ASCII_PORT_WRITE(portNum, a, checkSum, ",%.1f", pos.hAcc);					// 16
// 	ASCII_PORT_WRITE(portNum, a, checkSum, ",%.1f", pos.vAcc);					// 17
// 	ASCII_PORT_WRITE_NO_CHECKSUM(portNum, a, "*%.2x\r\n", checkSum);

	int n = SNPRINTF(a, aSize, "$GPGSA"
		",A"		// 1
		",%02u",	// 2
		(unsigned int)fixQuality);	// 1,2
		
	for (uint32_t i = 0; i < 12; i++)												// 3-14
	{
		if(sat.sat[i].svId)
		{
			n += SNPRINTF(a+n, aSize-n, ",%02u", (unsigned)(sat.sat[i].svId));
		}
		else
		{
			n += SNPRINTF(a+n, aSize-n, ",");
		}
	}
		
	n += SNPRINTF(a+n, aSize-n,
		",%.1f"		// 15
		",%.1f"		// 16
		",%.1f",	// 17
		pos.pDop,	// 15
		pos.hAcc,	// 16
		pos.vAcc);	// 17
	
	unsigned int checkSum = ASCII_compute_checksum((uint8_t*)(a+1), n);
	n += SNPRINTF(a+n, aSize-n, "*%.2x\r\n", checkSum);
	return n;
}

int gps_to_nmea_rmc(char a[], const int aSize, gps_pos_t &pos, gps_vel_t &vel, float magDeclination)
{
	Quat_t qe2n;
	Vector3_t vel_ned_;
	quat_ecef2ned((float)pos.lla[0], (float)pos.lla[1], qe2n);
	quatConjRot(vel_ned_, qe2n, vel.vel);

	// 	unsigned int checkSum = 0;
	// 	serWrite(portNum, (unsigned char*)"$", 1);
	// 	ASCII_PORT_WRITE_NO_FORMAT(portNum, "GPRMC", checkSum, 5);
	// 	asciiPortWriteGPSTimeOfLastFix(portNum, a, checkSum);									// 1	// time of last fix
	// 	if((pos.status&GPS_STATUS_FIX_MASK)!=GPS_STATUS_FIX_NONE)
	// 	{
	// 		ASCII_PORT_WRITE_NO_FORMAT(portNum, ",A", checkSum, 2);								// 2	// A=active (good)
	// 	}
	// 	else
	// 	{
	// 		ASCII_PORT_WRITE_NO_FORMAT(portNum, ",V", checkSum, 2);								// 2	// V=void (bad,warning)
	// 	}
	// 	asciiPortWriteCoordDegMin(portNum, a, checkSum, pos.lla[0], ",%02d", 'N', 'S');	// 3,4	// lat lon (degrees minutes)
	// 	asciiPortWriteCoordDegMin(portNum, a, checkSum, pos.lla[1], ",%03d", 'E', 'W');	// 5,6
	//
	// 	float speedInKnots = C_METERS_KNOTS_F * mag_Vec2(vel_ned_);
	// 	ASCII_PORT_WRITE(portNum, a, checkSum, ",%05.1f", speedInKnots);						// 7	// speed in knots
	// // 	float courseMadeTrue = atan2f(g_navInGpsA.velNed[1], g_navInGpsA.velNed[0]);
	// 	float courseMadeTrue = 0.0f;
	// 	ASCII_PORT_WRITE(portNum, a, checkSum, ",%05.1f", (courseMadeTrue*C_RAD2DEG_F));		// 8	// course made true
	// 	asciiPortWriteGPSDateOfLastFix(portNum, a, checkSum);									// 9	// date of last fix UTC
	//
	// 	// Magnetic variation degrees (Easterly var. subtracts from true course), i.e. 020.3,E - left pad to 3 zero
	// 	float magDec = g_nvmFlashCfg->magDeclination * C_RAD2DEG_F;
	// 	bool positive = (magDec >= 0.0);
	// 	ASCII_PORT_WRITE(portNum, a, checkSum, ",%05.1f,", abs(magDec));						// 10	// Magnetic variation
	// 	ASCII_PORT_WRITE_NO_FORMAT(portNum, (positive ? "E" : "W"), checkSum, 1);				// 11
	// 	ASCII_PORT_WRITE_NO_CHECKSUM(portNum, a, "*%.2x\r\n", checkSum);

	int n = SNPRINTF(a, aSize, "$GPRMC");
	n += asciiSnprintfGPSTimeOfLastFix(a+n, aSize-n, pos.timeOfWeekMs);					// 1	// time of last fix
	if((pos.status&GPS_STATUS_FIX_MASK)!=GPS_STATUS_FIX_NONE)
	{
		n += SNPRINTF(a+n, aSize-n, ",A");												// 2	// A=active (good)
	}
	else
	{
		n += SNPRINTF(a+n, aSize-n, ",V");												// 2	// V=void (bad,warning)
	}
	n += asciiSnprintfLatToDegMin(a+n, aSize-n, pos.lla[0]);							// 3,4	// lat (degrees minutes)
	n += asciiSnprintfLonToDegMin(a+n, aSize-n, pos.lla[1]);							// 5,6	// lon (degrees minutes)
	
	float speedInKnots = C_METERS_KNOTS_F * mag_Vec2(vel_ned_);
	// 	float courseMadeTrue = atan2f(g_navInGpsA.velNed[1], g_navInGpsA.velNed[0]);
	float courseMadeTrue = 0.0f;
	n += SNPRINTF(a+n, aSize-n,
	",%05.1f"		// 7
	",%05.1f",		// 8
	speedInKnots,																		// 7	// speed in knots
	courseMadeTrue*C_RAD2DEG_F);														// 8	// course made true
	
	n += asciiSnprintfGPSDateOfLastFix(a+n, aSize-n, pos);								// 9	// date of last fix UTC
	
	// Magnetic variation degrees (Easterly var. subtracts from true course), i.e. 020.3,E - left pad to 3 zero
	float magDec = magDeclination * C_RAD2DEG_F;
	bool positive = (magDec >= 0.0);
	
	n += SNPRINTF(a+n, aSize-n,
	",%05.1f"	// 10
	",%s",		// 11
	fabsf(magDec),																				// 10	// Magnetic variation
	(positive ? "E" : "W"));																	// 11
	
	unsigned int checkSum = ASCII_compute_checksum((uint8_t*)(a+1), n);
	n += SNPRINTF(a+n, aSize-n, "*%.2x\r\n", checkSum);
	return n;
}

int gps_to_nmea_zda(char a[], const int aSize, gps_pos_t &pos)
{
	// NMEA ZDA line - http://www.gpsinformation.org/dale/nmea.htm#ZDA
	/*
		hhmmss    HrMinSec(UTC)
		dd,mm,yyy Day,Month,Year
		xx        local zone hours -13..13 - Fixed field: 00
		yy        local zone minutes 0..59 - Fixed field: 00
		*CC       checksum
	*/

	int n = SNPRINTF(a, aSize, "$GPZDA");									//Field 1
	n += asciiSnprintfGPSTimeOfLastFix(a+n, aSize-n, pos.timeOfWeekMs + pos.leapS*1000);		//Field 2 
	n += asciiSnprintfGPSDateOfLastFixCSV(a+n, aSize-n, pos);				// 2,3,4
	n += SNPRINTF(a+n, aSize-n, ",00,00");									// 5,6
	
	unsigned int checkSum = ASCII_compute_checksum((uint8_t*)(a+1), n);
	n += SNPRINTF(a+n, aSize-n, "*%.2x\r\n", checkSum);
	return n;	
}

int gps_to_nmea_pashr(char a[], const int aSize, gps_pos_t &pos, ins_1_t &ins1, float heave, inl2_ned_sigma_t &sigma)
{
	// NMEA PASHR - RT300 proprietary roll and pitch sentence
	/*
		hhmmss.sss - UTC time
		hhh.hh - Heading in degrees
		T - flag to indicate that the Heading is True Heading (i.e. to True North)
		rrr.rr - Roll Angle in degrees
		ppp.pp - Pitch Angle in degrees
		xxx.xx - Heave
		a.aaa - Roll Angle Accuracy Estimate (Stdev) in degrees
		b.bbb - Pitch Angle Accuracy Estimate (Stdev) in degrees
		c.ccc - Heading Angle Accuracy Estimate (Stdev) in degrees
		d - Aiding Status
		e - IMU Status
		hh - Checksum
	*/
	
	int n = SNPRINTF(a, aSize, "$PASHR");																//Field 1 - Name
	n += asciiSnprintfGPSTimeOfLastFixMilliseconds(a+n, aSize-n, pos.timeOfWeekMs + pos.leapS*1000);	//Field 2 - UTC Time

	n += SNPRINTF(a+n, aSize-n, ",%.2f", RAD2DEG(ins1.theta[2]));										//Field 3 - Heading value in decimal degrees.
	n += SNPRINTF(a+n, aSize-n, ",T");																	//Field 4 - T (heading respect to True North)
	n += SNPRINTF(a+n, aSize-n, ",%+.2f", RAD2DEG(ins1.theta[0]));										//Field 5 - Roll in degrees
	n += SNPRINTF(a+n, aSize-n, ",%+.2f", RAD2DEG(ins1.theta[1]));										//Field 6 - Pitch in degrees
	n += SNPRINTF(a+n, aSize-n, ",%+.2f", heave);														//Field 7 - Heave
	
	n += SNPRINTF(a+n, aSize-n, ",%.3f", RAD2DEG(sigma.PattNED[0])); //roll accuracy	//8
	n += SNPRINTF(a+n, aSize-n, ",%.3f", RAD2DEG(sigma.PattNED[1])); //pitch accuracy	//9
	n += SNPRINTF(a+n, aSize-n, ",%.3f", RAD2DEG(sigma.PattNED[2])); //heading accuracy	//10
	
	int fix = 0;
	if(INS_STATUS_NAV_FIX_STATUS(ins1.insStatus) >= GPS_NAV_FIX_POSITIONING_RTK_FLOAT)
	{
		fix = 2;
	}
	else if(INS_STATUS_NAV_FIX_STATUS(ins1.insStatus) >= GPS_NAV_FIX_POSITIONING_3D)
	{
		fix = 1;
	}
	n += SNPRINTF(a+n, aSize-n, ",%d", fix);															//Field 11 - GPS Quality
	n += SNPRINTF(a+n, aSize-n, ",%d", INS_STATUS_SOLUTION(ins1.insStatus) >= INS_STATUS_SOLUTION_NAV); //Field 12 - INS Status
	
	unsigned int checkSum = ASCII_compute_checksum((uint8_t*)(a+1), n);
	n += SNPRINTF(a+n, aSize-n, "*%.2x\r\n", checkSum);
	return n;
}

	
//////////////////////////////////////////////////////////////////////////
// Parse NMEA Functions
//////////////////////////////////////////////////////////////////////////

uint32_t parse_nmea_ascb(int pHandle, const char msg[], int msgSize, ascii_msgs_t asciiPeriod[NUM_COM_PORTS])
{
	if(pHandle >= NUM_COM_PORTS)
	{
		return -1;
	}
	char *ptr = (char *)msg;
	
	// Default to current settings
	ascii_msgs_t tmp = asciiPeriod[pHandle];
	uint32_t options = 0;
	
	ptr = ASCII_find_next_field(ptr);			// Options
	if(*ptr!=','){ options = atoi(ptr);		}		
	ptr = ASCII_find_next_field(ptr);			// PIMU
	if(*ptr!=','){ tmp.pimu = atoi(ptr);	}	
	ptr = ASCII_find_next_field(ptr);			// PPIMU
	if(*ptr!=','){ tmp.ppimu = atoi(ptr);	}
	ptr = ASCII_find_next_field(ptr);			// PINS1
	if(*ptr!=','){ tmp.pins1 = atoi(ptr);	}
	ptr = ASCII_find_next_field(ptr);			// PINS2
	if(*ptr!=','){ tmp.pins2 = atoi(ptr);	}
	ptr = ASCII_find_next_field(ptr);			// PGPSP
	if(*ptr!=','){ tmp.pgpsp = atoi(ptr);	}
	ptr = ASCII_find_next_field(ptr);			// reserved
	
	ptr = ASCII_find_next_field(ptr);			// gpgga
	if(*ptr!=','){ tmp.gpgga = atoi(ptr);	}
	ptr = ASCII_find_next_field(ptr);			// gpgll
	if(*ptr!=','){ tmp.gpgll = atoi(ptr);	}
	ptr = ASCII_find_next_field(ptr);			// gpgsa
	if(*ptr!=','){ tmp.gpgsa = atoi(ptr);	}
	ptr = ASCII_find_next_field(ptr);			// gprmc
	if(*ptr!=','){ tmp.gprmc = atoi(ptr);	}
	ptr = ASCII_find_next_field(ptr);			// gpzda
	if(*ptr!=','){ tmp.gpzda = atoi(ptr);	}
	ptr = ASCII_find_next_field(ptr);			// pashr
	if(*ptr!=','){ tmp.pashr = atoi(ptr);	}
		
	// Copy tmp to corresponding port(s)
	switch(options&RMC_OPTIONS_PORT_MASK)
	{	
	case 0xFF:	// All ports
		for(int i=0; i<NUM_COM_PORTS; i++)
		{
			asciiPeriod[i] = tmp;
		}
		break;
		
	default:	// Current port
	case RMC_OPTIONS_PORT_CURRENT:	asciiPeriod[pHandle] = tmp;	break;
	case RMC_OPTIONS_PORT_SER0:		asciiPeriod[0] = tmp;		break;
	case RMC_OPTIONS_PORT_SER1:		asciiPeriod[1] = tmp;		break;
	case RMC_OPTIONS_PORT_USB:		asciiPeriod[2] = tmp;		break;
	}
		
	return options;
}


/* G_ZDA message
*  Provides day/month/year fort calculating iTOW.
*/
int parse_nmea_zda(const char msg[], int msgSize, double &day, double &month, double &year)
{
	char *ptr = (char *)&msg[7];
	//$xxZDA,time,day,month,year,ltzh,ltzn*cs<CR><LF>
			
	//time
	ptr = ASCII_find_next_field(ptr);
			
	//day
	day = atoi(ptr);
	ptr = ASCII_find_next_field(ptr);
			
	//month
	month = atoi(ptr);
	ptr = ASCII_find_next_field(ptr);
			
	//year
	year = atoi(ptr);

	return 0;
}

/* G_GNS Message
* Provides position data (newer message) using the following:
*   Time
*   Position (lat, lon)
*   Positioning mode (fix type)
*   Number Satellites
*   Altitude & Geoid separation
*/
int parse_nmea_gns(const char msg[], int msgSize, gps_pos_t *gpsPos, double datetime[6], int *satsUsed, int navMode)
{
	char *ptr = (char *)&msg[7];
	//$xxGNS,time,lat,NS,lon,EW,posMode,numSV,HDOP,alt,sep,diffAge,diffStation,navStatus*cs<CR><LF>

	//UTC time, hhmmss
	double UTCtime = atof(ptr);
	ptr = ASCII_find_next_field(ptr);

	//Convert time to iTOW
	datetime[3] = ((int)UTCtime / 10000) % 100;
	datetime[4] = ((int)UTCtime / 100) % 100;
	double subSec = UTCtime - (int)UTCtime;
	datetime[5] = (double)((int)UTCtime % 100) + subSec + gpsPos->leapS;
			
	gtime_t gtm = epoch2time(datetime);
	int week;
	double iTOWd = time2gpst(gtm, &week);
	uint32_t iTOW = (uint32_t)((iTOWd + 0.00001) * 1000.0);
		
	//Latitude
	Vector3d lla;
	lla[0] = ddmm2deg(atof(ptr));
	ptr = ASCII_find_next_field(ptr);
	if(*ptr == 'S')
		lla[0] = -lla[0];
	ptr = ASCII_find_next_field(ptr);

	//Longitude
	lla[1] = ddmm2deg(atof(ptr));
	ptr = ASCII_find_next_field(ptr);
	if(*ptr == 'W')
		lla[1] = -lla[1];
	ptr = ASCII_find_next_field(ptr);

	//Positioning Mode
	char pMode[4] = {0,0,0,0};
	if(*ptr != ',')
		pMode[0] = *ptr++;
	if(*ptr != ',')
		pMode[1] = *ptr++;
	if(*ptr != ',')
		pMode[2] = *ptr++;
	if(*ptr != ',')
		pMode[3] = *ptr++;
	ptr = ASCII_find_next_field(ptr);
		
	//Based off of ZED-F9P datasheet
	int fixType = 0;
	int differential = 0;
	if(pMode[0] == 'R' || pMode[1] == 'R' || pMode[2] == 'R' || pMode[3] == 'R')	//RTK fixed
		fixType = 2;
	else if(pMode[0] == 'F' || pMode[1] == 'F' || pMode[2] == 'F' || pMode[3] == 'F')	//RTK float
		fixType = 2;
	else if(pMode[0] == 'D' || pMode[1] == 'D' || pMode[2] == 'D' || pMode[3] == 'D')	//2D/3D GNSS fix
	{
		fixType = 2;
		differential = 1;
	}
	else if(pMode[0] == 'A' || pMode[1] == 'A' || pMode[2] == 'A' || pMode[3] == 'A')	//2D/3D GNSS fix
		fixType = 2;
	else if(pMode[0] == 'E' || pMode[1] == 'E' || pMode[2] == 'E' || pMode[3] == 'E')	//Dead rekoning fix
		fixType = 1;
		
	//Determine 2D / 3D
	if(fixType == 2 && navMode == 3)
		fixType = 3;
		
	//Number of satellites used in solution
	*satsUsed = atoi(ptr);
	ptr = ASCII_find_next_field(ptr);
		
	//HDOP
	ptr = ASCII_find_next_field(ptr);
		
	//MSL Altitude (altitude above mean sea level)
	lla[2] = atof(ptr);
	gpsPos->hMSL = (float)lla[2];
	ptr = ASCII_find_next_field(ptr);

	//Geoid separation (difference between ellipsoid and mean sea level)
	double sep = atof(ptr);
		
	//Store data		
	set_gpsPos_status_mask(&(gpsPos->status), *satsUsed, GPS_STATUS_NUM_SATS_USED_MASK);
	set_gpsPos_status_mask(&(gpsPos->status), (fixType != 0) << GPS_STATUS_FLAGS_BIT_OFFSET, GPS_STATUS_FLAGS_FIX_OK);
	set_gpsPos_status_mask(&(gpsPos->status), fixType << GPS_STATUS_FIX_BIT_OFFSET, GPS_STATUS_FIX_MASK);
	set_gpsPos_status_mask(&(gpsPos->status), differential << (GPS_STATUS_FLAGS_BIT_OFFSET + 1), GPS_STATUS_FLAGS_DGPS_USED);
		
	gpsPos->lla[0] = lla[0];
	gpsPos->lla[1] = lla[1];
	gpsPos->lla[2] = lla[2] + sep;

	//Change LLA to radians
	lla[0] = DEG2RAD(lla[0]);
	lla[1] = DEG2RAD(lla[1]);
	lla[2] = gpsPos->lla[2];	// Use ellipsoid alt
		
	//Convert LLA to ECEF.  Ensure LLA uses ellipsoid alt 
	Vector3d ecef;
	lla2ecef(lla, ecef);
				
	gpsPos->timeOfWeekMs = iTOW;
	gpsPos->week = week;
	gpsPos->ecef[0] = ecef[0];
	gpsPos->ecef[1] = ecef[1];
	gpsPos->ecef[2] = ecef[2];	
	//gpsPos->hAcc = 0;
	//gpsPos->vAcc = 0;	
		
	//Indicate it is coming from NMEA
	gpsPos->status |= GPS_STATUS_FLAGS_GPS_NMEA_DATA;

	return 0;	
}

/* G_GGA Message
* Provides position data (older message) using the following:
*   Time
*   Position (lat, lon)
*   Quality (fix type)
*   Number Satellites
*   Altitude & Geoid separation
*/	
int parse_nmea_gga(const char msg[], int msgSize, gps_pos_t *gpsPos, double datetime[6], int *satsUsed, int navMode)
{
	char *ptr = (char *)&msg[7];
	//$xxGGA,time,lat,NS,lon,EW,quality,numSV,HDOP,alt,altUnit,sep,sepUnit,diffAge,diffStation*cs<CR><LF>
			
	//UTC time, hhmmss
	double UTCtime = atof(ptr);
	ptr = ASCII_find_next_field(ptr);

	//Convert time to iTOW
	datetime[3] = ((int)UTCtime / 10000) % 100;
	datetime[4] = ((int)UTCtime / 100) % 100;
	double subSec = UTCtime - (int)UTCtime;
	datetime[5] = (double)((int)UTCtime % 100) + subSec + gpsPos->leapS;
			
	gtime_t gtm = epoch2time(datetime);
	int week;
	double iTOWd = time2gpst(gtm, &week);
	uint32_t iTOW = (uint32_t)((iTOWd + 0.00001) * 1000.0);
			
	//Latitude
	Vector3d lla;
	lla[0] = ddmm2deg(atof(ptr));
	ptr = ASCII_find_next_field(ptr);
	if(*ptr == 'S')
	lla[0] = -lla[0];
	ptr = ASCII_find_next_field(ptr);

	//Longitude
	lla[1] = ddmm2deg(atof(ptr));
	ptr = ASCII_find_next_field(ptr);
	if(*ptr == 'W')
	lla[1] = -lla[1];
	ptr = ASCII_find_next_field(ptr);

	//quality
	int quality = atoi(ptr);
	ptr = ASCII_find_next_field(ptr);
			
	//Based off of ZED-F9P datasheet
	int fixType = 0;
	int differential = 0;
	switch(quality)
	{
		case 1:
		case 4:
		case 5:
		fixType = 2;
		break;
		case 2:
		fixType = 2;
		differential = 1;
		break;
		case 6:
		fixType = 1;
		break;
	}
			
	//Determine 2D / 3D
	if(fixType == 2 && navMode == 3)
	fixType = 3;
			
	//Number of satellites used in solution
	*satsUsed = atoi(ptr);
	ptr = ASCII_find_next_field(ptr);
			
	//HDOP
	ptr = ASCII_find_next_field(ptr);
			
	//MSL Altitude (altitude above mean sea level)
	lla[2] = atof(ptr);
	gpsPos->hMSL = (float)lla[2];
	ptr = ASCII_find_next_field(ptr);

	//altUnit
	ptr = ASCII_find_next_field(ptr);

	//Geoid separation
	double sep = atof(ptr);
			
	//Store data
	set_gpsPos_status_mask(&(gpsPos->status), *satsUsed, GPS_STATUS_NUM_SATS_USED_MASK);
	set_gpsPos_status_mask(&(gpsPos->status), (fixType != 0) << GPS_STATUS_FLAGS_BIT_OFFSET, GPS_STATUS_FLAGS_FIX_OK);
	set_gpsPos_status_mask(&(gpsPos->status), fixType << GPS_STATUS_FIX_BIT_OFFSET, GPS_STATUS_FIX_MASK);
	set_gpsPos_status_mask(&(gpsPos->status), differential << (GPS_STATUS_FLAGS_BIT_OFFSET + 1), GPS_STATUS_FLAGS_DGPS_USED);
			
	gpsPos->lla[0] = lla[0];
	gpsPos->lla[1] = lla[1];
	gpsPos->lla[2] = lla[2] + sep;

	//Change LLA to radians
	lla[0] = DEG2RAD(lla[0]);
	lla[1] = DEG2RAD(lla[1]);
	lla[2] = gpsPos->lla[2];	// Use ellipsoid alt
			
	//Convert LLA to ECEF.  Ensure LLA uses ellipsoid alt
	Vector3d ecef;
	lla2ecef(lla, ecef);
			
	gpsPos->timeOfWeekMs = iTOW;
	gpsPos->week = week;
	gpsPos->ecef[0] = ecef[0];
	gpsPos->ecef[1] = ecef[1];
	gpsPos->ecef[2] = ecef[2];
	//gpsPos->hAcc = 0;
	//gpsPos->vAcc = 0;
			
	//Indicate it is coming from NMEA
	gpsPos->status |= GPS_STATUS_FLAGS_GPS_NMEA_DATA;

	return 0;	
}

/* G_RMC Message
* Provides speed (speed and course over ground)
*/
int parse_nmea_rmc(const char msg[], int msgSize, gps_vel_t *gpsVel, double datetime[6], int *satsUsed, int navMode)
{
	char *ptr = (char *)&msg[7];
	//$xxRMC,time,status,lat,NS,lon,EW,spd,cog,date,mv,mvEW,posMode,navStatus*cs<CR><LF>

	//UTC time, hhmmss
	double UTCtime = atof(ptr);
	ptr = ASCII_find_next_field(ptr);
			
	//Skip 5
	for(int i=0;i<5;++i)
	{
		ptr = ASCII_find_next_field(ptr);
	}

	//spd & cog
	float spdm_s = (float)atof(ptr) * C_KNOTS_METERS_F;
	ptr = ASCII_find_next_field(ptr);
	float cogRad = DEG2RAD((float)atof(ptr));
			
	//Convert time to iTOW
	datetime[3] = ((int)UTCtime / 10000) % 100;
	datetime[4] = ((int)UTCtime / 100) % 100;
	double subSec = UTCtime - (int)UTCtime;
	datetime[5] = (double)((int)UTCtime % 100) + subSec;
			
	gtime_t gtm = epoch2time(datetime);
	double iTOWd = time2gpst(gtm, 0);
	gpsVel->timeOfWeekMs = (uint32_t)((iTOWd + 0.00001) * 1000.0);
			
	//Speed data in NED
	gpsVel->vel[0] = spdm_s * cosf(cogRad);
	gpsVel->vel[1] = spdm_s * sinf(cogRad);
	gpsVel->vel[2] = 0;
	//dependencies_.gpsVel->sAcc = 0;
			
	//Indicate it is coming from NMEA
	gpsVel->status |= GPS_STATUS_FLAGS_GPS_NMEA_DATA;

	return 0;	
}

/* G_GSA Message
* Provides pDOP and navigation mode (saved to determine 2D/3D mode)
*/
int parse_nmea_gsa(const char msg[], int msgSize, gps_pos_t *gpsPos, int *navMode)
{
	char *ptr = (char *)&msg[7];
	//$xxGSA,opMode,navMode{,svid},PDOP,HDOP,VDOP,systemId*cs<CR><LF>

	//Operation mode
	ptr = ASCII_find_next_field(ptr);
			
	//Navigation mode - save for use to determine 2D / 3D mode
	*navMode = atoi(ptr);
			
	//Skip 13
	for(int i=0;i<13;++i)
	{
		ptr = ASCII_find_next_field(ptr);
	}

	//pDOP
	gpsPos->pDop = (float)atof(ptr);

	return 0;	
}

/* G_GSV Message
* Provides satellite information
* Multiple GSV messages will come in a block. We wait until block is finished before flagging data is ready.
*/
int parse_nmea_gsv(const char msg[], int msgSize, gps_sat_t* gpsSat, int lastGSVmsg[2], int *satCount, uint32_t *cnoSum, uint32_t *cnoCount)
{
	char *ptr = (char *)&msg[7];
	//$xxGSV,numMsg,msgNum,numSV{,svid,elv,az,cno},signalId*cs<CR><LF>
		
	//numMsg
	//int numMsg = atoi(ptr);
	ptr = ASCII_find_next_field(ptr);

	//msgNum
	int msgNum = atoi(ptr);
	ptr = ASCII_find_next_field(ptr);
		
	//numSV
	int numSV = atoi(ptr);
	ptr = ASCII_find_next_field(ptr);
		
	//For some reason the ZED-F9P outputs double messages with the second set having a zero signal strength for satellites for protocol version 27.10 & 27.11
	// (Data sheet for ZED-F9P indicates this message is only supported in version 27.11)
	//Only process the first message
	if(lastGSVmsg[0] != msg[2] || lastGSVmsg[1] < msgNum)
	{
		//save message
		lastGSVmsg[0] = msg[2];
		lastGSVmsg[1] = msgNum;
			
		//Process up to 4 satellites
		int countSat = numSV - (msgNum - 1) * 4;
		if(countSat > 4)
			countSat = 4;

		for(int i=0;i<countSat;++i)
		{
			//svid
			int svid = atoi(ptr);
			ptr = ASCII_find_next_field(ptr);
			
			//elv
			int elv = atoi(ptr);
			ptr = ASCII_find_next_field(ptr);
			
			//az
			int az = atoi(ptr);
			ptr = ASCII_find_next_field(ptr);
			
			//cno
			int cno = atoi(ptr);
			ptr = ASCII_find_next_field(ptr);
			
			//Save data (only if there is room available)
			if(*satCount < MAX_NUM_SAT_CHANNELS && gpsSat)
			{
				auto& svDest = gpsSat->sat[(*satCount)++];
				//IDs are different based on the GNSS type, convert to be the same as UBX message
				switch(msg[2])
				{
					default:
					case 'P':	//GPS, SBAS, QZSS
						if(svid >= 193)	//QZSS
						{
							svDest.gnssId = 5;
							if(svid >= 193)
								svDest.svId = svid-192;
							else
								svDest.svId = svid;
						}
						else if(svid > 32)	//SBAS
						{
							svDest.gnssId = 1;
							if(svid <= 64)
								svDest.svId = svid + 87;
							else
								svDest.svId = svid;
						}
						else //GPS
						{
							svDest.gnssId = 0;
							svDest.svId = svid;
						}
						break;
					case 'L':	//GLONASS
						svDest.gnssId = 6;
						if(svid >= 65)
							svDest.svId = svid - 64;
						else
							svDest.svId = svid;
						break;
					case 'A':	//Galileo
						svDest.gnssId = 2;
						svDest.svId = svid;
						break;
					case 'B':	//BeiDou
						svDest.gnssId = 3;
						svDest.svId = svid;
						break;	
				}
				svDest.cno = cno;
				svDest.elev = elv;
				svDest.azim = az;
				svDest.prRes = 0;
				svDest.flags = 0;
			}
			
			// Calculate the sum and count of non-zero cno values, in order to calculate the cnoMean
			if (cno != 0)
			{
				(*cnoSum) += cno;
				++(*cnoCount);
			}
		}
	}

	return 0;
}