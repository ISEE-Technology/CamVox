#ifndef PROTOCOL_NMEA_H_
#define PROTOCOL_NMEA_H_

#include "data_sets.h"


enum eNmeaMsgIdUint
{
	ASCII_MSG_ID_ASCB = 0x41534342,		// ASCII messages broadcast periods
	ASCII_MSG_ID_STPB = 0x53545042,		// Stop broadcasts on all ports
	ASCII_MSG_ID_STPC = 0x53545043,		// Stop broadcasts on current port
	ASCII_MSG_ID_BLEN = 0x424c454e,		// Enable bootloader on uINS
	ASCII_MSG_ID_EBLE = 0x45424c45,		// Enable bootloader on EVB
	ASCII_MSG_ID_NELB = 0x4e454c42,		// Enable SAM-BA mode
	ASCII_MSG_ID_SRST = 0x53525354,		// Software reset
	ASCII_MSG_ID_INFO = 0x494e464f,		// Device info
	ASCII_MSG_ID_PERS = 0x50455253,		// Save perstent messages
};


//////////////////////////////////////////////////////////////////////////
// Utility functions
//////////////////////////////////////////////////////////////////////////
uint32_t ASCII_compute_checksum(uint8_t* str, int size);
char *ASCII_find_next_field(char *str);
double ddmm2deg(double ddmm);
void set_gpsPos_status_mask(uint32_t *status, uint32_t state, uint32_t mask);

//////////////////////////////////////////////////////////////////////////
// DID to NMEA
//////////////////////////////////////////////////////////////////////////
int dev_info_to_nmea_info(char a[], const int aSize, dev_info_t &info);
int tow_to_nmea_ptow(char a[], const int aSize, double imuTow, double insTow, unsigned int gpsWeek);
int dimu_to_nmea_pimu(char a[], const int aSize, dual_imu_t &dImu);
int pimu_to_nmea_ppimu(char a[], const int aSize, preintegrated_imu_t &pimu);
int ins1_to_nmea_pins1(char a[], const int aSize, ins_1_t &ins1);
int ins2_to_nmea_pins2(char a[], const int aSize, ins_2_t &ins2);
int strobe_to_nmea_pstrb(char a[], const int aSize, strobe_in_time_t &strobe);
int gps_to_nmea_pgpsp(char a[], const int aSize, gps_pos_t &pos, gps_vel_t &vel);
int gps_to_nmea_gga(char a[], const int aSize, gps_pos_t &pos);
int gps_to_nmea_gll(char a[], const int aSize, gps_pos_t &pos);
int gps_to_nmea_gsa(char a[], const int aSize, gps_pos_t &pos, gps_sat_t &sat);
int gps_to_nmea_rmc(char a[], const int aSize, gps_pos_t &pos, gps_vel_t &vel, float magDeclination);
int gps_to_nmea_zda(char a[], const int aSize, gps_pos_t &pos);
int gps_to_nmea_pashr(char a[], const int aSize, gps_pos_t &pos, ins_1_t &ins1, float heave, inl2_ned_sigma_t &sigma);

//////////////////////////////////////////////////////////////////////////
// NMEA parse
//////////////////////////////////////////////////////////////////////////
uint32_t parse_nmea_ascb(int pHandle, const char msg[], int msgSize, ascii_msgs_t asciiPeriod[]);
int parse_nmea_zda(const char msgBuf[], int msgSize, double &day, double &month, double &year);
int parse_nmea_gns(const char msgBuf[], int msgSize, gps_pos_t *gpsPos, double datetime[6], int *satsUsed, int navMode);
int parse_nmea_gga(const char msg[], int msgSize, gps_pos_t *gpsPos, double datetime[6], int *satsUsed, int navMode);
int parse_nmea_rmc(const char msg[], int msgSize, gps_vel_t *gpsVel, double datetime[6], int *satsUsed, int navMode);
int parse_nmea_gsa(const char msg[], int msgSize, gps_pos_t *gpsPos, int *navMode);
int parse_nmea_gsv(const char msg[], int msgSize, gps_sat_t* gpsSat, int lastGSVmsg[2], int *satPointer, uint32_t *cnoSum, uint32_t *cnoCount);



#endif /* PROTOCOL_NMEA_H_ */
