/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __CLTOOL_H__
#define __CLTOOL_H__

#include <stdio.h>
#include <iostream>
#include <iomanip>      // std::setw
#include <algorithm>
#include <string>

// change these includes to be the correct path for your system
#include "InertialSense.h" // best to include this file first
#include "ISDisplay.h"
#include "ISUtilities.h"

using namespace std;

#define APP_NAME                "cltool"
#if PLATFORM_IS_WINDOWS
#define APP_EXT                 ".exe"
#define EXAMPLE_PORT            "COM5"
#define EXAMPLE_LOG_DIR         "c:\\logs\\20170117_222549    "
#define EXAMPLE_FIRMWARE_FILE   "c:\\fw\\IS_uINS-3.hex"
#define EXAMPLE_BOOTLOADER_FILE "c:\\fw\\SAMx70-Bootloader.bin"
#define EXAMPLE_SPACE_1         "    "
#define EXAMPLE_SPACE_2         ""
#else
#define APP_EXT	                ""
#define EXAMPLE_PORT            "/dev/ttyS2"
#define EXAMPLE_LOG_DIR         "logs/20170117_222549           "
#define EXAMPLE_FIRMWARE_FILE   "fw/IS_uINS-3.hex "
#define EXAMPLE_BOOTLOADER_FILE "fw/SAMx70-Bootloader.bin "
#define EXAMPLE_SPACE_1         "  "
#define EXAMPLE_SPACE_2			"    "
#endif

typedef struct
{
	// parsed
	string comPort; // -c=com_port
	string updateAppFirmwareFilename; // -b=file_name
    string updateBootloaderFilename; // -ub=file_name
    bool bootloaderVerify; // -bv
    bool replayDataLog;
    bool softwareReset;
    bool magRecal;
    uint32_t magRecalMode;
    survey_in_t surveyIn;
    string asciiMessages;
	double replaySpeed;
	int displayMode;

	uint64_t rmcPreset;
    bool persistentMessages;
	int streamINS1;
	int streamINS2;
	int streamINS3;
	int streamINS4;
	int streamDualIMU;
	int streamIMU1;
	int streamIMU2;
    int streamGPS;
    int streamRtkPos;
	int streamRtkPosRel;
	int streamRtkCmpRel;
    int streamMag1;
	int streamMag2;
	int streamBaro;
	int streamSysSensors;
	int streamDThetaVel;
	int streamRTOS;
	int streamSensorsADC;

	bool enableLogging;
	string logType; // -lt=dat
	string logPath; // -lp=path
	float maxLogSpacePercent; // -lms=max_space_mb
	uint32_t maxLogFileSize; // -lmf=max_file_size
	string logSubFolder; // -lts=1
	int baudRate; // -baud=3000000
	bool disableBroadcastsOnClose;

	string serverConnection; // -svr=type:host:port:url:user:password
	string host; // -host=ip:port

	string flashConfig;

	uint32_t timeoutFlushLoggerSeconds;
} cmd_options_t;

extern cmd_options_t g_commandLineOptions;
extern serial_port_t g_serialPort;
extern cInertialSenseDisplay g_inertialSenseDisplay;
extern bool g_ctrlCPressed;

int cltool_main(int argc, char* argv[]);
int cltool_serialPortSendComManager(CMHANDLE cmHandle, int pHandle, buffer_t* bufferToSend);

// returns false if failure
bool cltool_setupLogger(InertialSense& inertialSenseInterface);
bool cltool_parseCommandLine(int argc, char* argv[]);
bool cltool_replayDataLog();
void cltool_outputUsage();
void cltool_outputHelp();
bool cltool_updateFlashConfig(InertialSense& inertialSenseInterface, string flashConfig); // true if should continue

#endif // __CLTOOL_H__

