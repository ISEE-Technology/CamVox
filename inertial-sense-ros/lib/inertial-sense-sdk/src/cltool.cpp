/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "cltool.h"
#include "ISDataMappings.h"

cmd_options_t g_commandLineOptions;
serial_port_t g_serialPort;
cInertialSenseDisplay g_inertialSenseDisplay;

int cltool_serialPortSendComManager(CMHANDLE cmHandle, int pHandle, buffer_t* bufferToSend)
{
	(void)cmHandle;
	(void)pHandle;
	return serialPortWrite(&g_serialPort, bufferToSend->buf, bufferToSend->size);
}

bool cltool_setupLogger(InertialSense& inertialSenseInterface)
{
	// Enable logging in continuous background mode
	return inertialSenseInterface.SetLoggerEnabled
	(
		g_commandLineOptions.enableLogging, // enable logger
		g_commandLineOptions.logPath, // path to log to, if empty defaults to DEFAULT_LOGS_DIRECTORY
		cISLogger::ParseLogType(g_commandLineOptions.logType), // log type
		g_commandLineOptions.rmcPreset, // Stream rmc preset
		g_commandLineOptions.maxLogSpacePercent, // max space in percentage of free space to use, 0 for unlimited
		g_commandLineOptions.maxLogFileSize, // each log file will be no larger than this in bytes
		g_commandLineOptions.logSubFolder // log sub folder name
	);

	// Call these elsewhere as needed
// 	inertialSenseInterface.EnableLogger(false);	// Enable/disable during runtime
// 	inertialSenseInterface.CloseLogger();		// Stop logging and save remaining data to file
}

static bool startsWith(const char* str, const char* pre)
{
	size_t lenpre = strlen(pre), lenstr = strlen(str);
	return lenstr < lenpre ? false : strncasecmp(pre, str, lenpre) == 0;
}

#define CL_DEFAULT_BAUD_RATE				IS_COM_BAUDRATE_DEFAULT 
#define CL_DEFAULT_COM_PORT					"*"
#define CL_DEFAULT_DISPLAY_MODE				cInertialSenseDisplay::DMODE_PRETTY 
#define CL_DEFAULT_LOG_TYPE					"dat"
#define CL_DEFAULT_LOGS_DIRECTORY			DEFAULT_LOGS_DIRECTORY
#define CL_DEFAULT_ENABLE_LOGGING			false 
#define CL_DEFAULT_MAX_LOG_FILE_SIZE		1024 * 1024 * 5
#define CL_DEFAULT_MAX_LOG_SPACE_PERCENT	0.5f 
#define CL_DEFAULT_REPLAY_SPEED				1.0
#define CL_DEFAULT_BOOTLOAD_VERIFY			false

bool cltool_parseCommandLine(int argc, char* argv[])
{
	// set defaults
	g_commandLineOptions.baudRate = CL_DEFAULT_BAUD_RATE;
	g_commandLineOptions.comPort = CL_DEFAULT_COM_PORT;
	g_commandLineOptions.displayMode = CL_DEFAULT_DISPLAY_MODE;
	g_commandLineOptions.rmcPreset = 0;
	g_commandLineOptions.enableLogging = CL_DEFAULT_ENABLE_LOGGING;
	g_commandLineOptions.logType = CL_DEFAULT_LOG_TYPE;
	g_commandLineOptions.logPath = CL_DEFAULT_LOGS_DIRECTORY;
	g_commandLineOptions.maxLogFileSize = CL_DEFAULT_MAX_LOG_FILE_SIZE;
	g_commandLineOptions.maxLogSpacePercent = CL_DEFAULT_MAX_LOG_SPACE_PERCENT;
	g_commandLineOptions.replaySpeed = CL_DEFAULT_REPLAY_SPEED;
	g_commandLineOptions.bootloaderVerify = CL_DEFAULT_BOOTLOAD_VERIFY;
	g_commandLineOptions.timeoutFlushLoggerSeconds = 3;
	g_commandLineOptions.asciiMessages = "";
	g_commandLineOptions.updateBootloaderFilename = "";

    g_commandLineOptions.surveyIn.state = 0;
    g_commandLineOptions.surveyIn.maxDurationSec = 15 * 60; // default survey of 15 minutes
    g_commandLineOptions.surveyIn.minAccuracy = 0;

	cltool_outputHelp();

	if(argc <= 1)
	{	// Display usage menu if no options are provided 
		cltool_outputUsage();
		return false;
	}

	// parse command line.  Keep these options in alphabetic order!
	for (int i = 1; i < argc; i++)
	{
		const char* a = argv[i];
        if (startsWith(a, "-asciiMessages="))
        {
            g_commandLineOptions.asciiMessages = &a[15];
        }
        else if (startsWith(a, "-baud="))
		{
			g_commandLineOptions.baudRate = strtol(&a[6], NULL, 10);
		}
		else if (startsWith(a, "-c="))
		{
			g_commandLineOptions.comPort = &a[3];
		}
		else if (startsWith(a, "-dboc"))
		{
			g_commandLineOptions.disableBroadcastsOnClose = true;
		}
		else if (startsWith(a, "-flashConfig="))
		{
			g_commandLineOptions.flashConfig = &a[13];
		}
		else if (startsWith(a, "-flashConfig"))
		{
			g_commandLineOptions.flashConfig = ".";
		}
		else if (startsWith(a, "-host="))
		{
			g_commandLineOptions.host = &a[6];
		}
		else if (startsWith(a, "-h") || startsWith(a, "--h") || startsWith(a, "-help") || startsWith(a, "--help"))
		{
			cltool_outputUsage();
			return false;
		}
		else if (startsWith(a, "-lms="))
		{
			g_commandLineOptions.maxLogSpacePercent = (float)atof(&a[5]);
		}
		else if (startsWith(a, "-lmf="))
		{
			g_commandLineOptions.maxLogFileSize = (uint32_t)strtoul(&a[5], NULL, 10);
		}
        else if (startsWith(a, "-log-flush-timeout="))
        {
            g_commandLineOptions.timeoutFlushLoggerSeconds = strtoul(&a[19], NULLPTR, 10);
        }
        else if (startsWith(a, "-lts="))
		{
			const char* subFolder = &a[5];
			if (*subFolder == '1' || startsWith(subFolder, "true"))
			{
				g_commandLineOptions.logSubFolder = cISLogger::CreateCurrentTimestamp();
			}
			else if (*subFolder == '\0' || *subFolder == '0' || startsWith(subFolder, "false"))
			{
				g_commandLineOptions.logSubFolder = cISLogger::g_emptyString;
			}
			else
			{
				g_commandLineOptions.logSubFolder = subFolder;
			}
		}
		else if (startsWith(a, "-lp="))
		{
			g_commandLineOptions.logPath = &a[4];
		}
		else if (startsWith(a, "-lt="))
		{
			g_commandLineOptions.logType = &a[4];
		}
		else if (startsWith(a, "-lon"))
		{
			g_commandLineOptions.enableLogging = true;
		}
		else if (startsWith(a, "-magRecal"))
		{
			g_commandLineOptions.rmcPreset = 0;
			g_commandLineOptions.magRecal = true;
			g_commandLineOptions.magRecalMode = strtol(a + 9, NULL, 10);
		}
        else if (startsWith(a, "-survey="))
        {
            g_commandLineOptions.rmcPreset = 0;
            g_commandLineOptions.surveyIn.state = strtol(a + 8, NULL, 10);
            int maxDurationSec = strtol(a + 10, NULL, 10);
            if (maxDurationSec > 5)
            {
                g_commandLineOptions.surveyIn.maxDurationSec = maxDurationSec;
            }
        }
		else if (startsWith(a, "-msgBaro="))
		{
			g_commandLineOptions.streamBaro = (int)atof(&a[9]);
		}
		else if (startsWith(a, "-msgBaro"))
		{
			g_commandLineOptions.streamBaro = 50;
		}
		else if (startsWith(a, "-msgDualIMU="))
		{
			g_commandLineOptions.streamDualIMU = (int)atof(&a[12]);
		}
		else if (startsWith(a, "-msgDualIMU"))
		{
			g_commandLineOptions.streamDualIMU = 50;
		}
		else if (startsWith(a, "-msgGPS="))
		{
			g_commandLineOptions.streamGPS = (int)atof(&a[8]);
		}
        else if (startsWith(a, "-msgGPS"))
		{
			g_commandLineOptions.streamGPS = 1;
		}
		else if (startsWith(a, "-msgIMU1="))
		{
			g_commandLineOptions.streamIMU1 = (int)atof(&a[9]);
		}
		else if (startsWith(a, "-msgIMU1"))
		{
			g_commandLineOptions.streamIMU1 = 50;
		}
		else if (startsWith(a, "-msgIMU2=")) 
		{
			g_commandLineOptions.streamIMU2 = (int)atof(&a[9]);
		}
		else if (startsWith(a, "-msgIMU2"))
		{
			g_commandLineOptions.streamIMU2 = 50;
		}
		else if (startsWith(a, "-msgINS1="))
		{
			g_commandLineOptions.streamINS1 = (int)atof(&a[9]);
		}
		else if (startsWith(a, "-msgINS1"))
		{
			g_commandLineOptions.streamINS1 = 50;
		}
		else if (startsWith(a, "-msgINS2="))
		{
			g_commandLineOptions.streamINS2 = (int)atof(&a[9]);
		}
		else if (startsWith(a, "-msgINS2"))
		{
			g_commandLineOptions.streamINS2 = 50;
		}
		else if (startsWith(a, "-msgINS3="))
		{
			g_commandLineOptions.streamINS3 = (int)atof(&a[9]);
		}
		else if (startsWith(a, "-msgINS3"))
		{
			g_commandLineOptions.streamINS3 = 50;
		}
		else if (startsWith(a, "-msgINS4="))
		{
			g_commandLineOptions.streamINS4 = (int)atof(&a[9]);
		}
		else if (startsWith(a, "-msgINS4"))
		{
			g_commandLineOptions.streamINS4 = 50;
		}
		else if (startsWith(a, "-msgMag1="))
		{
			g_commandLineOptions.streamMag1 = (int)atof(&a[9]);
		}
		else if (startsWith(a, "-msgMag1"))
		{
			g_commandLineOptions.streamMag1 = 50;
		}
		else if (startsWith(a, "-msgMag2="))
		{
			g_commandLineOptions.streamMag2 = (int)atof(&a[9]);
		}
		else if (startsWith(a, "-msgMag2"))
		{
			g_commandLineOptions.streamMag2 = 50;
		}
		else if (startsWith(a, "-msgPIMU"))
		{
			g_commandLineOptions.streamDThetaVel = 1;
		}
		else if (startsWith(a, "-msgPresetPPD"))
		{
			g_commandLineOptions.rmcPreset = RMC_PRESET_PPD_BITS;
		}
		else if (startsWith(a, "-msgPresetINS2"))
		{
			g_commandLineOptions.rmcPreset = RMC_PRESET_INS_BITS;
		}
		else if (startsWith(a, "-msgRtkPosRel="))
		{
			g_commandLineOptions.streamRtkPosRel = (int)atof(&a[11]);
		}
		else if (startsWith(a, "-msgRtkPosRel"))
		{
			g_commandLineOptions.streamRtkPosRel = 1;
		}
		else if (startsWith(a, "-msgRtkCmpRel="))
		{
			g_commandLineOptions.streamRtkCmpRel = (int)atof(&a[11]);
		}
		else if (startsWith(a, "-msgRtkCmpRel"))
		{
			g_commandLineOptions.streamRtkCmpRel = 1;
		}
		else if (startsWith(a, "-msgRtkPos="))
		{
		g_commandLineOptions.streamRtkPos = (int)atof(&a[11]);
		}
		else if (startsWith(a, "-msgRtkPos"))
		{
		g_commandLineOptions.streamRtkPos = 1;
		}
		else if (startsWith(a, "-msgRTOS="))
		{
			g_commandLineOptions.streamRTOS = (int)atof(&a[9]);
		}
        else if (startsWith(a, "-msgRTOS"))
		{
			g_commandLineOptions.streamRTOS = 250;
		}
		else if (startsWith(a, "-msgSensorsADC="))
		{
			g_commandLineOptions.streamSensorsADC = (int)atof(&a[15]);
		}
		else if (startsWith(a, "-msgSensorsADC"))
		{
			g_commandLineOptions.streamSensorsADC = 50;
		}
		else if (startsWith(a, "-msgSensors="))
		{
			g_commandLineOptions.streamSysSensors = (int)atof(&a[12]);
		}
		else if (startsWith(a, "-msgSensors"))
		{
			g_commandLineOptions.streamSysSensors = 50;
		}
        else if (startsWith(a, "-persistent"))
        {
            g_commandLineOptions.persistentMessages = true;
        }
        else if (startsWith(a, "-q"))
		{
			g_commandLineOptions.displayMode = cInertialSenseDisplay::DMODE_QUIET;
		}
		else if (startsWith(a, "-rp="))
		{
			g_commandLineOptions.replayDataLog = true;
			g_commandLineOptions.logPath = &a[4];
		}
		else if (startsWith(a, "-rs="))
		{
			g_commandLineOptions.replayDataLog = true;
			g_commandLineOptions.replaySpeed = (float)atof(&a[4]);
		}
        else if (startsWith(a, "-reset"))
        {
            g_commandLineOptions.softwareReset = true;
        }
		else if (startsWith(a, "-r"))
		{
			g_commandLineOptions.replayDataLog = true;
		}
        else if (startsWith(a, "-stats"))
		{
			g_commandLineOptions.displayMode = cInertialSenseDisplay::DMODE_STATS;
		}
		else if (startsWith(a, "-svr=") || startsWith(a, "-srv="))
		{
			g_commandLineOptions.serverConnection = &a[5];
		}
		else if (startsWith(a, "-s"))
		{
			g_commandLineOptions.displayMode = cInertialSenseDisplay::DMODE_SCROLL;
		}
		else if (startsWith(a, "-ub="))
		{
			g_commandLineOptions.updateBootloaderFilename = &a[4];
		}
        else if (startsWith(a, "-uf="))
        {
            g_commandLineOptions.updateAppFirmwareFilename = &a[4];
        }
		else if (startsWith(a, "-uv"))
		{
			g_commandLineOptions.bootloaderVerify = true;
		}
		else
		{
			cout << "Unrecognized command line option: " << a << endl;
			cltool_outputUsage();
			return false;
		}
	}

	// We are either using a serial port or replaying data
	if ((g_commandLineOptions.comPort.length() == 0) && !g_commandLineOptions.replayDataLog)
	{
		cltool_outputUsage();
		return false;
	}
	else if (g_commandLineOptions.updateAppFirmwareFilename.length() != 0 && g_commandLineOptions.comPort.length() == 0)
	{
		cout << "Use COM_PORT option \"-c=\" with bootloader" << endl;
		return false;
	}
    else if (g_commandLineOptions.updateBootloaderFilename.length() != 0 && g_commandLineOptions.comPort.length() == 0)
    {
        cout << "Use COM_PORT option \"-c=\" with bootloader" << endl;
        return false;
    }
        
	return true;
}

bool cltool_replayDataLog()
{
	if (g_commandLineOptions.logPath.length() == 0)
	{
		cout << "Please specify the replay log path!" << endl;
		return false;
	}

	cISLogger logger;
	if (!logger.LoadFromDirectory(g_commandLineOptions.logPath, cISLogger::ParseLogType(g_commandLineOptions.logType), { "ALL" }))
	{
		cout << "Failed to load log files: " << g_commandLineOptions.logPath << endl;
		return false;
	}

	cout << "Replaying log files: " << g_commandLineOptions.logPath << endl;
	p_data_t *data;
	while ((data = logger.ReadData()) != NULL)
	{
		g_inertialSenseDisplay.ProcessData(data, g_commandLineOptions.replayDataLog, g_commandLineOptions.replaySpeed);

// 		if (data->hdr.id == DID_GPS1_RAW)
// 		{
// 			// Insert your code for processing data here
// 		}
	}

	cout << "Done replaying log files: " << g_commandLineOptions.logPath << endl;
	g_inertialSenseDisplay.Goodbye();
	return true;
}

void cltool_outputUsage()
{
	cout << boldOff;
	cout << "-----------------------------------------------------------------" << endl;
	cout << endlbOn;
	cout << "DESCRIPTION" << endlbOff;
	cout << "    Command line utility for communicating, logging, and updating" << endl;
	cout << "    firmware with Inertial Sense product line." << endl;
	cout << endlbOn;
	cout << "EXAMPLES" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c="  <<     EXAMPLE_PORT << " -msgPresetPPD            " << EXAMPLE_SPACE_1 << boldOff << " # stream post processing data (PPD) with INS2" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c="  <<     EXAMPLE_PORT << " -msgPresetPPD -lon       " << EXAMPLE_SPACE_1 << boldOff << " # stream PPD + INS2 data, logging" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c="  <<     EXAMPLE_PORT << " -msgPresetPPD -lon -lts=1" << EXAMPLE_SPACE_1 << boldOff << " # stream PPD + INS2 data, logging, dir timestamp" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c="  <<     EXAMPLE_PORT << " -baud=115200 -msgINS2 -msgGPS=10 -msgBaro" << boldOff << " # stream multiple at 115200 bps, GPS data streamed at 10 times the base period (200ms x 10 = 2 sec)" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -rp=" <<     EXAMPLE_LOG_DIR                                           << boldOff << " # replay log files from a folder" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c="  <<     EXAMPLE_PORT << " -b=" << EXAMPLE_FIRMWARE_FILE << " -bl=" << EXAMPLE_BOOTLOADER_FILE << " -bv" << boldOff << " # bootload application firmware and update bootloader if needed" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c=* -baud=921600              "                    << EXAMPLE_SPACE_2 << boldOff << " # 921600 bps baudrate on all serial ports" << endlbOn;
	cout << endlbOn;
	cout << "OPTIONS (General)" << endl;
	cout << "    -h --help" << boldOff << "       display this help menu" << endlbOn;
	cout << "    -c=" << boldOff << "COM_PORT     select the serial port. Set COM_PORT to \"*\" for all ports and \"*4\" to use" << endlbOn;
	cout << "       " << boldOff << "             only the first four ports. " <<  endlbOn;
	cout << "    -baud=" << boldOff << "BAUDRATE  set serial port baudrate.  Options: " << IS_BAUDRATE_115200 << ", " << IS_BAUDRATE_230400 << ", " << IS_BAUDRATE_460800 << ", " << IS_BAUDRATE_921600 << " (default)" << endlbOn;
	cout << "    -magRecal[n]" << boldOff << "    recalibrate magnetometers: 0=multi-axis, 1=single-axis" << endlbOn;
    cout << "    -q" << boldOff << "              quiet mode, no display" << endlbOn;
    cout << "    -reset         " << boldOff << " issue software reset.  Use caution." << endlbOn;
    cout << "    -s" << boldOff << "              scroll displayed messages to show history" << endlbOn;
	cout << "    -stats" << boldOff << "          display statistics of data received" << endlbOn;
    cout << "    -survey=[s],[d]" << boldOff << " survey-in and store base position to refLla: s=[" << SURVEY_IN_STATE_START_3D << "=3D, " << SURVEY_IN_STATE_START_FLOAT << "=float, " << SURVEY_IN_STATE_START_FIX << "=fix], d=durationSec" << endlbOn;
	cout << "    -uf=" << boldOff << "FILEPATH    update firmware using .hex file FILEPATH.  Add -baud=115200 for systems w/ baud rate limits." << endlbOn;
	cout << "    -ub=" << boldOff << "BLFILEPATH  update bootloader using .bin file BLFILEPATH. Combine with -b option to check version and updated if needed." << endlbOn;
	cout << "    -uv" << boldOff << "             verify after firmware update." << endlbOn;

	cout << endlbOn;
	cout << "OPTIONS (Message Streaming)" << endl;
	cout << "    -msgPresetPPD " << boldOff << "  stream preset: post processing data sets" << endlbOn;
	cout << "    -msgPresetINS2" << boldOff << "  stream preset: INS2 sets" << endlbOn;
	cout << "    -msgINS[n] *   " << boldOff << "  stream DID_INS_[n], where [n] = 1, 2, 3 or 4 (without brackets)" << endlbOn;
	cout << "    -msgDualIMU *  " << boldOff << "  stream DID_DUAL_IMU" << endlbOn;
	cout << "    -msgPIMU       " << boldOff << "  stream DID_PREINTEGRATED_IMU" << endlbOn;
	cout << "    -msgMag[n] *   " << boldOff << "  stream DID_MAGNETOMETER_[n], where [n] = 1 or 2 (without brackets)" << endlbOn;
	cout << "    -msgBaro *     " << boldOff << "  stream DID_BAROMETER" << endlbOn;
	cout << "    -msgGPS *      " << boldOff << "  stream DID_GPS_NAV" << endlbOn;
	cout << "    -msgSensors *  " << boldOff << "  stream DID_SYS_SENSORS" << endlbOn;
	cout << "    -msgRtkPos *   " << boldOff << "  stream DID_GPS1_RTK_POS" << endlbOn;
	cout << "    -msgRtkPosRel *" << boldOff << "  stream DID_GPS1_RTK_POS_REL" << endlbOn;
	cout << "    -msgRtkCmpRel *" << boldOff << "  stream DID_GPS2_RTK_CMP_REL" << endlbOn;
	cout << "    -persistent   " << boldOff << "  save current streams as persistent messages enabled on startup" << endlbOn;
	cout << "                * Message can be appended with =<PERIODMULTIPLE> to change message frequency. Period is then equal to message" << endlbOn; 
	cout << "                  source times the PERIODMULTIPLE. If not appended the data will stream at a default rate." << endlbOn;
	cout << "                  Example: -msgINS2=10 will stream data at startupNavDtMs x 10" << endlbOn;
    cout << endlbOn;
	cout << "OPTIONS (Logging to file, disabled by default)" << endl;
	cout << "    -lon" << boldOff << "            enable logging" << endlbOn;
	cout << "    -lt=" << boldOff << "TYPE        log type dat (default), sdat, kml or csv" << endlbOn;
	cout << "    -lp=" << boldOff << "PATH        log data to path (default: ./" << CL_DEFAULT_LOGS_DIRECTORY << ")" << endlbOn;
	cout << "    -lms=" << boldOff << "PERCENT    log max space in percent of free space (default: " << CL_DEFAULT_MAX_LOG_SPACE_PERCENT << ")" << endlbOn;
	cout << "    -lmf=" << boldOff << "BYTES      log max file size in bytes (default: " << CL_DEFAULT_MAX_LOG_FILE_SIZE << ")" << endlbOn;
	cout << "    -lts=" << boldOff << "0          log sub folder, 0 or blank for none, 1 for timestamp, else use as is" << endlbOn;
	cout << "    -r" << boldOff << "              replay data log from default path" << endlbOn;
	cout << "    -rp=" << boldOff << "PATH        replay data log from PATH" << endlbOn;
	cout << "    -rs=" << boldOff << "SPEED       replay data log at x SPEED. SPEED=0 runs as fast as possible." << endlbOn;
	cout << endlbOn;
	cout << "OPTIONS (Read or write flash configuration)" << endl;
	cout << "    -flashConfig" << boldOff << "    list all \"keys\" and \"values\"" << endlbOn;
	cout << "   \"-flashConfig=key=value|key=value\" " << boldOff <<  endlbOn;
	cout << "        " << boldOff << "            set key / value pairs in flash config. Surround with \"quotes\" when using pipe operator." << endlbOn;
	cout << "EXAMPLES" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c=" << EXAMPLE_PORT << " -flashConfig  " << boldOff << "# Read from device and print all keys and values" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c=" << EXAMPLE_PORT << " -flashConfig=insRotation[0]=1.5708|insOffset[1]=1.2  " << boldOff << "# Set multiple flashConfig values" << endlbOn;
	cout << endlbOn;
	cout << "OPTIONS (Client / Server)" << endl;
	cout << "    -svr=" << boldOff << "INFO       used to retrieve external data and send to the uINS. Examples:" << endl;
	cout << "        - SERIAL:        -svr=RTCM3:SERIAL:" << EXAMPLE_PORT << ":57600         (port, baud rate)" << endl;
	cout << "        - RTCM3:         -svr=RTCM3:192.168.1.100:7777:URL:user:password" << endl;
	cout << "                                                              (URL, user, password optional)" << endl;
	cout << "        - UBLOX data:    -svr=UBLOX:192.168.1.100:7777        (no URL, user or password)" << endl;
	cout << "        - InertialSense: -svr=IS:192.168.1.100:7777           (no URL, user or password)" << endlbOn;
	cout << "    -host=" << boldOff << "IP:PORT   used to host a TCP/IP InertialSense server. Examples:" << endl;
	cout << "                         -host=:7777                          (IP is optional)" << endl;
	cout << "                         -host=192.168.1.43:7777" << endl;

	cout << boldOff;   // Last line.  Leave bold text off on exit.
}

void cltool_outputHelp()
{
	cout << endlbOff << "Run \"" << boldOn << "cltool -h" << boldOff << "\" to display the help menu." << endl;
}

bool cltool_updateFlashConfig(InertialSense& inertialSenseInterface, string flashConfigString)
{
	nvm_flash_cfg_t flashConfig = inertialSenseInterface.GetFlashConfig();
	const map_name_to_info_t& flashMap = *cISDataMappings::GetMapInfo(DID_FLASH_CONFIG);

	if (flashConfigString.length() < 2)
	{
		// read flash config and display
		data_mapping_string_t stringBuffer;
		cout << "Current flash config" << endl;
		for (map_name_to_info_t::const_iterator i = flashMap.begin(); i != flashMap.end(); i++)
		{
			if (cISDataMappings::DataToString(i->second, NULL, (const uint8_t*)&flashConfig, stringBuffer))
			{
				cout << i->second.name << " = " << stringBuffer << endl;
			}
		}
		return false;
	}
	else
	{
		vector<string> keyValues;
		splitString(flashConfigString, "|", keyValues);
		for (size_t i = 0; i < keyValues.size(); i++)
		{
			vector<string> keyAndValue;
			splitString(keyValues[i], "=", keyAndValue);
			if (keyAndValue.size() == 2)
			{
				if (flashMap.find(keyAndValue[0]) == flashMap.end())
				{
					cout << "Unrecognized flash config key '" << keyAndValue[0] << "' specified, ignoring." << endl;
				}
				else
				{
					const data_info_t& info = flashMap.at(keyAndValue[0]);
					int radix = (keyAndValue[1].compare(0, 2, "0x") == 0 ? 16 : 10);
					int substrIndex = 2 * (radix == 16); // skip 0x for hex
					const string& str = keyAndValue[1].substr(substrIndex);
					cISDataMappings::StringToData(str.c_str(), (int)str.length(), NULL, (uint8_t*)&flashConfig, info, radix);
					cout << "Updated flash config key '" << keyAndValue[0] << "' to '" << keyAndValue[1].c_str() << "'" << endl;
				}
			}
		}
		inertialSenseInterface.SetFlashConfig(flashConfig);
		g_inertialSenseDisplay.Clear();
		return true;
	}
}
