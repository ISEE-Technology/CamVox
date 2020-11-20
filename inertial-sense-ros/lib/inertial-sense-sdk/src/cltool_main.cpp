/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
*  main.cpp
*
*  (c) 2014 Inertial Sense, LLC
*
*  The Inertial Sense Command Line Tool (cltool) shows how easy it is to communicate with the uINS, log data, update firmware and more.
*
*  The following keywords are found within this file to identify instructions
*  for SDK implementation.  
*
*    KEYWORD:                  SDK Implementation
*    [C++ COMM INSTRUCTION]    - C++ binding API - InertialSense class with binary communication 
*                                protocol and serial port support for Linux and Windows.
*    [C COMM INSTRUCTION]      - C binding API - Com Manager with binary communication protocol.
*    [LOGGER INSTRUCTION]      - Data logger.
*    [BOOTLOADER INSTRUCTION]  - Firmware update feature.
*
*  This app is designed to be compiled in Linux and Windows.  When using MS
*  Visual Studio IDE, command line arguments can be supplied by right clicking 
*  the project in solution explorer and then selecting properties -> debugging -> command line arguments
*/

// Contains command line parsing and utility functions.  Include this in your project to use these utility functions.
#include "cltool.h"

static bool output_server_bytes(InertialSense* i, const char* prefix = "", const char* suffix = "")
{
	static float serverKBps = 0;
	static uint64_t serverByteCount = 0;
	static uint64_t updateCount = 0;
	static uint64_t serverByteRateTimeMsLast = 0;
	static uint64_t serverByteCountLast = 0;

	if (g_inertialSenseDisplay.GetDisplayMode() != cInertialSenseDisplay::DMODE_QUIET)
	{
		uint64_t newServerByteCount = i->GetClientServerByteCount();
		if (serverByteCount != newServerByteCount)
		{
			serverByteCount = newServerByteCount;

			// Data rate of server bytes
			uint64_t timeMs = getTickCount();
			uint64_t dtMs = timeMs - serverByteRateTimeMsLast;
			if (dtMs >= 1000)
			{
				uint64_t serverBytesDelta = serverByteCount - serverByteCountLast;
				serverKBps = ((float)serverBytesDelta / (float)dtMs);

				// Update history
				serverByteCountLast = serverByteCount;
				serverByteRateTimeMsLast = timeMs;
			}
			printf("%sServer: %02" PRIu64 " (%3.1f KB/s : %lld)     %s", prefix, (++updateCount) % 100, serverKBps, (long long)i->GetClientServerByteCount(), suffix);
			return true;
		}
	}
	return false;
}

static void output_client_bytes(InertialSense* i)
{
	if (g_inertialSenseDisplay.GetDisplayMode() == cInertialSenseDisplay::DMODE_QUIET)
	{
		return;
	}

	if (g_inertialSenseDisplay.GetDisplayMode() != cInertialSenseDisplay::DMODE_SCROLL)
	{
		char suffix[256];
		com_manager_status_t* status = comManagerGetStatus(0);
		suffix[0] = '\0';
		if (status != NULLPTR && status->communicationErrorCount != 0 && g_commandLineOptions.displayMode != cInertialSenseDisplay::DMODE_QUIET)
		{
			snprintf(suffix, sizeof(suffix), "Com errors: %d     ", status->communicationErrorCount);
		}
		g_inertialSenseDisplay.Home();
		if (i->GetClientServerByteCount() == 0)
		{
			cout << endl << suffix;
		}
		else
		{
			output_server_bytes(i, "\n", suffix);
		}
	}
}

// [C++ COMM INSTRUCTION] STEP 5: Handle received data 
static void cltool_dataCallback(InertialSense* i, p_data_t* data, int pHandle)
{
    (void)i;
    (void)pHandle;
	// Print data to terminal
	g_inertialSenseDisplay.ProcessData(data);
	output_client_bytes(i);

#if 0

	// uDatasets is a union of all datasets that we can receive.  See data_sets.h for a full list of all available datasets. 
	uDatasets d = {};
	copyDataPToStructP(&d, data, sizeof(uDatasets));
    
	// Example of how to access dataset fields.
	switch (data->hdr.id)
	{
	case DID_INS_2:
		d.ins2.qn2b;		// quaternion attitude 
		d.ins2.uvw;			// body velocities
		d.ins2.lla;			// latitude, longitude, altitude
		break;
	case DID_INS_1:
		d.ins1.theta;		// euler attitude
		d.ins1.lla;			// latitude, longitude, altitude
		break;
	case DID_DUAL_IMU:				
		d.dualImu;      
		break;
	case DID_PREINTEGRATED_IMU:		
		d.pImu;    
		break;
	case DID_GPS_NAV:				
		d.gpsNav;       
		break;
	case DID_MAGNETOMETER_1:		
		d.mag;          
		break;
	case DID_MAGNETOMETER_2:		
		d.mag;          
		break;
	case DID_BAROMETER:				
		d.baro;         
		break;
	case DID_SYS_SENSORS:			
		d.sysSensors;   
		break;
	}
    
#endif
    
}

// Where we tell the uINS what data to send and at what rate.  
// "cltool_dataCallback()" is registered as the callback functions for all received data.
// All DID messages are found in data_sets.h
static bool cltool_setupCommunications(InertialSense& inertialSenseInterface)
{
	inertialSenseInterface.StopBroadcasts();	// Stop streaming any prior messages

	if (g_commandLineOptions.asciiMessages.size() != 0)
	{
		serialPortWriteAscii(inertialSenseInterface.GetSerialPort(), g_commandLineOptions.asciiMessages.c_str(), (int)g_commandLineOptions.asciiMessages.size());
		return true;
	}

	// ask for device info every 2 seconds
	inertialSenseInterface.BroadcastBinaryData(DID_DEV_INFO, 2000);

	// depending on command line options. stream various data sets
	if (g_commandLineOptions.streamINS1)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_INS_1, g_commandLineOptions.streamINS1);
	}
	if (g_commandLineOptions.streamINS2)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_INS_2, g_commandLineOptions.streamINS2);
	}
	if (g_commandLineOptions.streamINS3)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_INS_3, g_commandLineOptions.streamINS3);
	}
	if (g_commandLineOptions.streamINS4)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_INS_4, g_commandLineOptions.streamINS4);
	}
	if (g_commandLineOptions.streamSysSensors)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_SYS_SENSORS, g_commandLineOptions.streamSysSensors);
	}
	if (g_commandLineOptions.streamDualIMU)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_DUAL_IMU, g_commandLineOptions.streamDualIMU);
	}
	if (g_commandLineOptions.streamDThetaVel)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_PREINTEGRATED_IMU, g_commandLineOptions.streamDThetaVel);
	}
	if (g_commandLineOptions.streamGPS)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_GPS1_POS, g_commandLineOptions.streamGPS);
	}
    if (g_commandLineOptions.streamRtkPos)
    {
        inertialSenseInterface.BroadcastBinaryData(DID_GPS1_RTK_POS, g_commandLineOptions.streamRtkPos);
    }
    if (g_commandLineOptions.streamRtkPosRel)
    {
        inertialSenseInterface.BroadcastBinaryData(DID_GPS1_RTK_POS_REL, g_commandLineOptions.streamRtkPosRel);
    }
	if (g_commandLineOptions.streamRtkCmpRel)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_GPS2_RTK_CMP_REL, g_commandLineOptions.streamRtkCmpRel);
	}
    if (g_commandLineOptions.streamMag1)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_MAGNETOMETER_1, g_commandLineOptions.streamMag1);
	}
	if (g_commandLineOptions.streamMag2)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_MAGNETOMETER_2, g_commandLineOptions.streamMag2);
	}
	if (g_commandLineOptions.streamBaro)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_BAROMETER, g_commandLineOptions.streamBaro);
	}
	if (g_commandLineOptions.streamRTOS)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_RTOS_INFO, g_commandLineOptions.streamRTOS);
        system_command_t cfg;
        cfg.command = SYS_CMD_ENABLE_RTOS_STATS;
        cfg.invCommand = ~cfg.command;
		inertialSenseInterface.SendRawData(DID_SYS_CMD, (uint8_t*)&cfg, sizeof(system_command_t), 0);
	}
	if (g_commandLineOptions.streamSensorsADC)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_SENSORS_ADC, g_commandLineOptions.streamSensorsADC);
	}
	if (g_commandLineOptions.timeoutFlushLoggerSeconds > 0)
	{
		inertialSenseInterface.SetTimeoutFlushLoggerSeconds(g_commandLineOptions.timeoutFlushLoggerSeconds);
	}
	if (g_commandLineOptions.magRecal)
	{	
		// Enable broadcase of DID_MAG_CAL so we can observe progress and tell when the calibration is done (i.e. DID_MAG_CAL.recalCmd == 100).
		inertialSenseInterface.BroadcastBinaryData(DID_MAG_CAL, 100);
		// Enable mag recal
		inertialSenseInterface.SendRawData(DID_MAG_CAL, (uint8_t*)&g_commandLineOptions.magRecalMode, sizeof(g_commandLineOptions.magRecalMode), offsetof(mag_cal_t, recalCmd));
	}
    if (g_commandLineOptions.surveyIn.state)
    {   // Enable mult-axis 
        inertialSenseInterface.SendRawData(DID_SURVEY_IN, (uint8_t*)&g_commandLineOptions.surveyIn, sizeof(survey_in_t), 0);
    }
	if (g_commandLineOptions.rmcPreset)
	{
		inertialSenseInterface.BroadcastBinaryDataRmcPreset(g_commandLineOptions.rmcPreset, RMC_OPTIONS_PRESERVE_CTRL);
	}
    if (g_commandLineOptions.persistentMessages)
    {   // Save persistent messages to flash
        system_command_t cfg;
        cfg.command = SYS_CMD_SAVE_PERSISTENT_MESSAGES;
        cfg.invCommand = ~cfg.command;
        inertialSenseInterface.SendRawData(DID_SYS_CMD, (uint8_t*)&cfg, sizeof(system_command_t), 0);
    }
    if (g_commandLineOptions.softwareReset)
    {   // Issue software reset
        system_command_t cfg;
        cfg.command = SYS_CMD_SOFTWARE_RESET;
        cfg.invCommand = ~cfg.command;
        inertialSenseInterface.SendRawData(DID_SYS_CMD, (uint8_t*)&cfg, sizeof(system_command_t), 0);
    }

	if (g_commandLineOptions.serverConnection.length() != 0)
	{
		if (g_commandLineOptions.serverConnection.find("RTCM3:") == 0 ||
			g_commandLineOptions.serverConnection.find("IS:") == 0 ||
			g_commandLineOptions.serverConnection.find("UBLOX:") == 0)
		{
			if (!inertialSenseInterface.OpenServerConnection(g_commandLineOptions.serverConnection))
			{
				cout << "Failed to connect to server." << endl;
			}
		}
		else
		{
			cout << "Invalid server connection, must prefix with RTCM3:, IS: or UBLOX:, " << g_commandLineOptions.serverConnection << endl;
			return false;
		}
	}
	if (g_commandLineOptions.flashConfig.length() != 0)
	{
		return cltool_updateFlashConfig(inertialSenseInterface, g_commandLineOptions.flashConfig);
	}
	return true;
}

static int cltool_updateAppFirmware()
{
	// [BOOTLOADER INSTRUCTION] Update firmware
	cout << "Updating application firmware using file at " << g_commandLineOptions.updateAppFirmwareFilename << endl;
	vector<InertialSense::bootloader_result_t> results = InertialSense::BootloadFile(g_commandLineOptions.comPort, 
        g_commandLineOptions.updateAppFirmwareFilename, 
		g_commandLineOptions.updateBootloaderFilename,
        g_commandLineOptions.baudRate, 
        bootloadUploadProgress,
		(g_commandLineOptions.bootloaderVerify ? bootloadVerifyProgress : 0),
		bootloadStatusInfo);
	cout << endl << "Results:" << endl;
	int errorCount = 0;
	for (size_t i = 0; i < results.size(); i++)
	{
		cout << results[i].port << ": " << (results[i].error.size() == 0 ? "Success\n" : results[i].error);
		errorCount += (int)(results[i].error.size() != 0);
	}
	if (errorCount != 0)
	{
		cout << endl << errorCount << " ports failed." << endl;
	}
	return (errorCount == 0 ? 0 : -1);
}

static int cltool_updateBootloader()
{
    cout << "Updating bootloader using file at " << g_commandLineOptions.updateBootloaderFilename << endl;
    vector<InertialSense::bootloader_result_t> results = InertialSense::BootloadFile(g_commandLineOptions.comPort, 
        g_commandLineOptions.updateBootloaderFilename,
		"",
        g_commandLineOptions.baudRate, 
        bootloadUploadProgress,
        (g_commandLineOptions.bootloaderVerify ? bootloadVerifyProgress : 0), 
		bootloadStatusInfo,
        true);
    cout << endl << "Results:" << endl;
    int errorCount = 0;
    for (size_t i = 0; i < results.size(); i++)
    {
        cout << results[i].port << ": " << (results[i].error.size() == 0 ? "Success\n" : results[i].error);
        errorCount += (int)(results[i].error.size() != 0);
    }
    if (errorCount != 0)
    {
        cout << endl << errorCount << " ports failed." << endl;
    }
    return (errorCount == 0 ? 0 : -1);
}

static int cltool_createHost()
{
	InertialSense inertialSenseInterface;
	if (!inertialSenseInterface.Open(g_commandLineOptions.comPort.c_str(), g_commandLineOptions.baudRate))
	{
		cout << "Failed to open serial port at " << g_commandLineOptions.comPort.c_str() << endl;
		return -1;
	}
	else if (g_commandLineOptions.flashConfig.length() != 0 && !cltool_updateFlashConfig(inertialSenseInterface, g_commandLineOptions.flashConfig))
	{
		cout << "Failed to update flash config" << endl;
		return -1;
	}
	else if (!inertialSenseInterface.CreateHost(g_commandLineOptions.host))
	{
		cout << "Failed to create host at " << g_commandLineOptions.host << endl;
		return -1;
	}

	inertialSenseInterface.StopBroadcasts();

	while (!g_inertialSenseDisplay.ControlCWasPressed())
	{
		inertialSenseInterface.Update();
		g_inertialSenseDisplay.Home();
		output_server_bytes(&inertialSenseInterface);
	}
	cout << "Shutting down..." << endl;

	// close the interface cleanly, this ensures serial port and any logging are shutdown properly
	inertialSenseInterface.Close();
	
	return 0;
}

static int inertialSenseMain()
{	
	// clear display
	g_inertialSenseDisplay.SetDisplayMode((cInertialSenseDisplay::eDisplayMode)g_commandLineOptions.displayMode);
	g_inertialSenseDisplay.Clear();

	// if replay data log specified on command line, do that now and return
	if (g_commandLineOptions.replayDataLog)
	{	
		// [REPLAY INSTRUCTION] 1.) Replay data log
		return !cltool_replayDataLog();
	}
	// if app firmware was specified on the command line, do that now and return
	else if (g_commandLineOptions.updateAppFirmwareFilename.length() != 0)
	{
        if (g_commandLineOptions.updateAppFirmwareFilename.substr(g_commandLineOptions.updateAppFirmwareFilename.length() - 4) != ".hex")
        {
            cout << "Incorrect file extension." << endl;
            return -1;
        }

		// [BOOTLOADER INSTRUCTION] 1.) Run bootloader
		return cltool_updateAppFirmware();
	}
    // if bootloader filename was specified on the command line, do that now and return
    else if (g_commandLineOptions.updateBootloaderFilename.length() != 0)
    {
        if (g_commandLineOptions.updateBootloaderFilename.substr(g_commandLineOptions.updateBootloaderFilename.length() - 4) != ".bin")
        {
            cout << "Incorrect file extension." << endl;
            return -1;
        }

        return cltool_updateBootloader();
    }
    // if host was specified on the command line, create a tcp server
	else if (g_commandLineOptions.host.length() != 0)
	{
		return cltool_createHost();
	}
	else if (g_commandLineOptions.asciiMessages.size() != 0)
	{
		serial_port_t serialForAscii;
		serialPortPlatformInit(&serialForAscii);
		serialPortOpen(&serialForAscii, g_commandLineOptions.comPort.c_str(), g_commandLineOptions.baudRate, 0);
		serialPortWriteAscii(&serialForAscii, "STPB", 4);
		serialPortWriteAscii(&serialForAscii, ("ASCB," + g_commandLineOptions.asciiMessages).c_str(), (int)(5 + g_commandLineOptions.asciiMessages.size()));
		unsigned char line[512];
		unsigned char* asciiData;
		while (!g_inertialSenseDisplay.ControlCWasPressed())
		{
			int count = serialPortReadAsciiTimeout(&serialForAscii, line, sizeof(line), 100, &asciiData);
			if (count > 0)
			{
				printf("%s", (char*)asciiData);
				printf("\r\n");
			}
		}
	}
	// open the device, start streaming data and logging if needed
	else
	{
		// [C++ COMM INSTRUCTION] STEP 1: Instantiate InertialSense Class  
		// Create InertialSense object, passing in data callback function pointer.
		InertialSense inertialSenseInterface(cltool_dataCallback);

		// [C++ COMM INSTRUCTION] STEP 2: Open serial port
		if (!inertialSenseInterface.Open(g_commandLineOptions.comPort.c_str(), g_commandLineOptions.baudRate, g_commandLineOptions.disableBroadcastsOnClose))
		{
			cout << "Failed to open serial port at " << g_commandLineOptions.comPort.c_str() << endl;
			return -1;	// Failed to open serial port
		}

		// [C++ COMM INSTRUCTION] STEP 3: Enable data broadcasting
		if (cltool_setupCommunications(inertialSenseInterface))
		{
			// [LOGGER INSTRUCTION] Setup and start data logger
			if (g_commandLineOptions.asciiMessages.size() == 0 && !cltool_setupLogger(inertialSenseInterface))
			{
				cout << "Failed to setup logger!" << endl;
				inertialSenseInterface.Close();
				return -1;
			}
			try
			{
				// Main loop. Could be in separate thread if desired.
				while (!g_inertialSenseDisplay.ControlCWasPressed())
				{
					// [C++ COMM INSTRUCTION] STEP 4: Read data
					if (!inertialSenseInterface.Update())
					{
						// device disconnected, exit
						break;
					}
				}
			}
			catch (...)
			{
				cout << "Unknown exception...";
			}
		}

		cout << "Shutting down..." << endl;

		// [C++ COMM INSTRUCTION] STEP 6: Close interface
		// Close cleanly to ensure serial port and logging are shutdown properly.  (optional)
		inertialSenseInterface.Close();
	}

	return 0;
}


int cltool_main(int argc, char* argv[])
{
	// Parse command line options
	if (!cltool_parseCommandLine(argc, argv))
	{
		g_inertialSenseDisplay.ShutDown();

		// parsing failed
		return -1;
	}

	// InertialSense class example using command line options
	int result = inertialSenseMain();
	if (result == -1)
	{
		cltool_outputHelp();

        // Pause so user can read error
        SLEEP_MS(2000);
	}

	g_inertialSenseDisplay.ShutDown();

	return result;
}
