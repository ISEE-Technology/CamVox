/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __INTERTIALSENSE_H
#define __INTERTIALSENSE_H

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "ISConstants.h"
#include "ISTcpClient.h"
#include "ISTcpServer.h"
#include "ISLogger.h"
#include "ISDisplay.h"
#include "ISUtilities.h"
#include "ISSerialPort.h"
#include "ISDataMappings.h"
#include "ISStream.h"

// use of InertialSense class requires winsock
#if PLATFORM_IS_WINDOWS

#pragma comment (lib, "Ws2_32.lib")

#endif

extern "C"
{
	// [C COMM INSTRUCTION]  Include data_sets.h and com_manager.h  
	#include "data_sets.h"
	#include "com_manager.h"

	#include "serialPortPlatform.h"
	#include "inertialSenseBootLoader.h"
}

#include <functional>

class InertialSense;

typedef std::function<void(InertialSense* i, p_data_t* data, int pHandle)> pfnHandleBinaryData;
typedef void(*pfnStepLogFunction)(InertialSense* i, const p_data_t* data, int pHandle);

using namespace std;

/**
* Inertial Sense C++ interface
* Note only one instance of this class per process is supported
*/
class InertialSense : public iISTcpServerDelegate
{
public:
	struct is_device_t
	{
		serial_port_t serialPort;
		dev_info_t devInfo;
		system_command_t sysCmd;
		nvm_flash_cfg_t flashConfig;
	};

	struct com_manager_cpp_state_t
	{
		// per device vars
		vector<is_device_t> devices;

		// common vars
		pfnHandleBinaryData binaryCallbackGlobal;
		pfnHandleBinaryData binaryCallback[DID_COUNT];
		pfnStepLogFunction stepLogFunction;
		InertialSense* inertialSenseInterface;
		char* clientBuffer;
		int clientBufferSize;
		int* clientBytesToSend;
	};

	typedef struct
	{
		string port;
		string error;
	} bootloader_result_t;

	/**
	* Constructor
	* @param callback binary data callback, optional. If specified, ALL BroadcastBinaryData requests will callback to this function.
	*/
	InertialSense(pfnHandleBinaryData callback = NULL);

	/**
	* Destructor
	*/
	virtual ~InertialSense();

	/*
	* Broadcast binary data
	* @param dataId the data id (DID_* - see data_sets.h) to broadcast
	* @param periodMultiple a scalar that the source period is multiplied by to give the output period in milliseconds, 0 for one time message, less than 0 to disable broadcast of the specified dataId
	* @param callback optional callback for this dataId
	* @return true if success, false if error - if callback is NULL and no global callback was passed to the constructor, this will return false
	*/ 
	bool BroadcastBinaryData(uint32_t dataId, int periodMultiple, pfnHandleBinaryData callback = NULL);

	/**
	* Close the connection, logger and free all resources
	*/
	void Close();

	/**
	* Get all open serial port names
	*/
	vector<string> GetPorts();

	/**
	* Check if the connection is open
	*/
	bool IsOpen();

	/**
	* Closes any open connection and then opens the device
	* @param port the port to open
	* @param baudRate the baud rate to connect with - supported rates are 115200, 230400, 460800, 921600, 2000000, 3000000
	* @param disableBroadcastsOnClose whether to send a stop broadcasts command to all units on Close
	* @return true if opened, false if failure (i.e. baud rate is bad or port fails to open)
	*/
	bool Open(const char* port, int baudRate = IS_COM_BAUDRATE_DEFAULT, bool disableBroadcastsOnClose = false);

	/**
	* Get the number of open devices
	* @return the number of open devices
	*/
	size_t GetDeviceCount();

	/**
	* Call in a loop to send and receive data.  Call at regular intervals as frequently as want to receive data.
	* @return true if updating should continue, false if the process should be shutdown
	*/
	bool Update();

	/**
	* Enable or disable logging - logging is disabled by default
	* @param enable enable or disable the logger - disabling the logger after enabling it will close it and flush all data to disk
	* @param path the path to write the log files to
	* @param logType the type of log to write
	* @param logSolution true to log solution stream, false otherwise
	* @param maxDiskSpacePercent the max disk space to use in percent of free space (0.0 to 1.0)
	* @param maxFileSize the max file size for each log file in bytes
	* @param chunkSize the max data to keep in RAM before flushing to disk in bytes
	* @param subFolder timestamp sub folder or empty for none
	* @return true if success, false if failure
	*/
	bool SetLoggerEnabled(
        bool enable, 
        const string& path = cISLogger::g_emptyString, 
        cISLogger::eLogType logType = cISLogger::eLogType::LOGTYPE_DAT, 
        uint64_t rmcPreset = RMC_PRESET_PPD_BITS, 
        float maxDiskSpacePercent = 0.5f, 
        uint32_t maxFileSize = 1024 * 1024 * 5, 
        const string& subFolder = cISLogger::g_emptyString);

	/**
	* Enable streaming of predefined set of messages.  The default preset, RMC_PRESET_INS_BITS, stream data necessary for post processing.
	* @param rmcPreset realtimeMessageController preset
	*/
	void BroadcastBinaryDataRmcPreset(uint64_t rmcPreset=RMC_PRESET_INS_BITS, uint32_t rmcOptions=0);

	/**
	* Gets whether logging is enabled
	* @return whether logging is enabled
	*/
	bool LoggerEnabled() { return m_logger.Enabled(); }

	/**
	* Connect to a server and send the data from that server to the uINS. Open must be called first to connect to the uINS unit.
	* @param connectionString the server to connect, this is the data type (RTCM3,IS,UBLOX) followed by a colon followed by connection info (ip:port or serial:baud). This can also be followed by an optional url, user and password, i.e. RTCM3:192.168.1.100:7777:RTCM3_Mount:user:password
	* @return true if connection opened, false if failure
	*/
	bool OpenServerConnection(const string& connectionString);

	/**
	* Create a host that will stream data from the uINS to connected clients. Open must be called first to connect to the uINS unit.
	* @param ipAndPort ip address followed by colon followed by port. Ip address is optional and can be blank to auto-detect.
	* @return true if success, false if error
	*/
	bool CreateHost(const string& ipAndPort);

	/**
	* Close any open connection to a server
	*/
	void CloseServerConnection();

	/**
	* Turn off all messages.  Current port only if allPorts = false.
	*/
	void StopBroadcasts(bool allPorts=true);

	/**
	* Send data to the uINS - this is usually only used for advanced or special cases, normally you won't use this method
	* @param dataId the data id of the data to send
	* @param data the data to send
	* @param length length of data to send
	* @param offset offset into data to send at
	*/
	void SendData(eDataIDs dataId, uint8_t* data, uint32_t length, uint32_t offset);

	/**
	* Send raw data to the uINS - this is usually only used for advanced or special cases, normally you won't use this method
	* @param dataId the data id of the data to send
	* @param data the data to send
	* @param length length of data to send
	* @param offset offset into data to send at
	*/
	void SendRawData(eDataIDs dataId, uint8_t* data, uint32_t length = 0, uint32_t offset = 0);

	/**
	* Get the device info
	* @param pHandle the pHandle to get device info for
	* @return the device info
	*/
	const dev_info_t GetDeviceInfo(int pHandle = 0)
	{
		if ((size_t)pHandle >= m_comManagerState.devices.size())
		{
			pHandle = 0;
		}
		return m_comManagerState.devices[pHandle].devInfo;
	}

	/**
	* Get current device system command
	* @param pHandle the pHandle to get sysCmd for
	* @return current device system command
	*/
	system_command_t GetSysCmd(int pHandle = 0) 
	{ 
		if ((size_t)pHandle >= m_comManagerState.devices.size())
		{
			pHandle = 0;
		}
		return m_comManagerState.devices[pHandle].sysCmd;
	}

	/**
	* Set device configuration
	* @param pHandle the pHandle to set sysCmd for
	* @param sysCmd new device configuration
	*/
	void SetSysCmd(const system_command_t& sysCmd, int pHandle = 0);

	/**
	* Get the flash config, returns the latest flash config read from the uINS flash memory
	* @param pHandle the pHandle to get flash config for
	* @return the flash config
	*/
	nvm_flash_cfg_t GetFlashConfig(int pHandle = 0) 
	{
		if ((size_t)pHandle >= m_comManagerState.devices.size())
		{
			pHandle = 0;
		}
		return m_comManagerState.devices[pHandle].flashConfig;
	}

	/**
	* Set the flash config and update flash config on the uINS flash memory
	* @param flashConfig the flash config
	* @param pHandle the pHandle to set flash config for
	*/
	void SetFlashConfig(const nvm_flash_cfg_t& flashConfig, int pHandle = 0);

	/**
	* Get the number of bytes read or written to/from client or server connections
	* @return byte count
	*/
	uint64_t GetClientServerByteCount() { return m_clientServerByteCount; }

	/**
	* Get access to the underlying serial port
	* @param pHandle the pHandle to get the serial port for
	* @return the serial port
	*/
	serial_port_t* GetSerialPort(int pHandle = 0) 
	{
		if ((size_t)pHandle >= m_comManagerState.devices.size())
		{
			return NULL;
		}
		return &(m_comManagerState.devices[pHandle].serialPort);
	}

	/**
	* Get the timeout flush logger parameter in seconds
	* @return the timeout flush logger parameter in seconds
	*/
	time_t GetTimeoutFlushLoggerSeconds() { return m_logger.GetTimeoutFlushSeconds(); }

	/**
	* Set the timeout flush logger parameter in seconds
	* @param timeoutFlushLoggerSeconds the timeout flush logger parameter in seconds
	*/
	void SetTimeoutFlushLoggerSeconds(time_t timeoutFlushLoggerSeconds) { m_logger.SetTimeoutFlushSeconds(timeoutFlushLoggerSeconds); }

	/**
	* Bootload a file - if the bootloader fails, the device stays in bootloader mode and you must call BootloadFile again until it succeeds. If the bootloader gets stuck or has any issues, power cycle the device.
	* Please ensure that all other connections to the com port are closed before calling this function.
	* @param the com port to bootload, can be comma separated for multiple
	* @param fileName the path of the file to bootload
	* @param baudRate the baud rate to bootload at
	* @param uploadProgress optional callback for upload progress
	* @param verifyProgress optional callback for verify progress
	* @return ports and errors, error will be empty if success
	*/
	static vector<bootloader_result_t> BootloadFile(const string& comPort, const string& fileName, const string& bootloaderFileName, int baudRate = IS_BAUD_RATE_BOOTLOADER, pfnBootloadProgress uploadProgress = NULLPTR, pfnBootloadProgress verifyProgress = NULLPTR, pfnBootloadStatus infoProgress = NULLPTR, bool updateBootloader = false);
	static vector<bootloader_result_t> BootloadFile(const string& comPort, const string& fileName, int baudRate = IS_BAUD_RATE_BOOTLOADER, pfnBootloadProgress uploadProgress = NULLPTR, pfnBootloadProgress verifyProgress = NULLPTR, bool updateBootloader = false);

protected:
	bool OnPacketReceived(const uint8_t* data, uint32_t dataLength);
	void OnClientConnecting(cISTcpServer* server) OVERRIDE;
	void OnClientConnected(cISTcpServer* server, socket_t socket) OVERRIDE;
	void OnClientConnectFailed(cISTcpServer* server) OVERRIDE;
	void OnClientDisconnected(cISTcpServer* server, socket_t socket) OVERRIDE;

private:
	InertialSense::com_manager_cpp_state_t m_comManagerState;
	cISLogger m_logger;
	void* m_logThread;
	cMutex m_logMutex;
	map<int, vector<p_data_t>> m_logPackets;
	time_t m_lastLogReInit;
	cISTcpClient m_tcpClient;
	char m_clientBuffer[512];
	int m_clientBufferBytesToSend;
	cISTcpServer m_tcpServer;
	cISSerialPort m_serialServer;
	cISStream* m_clientStream;
	uint64_t m_clientServerByteCount;
	bool m_disableBroadcastsOnClose;
	com_manager_init_t m_cmInit;
	com_manager_port_t *m_cmPorts;
	is_comm_instance_t m_gpComm;
	uint8_t m_gpCommBuffer[PKT_BUF_SIZE];

	// returns false if logger failed to open
	bool EnableLogging(const string& path, cISLogger::eLogType logType, float maxDiskSpacePercent, uint32_t maxFileSize, const string& subFolder);
	void DisableLogging();
	bool HasReceivedResponseFromDevice(size_t index);
	bool HasReceivedResponseFromAllDevices();
	void RemoveDevice(size_t index);
	bool OpenSerialPorts(const char* port, int baudRate);
	static void LoggerThread(void* info);
	static void StepLogger(InertialSense* i, const p_data_t* data, int pHandle);
};

#endif
