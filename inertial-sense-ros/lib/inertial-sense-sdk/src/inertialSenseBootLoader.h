/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __INERTIALSENSEBOOTLOADER_H
#define __INERTIALSENSEBOOTLOADER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "serialPort.h"

/** uINS bootloader baud rate */
#define IS_BAUD_RATE_BOOTLOADER 921600

/** uINS rs232 bootloader baud rate */
#define IS_BAUD_RATE_BOOTLOADER_RS232 230400

/** uINS slow bootloader baud rate */
#define IS_BAUD_RATE_BOOTLOADER_SLOW 115200

/** uINS bootloader baud rate - legacy */
#define IS_BAUD_RATE_BOOTLOADER_LEGACY 2000000

#define ENABLE_BOOTLOADER_BAUD_DETECTION 1
#define BOOTLOADER_REFRESH_DELAY   500
#define BOOTLOADER_RESPONSE_DELAY  10
#if ENABLE_BOOTLOADER_BAUD_DETECTION
#define BOOTLOADER_RETRIES         30
#else
#define BOOTLOADER_RETRIES         1
#endif

#ifndef BOOTLOADER_ERROR_LENGTH
#define BOOTLOADER_ERROR_LENGTH	512		// Set to zero to disable
#endif

/** Bootloader callback function prototype, return 1 to stay running, return 0 to cancel */
typedef int(*pfnBootloadProgress)(const void* obj, float percent);

/** Bootloader information string function prototype. */
typedef void(*pfnBootloadStatus)(const void* obj, const char* infoString);

typedef struct
{
	const char* fileName; // read from this file
    const char* bootName; // optional bootloader file
    int forceBootloaderUpdate;
	serial_port_t* port; // connect with this serial port
	char error[BOOTLOADER_ERROR_LENGTH];
	const void* obj; // user defined pointer
	pfnBootloadProgress uploadProgress; // upload progress
	pfnBootloadProgress verifyProgress; // verify progress
    pfnBootloadStatus statusText;       // receives status text for progress
	const char* verifyFileName; // optional, writes verify file to the path if not 0
	int numberOfDevices; // number of devices if bootloading in parallel
	int baudRate; // baud rate to connect to
    union
    {
        unsigned int bits;
        struct
        {
            unsigned int enableVerify : 1; // whether to enable the verify phase
        } bitFields;
    } flags;
    char bootloadEnableCmd[16];

} bootload_params_t;

/**
Boot load a .hex or .bin file to a device

@param port the serial port to bootload to, will be opened and closed, must have port set
@param error a buffer to store any error messages in - can be NULL
@param errorLength the number of bytes available in error
@param obj custom user object that will be passed in the callback
@param uploadProgress called periodically during firmware upload - can be NULL
@param verifyProgress called periodically during firmware verification - can be NULL

@return 0 if failure, non-zero if success
*/
int bootloadFile(serial_port_t* port, const char* fileName, const char* bootName,
    const void* obj, pfnBootloadProgress uploadProgress, pfnBootloadProgress verifyProgress);
int bootloadFileEx(bootload_params_t* params);

/**
Boot load a new bootloader .bin file to device. Device must be in application or bootloader assist mode, not bootloader mode.
@param port the serial port, must be initialized and have the port set
@param fileName the new bootloader .bin file
@param error a buffer to store any error messages in - can be NULL
@param errorLength the number of bytes available in error
@param obj custom user object that will be passed in the callback
@param uploadProgress called periodically during firmware upload - can be NULL
@param verifyProgress called periodically during firmware verification - can be NULL
@retur 0 if failure, non-zero if success
*/
int bootloadUpdateBootloader(serial_port_t* port, const char* fileName, 
    const void* obj, pfnBootloadProgress uploadProgress, pfnBootloadProgress verifyProgress);
int bootloadUpdateBootloaderEx(bootload_params_t* p);

/**
Retrieve the bootloader version from .bin file

@param fileName for the new bootloader .bin file
@param pointer to int to store major version
@param pointer to char to store minor version

@return 0 if failure, 1 if success
*/
int bootloadGetBootloaderVersionFromFile(const char* bootName, int* verMajor, char* verMinor);

/**
Enable bootloader mode for a device

@param port the port to enable the bootloader on
@param baudRate the baud rate to communicate with, IS_BAUD_RATE_BOOTLOADER or IS_BAUD_RATE_BOOTLOADER_RS232
@param error a buffer to store any error messages - can be NULL
@param errorLength the number of bytes available in error

@return 0 if failure, non-zero if success
*/
int enableBootloader(serial_port_t* port, int baudRate, char* error, int errorLength, const char* bootloadEnableCmd);

/**
Disables the bootloader and goes back to program mode

@port the port to go back to program mode on
@error a buffer to store any error messages - can be NULL
@errorLength the number of bytes available in error

@return 0 if failure, non-zero if success
*/
int disableBootloader(serial_port_t* port, char* error, int errorLength);


int bootloaderCycleBaudRate(int baudRate);
int bootloaderClosestBaudRate(int baudRate);


#ifdef __cplusplus
}
#endif

#endif	// __INERTIALSENSEBOOTLOADER_H
