/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_SERIALPORT_H
#define __IS_SERIALPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern int SERIAL_PORT_DEFAULT_TIMEOUT;

#define MAX_SERIAL_PORT_NAME_LENGTH 63

// Standard Baud Rates - FTDI Functional.	// Bit period = 1/baudrate, Actual baud (FTDI,AVR,ARM)
#define BAUDRATE_300			300			// 3333 us
#define BAUDRATE_600			600			// 1667 us
#define BAUDRATE_1200			1200		//  833 us
#define BAUDRATE_2400			2400		//  417 us
#define BAUDRATE_4800			4800		//  208 us
#define BAUDRATE_9600			9600		//  104 us
#define BAUDRATE_19200			19200		//   52 us
#define BAUDRATE_38400			38400		//   26 us
#define BAUDRATE_57600			57600		//   17 us
#define BAUDRATE_115200			115200		// 8680 ns
#define BAUDRATE_230400			230400		// 4340 ns
#define BAUDRATE_460800			460800		// 2170 ns
#define BAUDRATE_921600			921600		// 1085 ns
#define BAUDRATE_1000000		1000000		// 1000 ns
#define BAUDRATE_1220000		1220000		//  820 ns
#define BAUDRATE_1440000		1440000		//  794 ns
#define BAUDRATE_1500000		1500000		//  667 ns	(FTDI 1520, AFR 1500)
#define BAUDRATE_2000000		2000000		//  500 ns	(FTDI 2080, AVR/ARM 2016)
#define BAUDRATE_3000000		3000000		//  333 ns	(FTDI 3150, AVR/ARM 3030)

typedef struct serial_port_t serial_port_t;

typedef int(*pfnSerialPortOpen)(serial_port_t* serialPort, const char* port, int baudRate, int blocking);
typedef int(*pfnSerialPortIsOpen)(serial_port_t* serialPort);
typedef int(*pfnSerialPortRead)(serial_port_t* serialPort, unsigned char* buf, int len, int timeoutMilliseconds);
typedef void(*pfnSerialPortAsyncReadCompletion)(serial_port_t* serialPort, unsigned char* buf, int len, int errorCode);
typedef int(*pfnSerialPortAsyncRead)(serial_port_t* serialPort, unsigned char* buf, int len, pfnSerialPortAsyncReadCompletion completion);
typedef int(*pfnSerialPortWrite)(serial_port_t* serialPort, const unsigned char* buf, int len);
typedef int(*pfnSerialPortClose)(serial_port_t* serialPort);
typedef int(*pfnSerialPortFlush)(serial_port_t* serialPort);
typedef int(*pfnSerialPortGetByteCountAvailableToRead)(serial_port_t* serialPort);
typedef int(*pfnSerialPortGetByteCountAvailableToWrite)(serial_port_t* serialPort);
typedef int(*pfnSerialPortSleep)(int sleepMilliseconds);

// Allows communicating over a serial port
struct serial_port_t
{
	// platform specific handle
	void* handle;

	// the port name (do not modify directly)
	char port[MAX_SERIAL_PORT_NAME_LENGTH + 1];

	// optional error buffer to store errors
	char* error;

	// length of error
	int errorLength;

	// open the serial port
	pfnSerialPortOpen pfnOpen;

	// is the serial port open?
	pfnSerialPortIsOpen pfnIsOpen;

	// read data synchronously
	pfnSerialPortRead pfnRead;

	// read data asynchronously
	pfnSerialPortAsyncRead pfnAsyncRead;

	// write data synchronously
	pfnSerialPortWrite pfnWrite;

	// close the serial port
	pfnSerialPortClose pfnClose;

	// remove all data from all buffers
	pfnSerialPortFlush pfnFlush;

	// get number of bytes in the receive buffer that can be read
	pfnSerialPortGetByteCountAvailableToRead pfnGetByteCountAvailableToRead;

	// get the number of available bytes in the send buffer
	pfnSerialPortGetByteCountAvailableToWrite pfnGetByteCountAvailableToWrite;

	// sleep for a specified number of milliseconds
	pfnSerialPortSleep pfnSleep;
};

// set the port name for a serial port, in case you are opening it later
void serialPortSetPort(serial_port_t* serialPort, const char* port);

// open a serial port
// port is null terminated, i.e. COM1\0, COM2\0, etc.
// use blocking = 0 when data is being streamed from the serial port rapidly and blocking = 1 for
// uses such as a boot loader where a write would then require n bytes to be read in a single operation.
// blocking simply determines the default timeout value of the serialPortRead function
// returns 1 if success, 0 if failure
int serialPortOpen(serial_port_t* serialPort, const char* port, int baudRate, int blocking);

// open a serial port with retry
// port is null terminated, i.e. COM1\0, COM2\0, etc.
// use blocking = 0 when data is being streamed from the serial port rapidly and blocking = 1 for
// uses such as a boot loader where a write would then require n bytes to be read in a single operation.
// blocking simply determines the default timeout value of the serialPortRead function
// returns 1 if success, 0 if failure
int serialPortOpenRetry(serial_port_t* serialPort, const char* port, int baudRate, int blocking);

// check if the port is open
// returns 1 if open, 0 if not open
int serialPortIsOpen(serial_port_t* serialPort);

// close the serial port - this object can be re-used by calling open again, returns 1 if closed and returns 0 if the port was not closed
int serialPortClose(serial_port_t* serialPort);

// clear all buffers and pending reads and writes - returns 1 if success, 0 if failure
int serialPortFlush(serial_port_t* serialPort);

// read up to readCount bytes into buffer
// call is forwarded to serialPortReadTimeout with timeoutMilliseconds of 0 for non-blocking, or SERIAL_PORT_DEFAULT_TIMEOUT for blocking
int serialPortRead(serial_port_t* serialPort, unsigned char* buffer, int readCount);

// read up to thue number of bytes requested, returns number of bytes read which is less than or equal to readCount
int serialPortReadTimeout(serial_port_t* serialPort, unsigned char* buffer, int readCount, int timeoutMilliseconds);

// start an async read - not all platforms will support an async read and may call the callback function immediately
// reads up to readCount bytes into buffer
// buffer must exist until callback is executed, if it needs to be freed, free it in the callback or later
// returns 1 if success, 0 if failed to start async operation
int serialPortReadTimeoutAsync(serial_port_t* serialPort, unsigned char* buffer, int readCount, pfnSerialPortAsyncReadCompletion callback);

// read up until a \r\n sequence has been read
// buffer will not contain \r\n sequence
// returns number of bytes read or -1 if timeout or buffer overflow, count does not include the null terminator
int serialPortReadLine(serial_port_t* serialPort, unsigned char* buffer, int bufferLength);

// read up until a \r\n sequence has been read
// result will not contain \r\n sequence
// returns number of bytes read or -1 if timeout or buffer overflow, count does not include the null terminator
int serialPortReadLineTimeout(serial_port_t* serialPort, unsigned char* buffer, int bufferLength, int timeoutMilliseconds);

// read ASCII data (starts with $ and ends with \r\n, based on NMEA format)
// will ignore data that fails checksum
// asciiData gets set to start of ASCII data
// return -1 if timeout or buffer overflow or checksum failure
int serialPortReadAscii(serial_port_t* serialPort, unsigned char* buffer, int bufferLength, unsigned char** asciiData);

// read ASCII data (starts with $ and ends with \r\n, based on NMEA format)
// will ignore data that fails checksum
// asciiData gets set to start of ASCII data
// return -1 if timeout or buffer overflow or checksum failure
int serialPortReadAsciiTimeout(serial_port_t* serialPort, unsigned char* buffer, int bufferLength, int timeoutMilliseconds, unsigned char** asciiData);

// read one char, waiting SERIAL_PORT_DEFAULT_TIMEOUT milliseconds to get a char
int serialPortReadChar(serial_port_t* serialPort, unsigned char* c);

// read one char, waiting timeoutMilliseconds to get a char, returns number of chars read
int serialPortReadCharTimeout(serial_port_t* serialPort, unsigned char* c, int timeoutMilliseconds);

// write, returns the number of bytes written
int serialPortWrite(serial_port_t* serialPort, const unsigned char* buffer, int writeCount);

// write with a \r\n added at the end, \r\n should not be part of buffer, returns the number of bytes written
int serialPortWriteLine(serial_port_t* serialPort, const unsigned char* buffer, int writeCount);

// write ascii data - if buffer does not start with $, a $ will be written first, followed by buffer, followed by *xx\r\n, where xx is a two hex character checksum
int serialPortWriteAscii(serial_port_t* serialPort, const char* buffer, int bufferLength);

// write and wait for a response, returns 1 if success, 0 if failure
int serialPortWriteAndWaitFor(serial_port_t* serialPort, const unsigned char* buffer, int writeCount, const unsigned char* waitFor, int waitForLength);
int serialPortWriteAndWaitForTimeout(serial_port_t* serialPort, const unsigned char* buffer, int writeCount, const unsigned char* waitFor, int waitForLength, const int timeoutMilliseconds);

// wait for a response, returns 0 if failure, 1 if success
int serialPortWaitFor(serial_port_t* serialPort, const unsigned char* waitFor, int waitForLength);
int serialPortWaitForTimeout(serial_port_t* serialPort, const unsigned char* waitFor, int waitForLength, int timeoutMilliseconds);

// get available bytes in the receive buffer
int serialPortGetByteCountAvailableToRead(serial_port_t* serialPort);

// get available bytes in the send buffer
int serialPortGetByteCountAvailableToWrite(serial_port_t* serialPort);

// sleep for the specified number of milliseconds if supported, returns 1 if success, 0 if failed to sleep
int serialPortSleep(serial_port_t* serialPort, int sleepMilliseconds);

#ifdef __cplusplus
}
#endif

#endif // __IS_SERIALPORT_H
