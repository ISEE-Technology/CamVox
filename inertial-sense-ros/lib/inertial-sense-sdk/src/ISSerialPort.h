/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __ISSERIALPORT_H__
#define __ISSERIALPORT_H__

#include <inttypes.h>
#include <string>
#include <vector>

#include "ISStream.h"
#include "serialPortPlatform.h"

using namespace std;

class cISSerialPort : public cISStream
{
private:
	cISSerialPort(const cISSerialPort& copy); // disable copy constructor

	serial_port_t m_serial;
	int m_timeout;
	bool m_blocking;

public:
	/**
	* Constructor
	* @param serial inner serial port implementation or NULL for default, if not NULL, it will be copied
	*/
	cISSerialPort(serial_port_t* serial = NULL);

	/**
	* Destructor - closes the serial port
	*/
	virtual ~cISSerialPort();

	/**
	* Open the serial port
	* @param portName the port name to open
	* @param baudRate the baud rate to open at
	* @param timeout read timeout, 0 for none
	* @param blocking whether the serial port blocks until data is read or written
	* @return true if success, false if failure
	*/
	bool Open(const std::string& portName, int baudRate = BAUDRATE_921600, int timeout = 0, bool blocking = false);

	/**
	* Checks if the serial port is open
	* @return true if open, false otherwise
	*/
	bool IsOpen() { return (serialPortIsOpen(&m_serial) != 0); }

	/**
	* Close the serial port
	* @return 0 if success, otherwise an error code
	*/
	int Close() OVERRIDE;

	/**
	* Read data from the serial port
	* @param data the buffer to read data into
	* @param dataLength the number of bytes available in data
	* @return the number of bytes read or less than 0 if error
	*/
	int Read(void* data, int dataLength) OVERRIDE;

	/**
	* Write data to the serial port
	* @param data the data to write
	* @param dataLength the number of bytes to write
	* @return the number of bytes written or less than 0 if error
	*/
	int Write(const void* data, int dataLength) OVERRIDE;

	/**
	* Gets a list of com names of all connected usb ports
	* @param ports cleared and then receives the name of each connected usb port
	*/
	static void GetComPorts(vector<string>& ports);
};

#endif // __ISSERIALPORT_H__
