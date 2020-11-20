/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>

// STEP 1: Add Includes
// Change these include paths to the correct paths for your project
#include "../../src/serialPortPlatform.h"
#include "../../src/ISComm.h"

static int running = 1;

int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		printf("Please pass the com port as the only argument\r\n");
		// In Visual Studio IDE, this can be done through "Project Properties -> Debugging -> Command Arguments: COM3" 
		return -1;
	}

	// STEP 2: Initialize and open serial port
	serial_port_t serialPort;

	// Initialize the serial port (Windows, MAC or Linux) - if using an embedded system like Arduino,
	//  you will need to handle the serial port creation, open and reads yourself. In this
	//  case, you do not need to include serialPort.h/.c and serialPortPlatform.h/.c in your project.
	serialPortPlatformInit(&serialPort);

	// Open serial, last parameter is a 1 which means a blocking read, you can set as 0 for non-blocking
	// you can change the baudrate to a supported baud rate (IS_BAUDRATE_*), make sure to reboot the uINS
	//  if you are changing baud rates, you only need to do this when you are changing baud rates.
	if (!serialPortOpen(&serialPort, argv[1], IS_BAUDRATE_921600, 1))
	{
		printf("Failed to open serial port on com port %s\r\n", argv[1]);
		return -2;
	}


	// STEP 3: Enable prior message broadcasting
	// Stop all broadcasts on the device on all ports.  We don't want binary message coming through while we are doing ASCII
	if (!serialPortWriteAscii(&serialPort, "STPB", 4))
	{
		printf("Failed to encode stop broadcasts message\r\n");
		return -3;
	}
#if 0
    // Query device version information
	if (!serialPortWriteAscii(&serialPort, "INFO", 4))
	{
		printf("Failed to encode stop broadcasts message\r\n");
		return -3;
	}
#endif
    // STEP 4: Enable message broadcasting

	// ASCII protocol is based on NMEA protocol https://en.wikipedia.org/wiki/NMEA_0183
	// turn on the INS message at a period of 100 milliseconds (10 hz)
	// serialPortWriteAscii takes care of the leading $ character, checksum and ending \r\n newline
	// ASCB message enables ASCII broadcasts
	// ASCB fields: 1:options, 2:PIMU, 3:PPIMU, 4:PINS1, 5:PINS2, 6:PGPSP, 7:reserved, 8:GPGGA, 9:GPGLL, 10:GPGSA, 11:GPRMC
	// options can be 0 for current serial port, 1 for serial 0, 2 for serial 1 or 3 for both serial ports
	// Instead of a 0 for a message, it can be left blank (,,) to not modify the period for that message
	// please see the user manual for additional updates and notes

    // Get PINS1 @ 10Hz on the connected serial port, leave all other broadcasts the same, and save persistent messages.
	const char* asciiMessage = "ASCB,512,,,1000,,,,,,,";

    // Get PINS1 @ 50Hz and PGPSP @ 5Hz on the connected serial port, leave all other broadcasts the same
//     const char* asciiMessage = "ASCB,,,,20,,200,,,,,";

	// Get PIMU @ 50Hz, GPGGA @ 5Hz, both serial ports, set all other periods to 0
//     const char* asciiMessage = "ASCB,3,20,0,0,0,0,0,100,0,0,0";

    if (!serialPortWriteAscii(&serialPort, asciiMessage, (int)strnlen(asciiMessage, 128)))
	{
		printf("Failed to encode ASCII get INS message\r\n");
		return -4;
	}


#if 0
    // STEP 5: (optional) Save Persistent Messages.  This remembers the current communications and automatically streams data following reboot.
    if (!serialPortWriteAscii(&serialPort, "PERS", 4))
    {
        printf("Failed to encode ASCII save persistent message\r\n");
        return -4;
    }
#endif


	// STEP 6: Handle received data
	unsigned char* asciiData;
	unsigned char asciiLine[512];

	// You can set running to false with some other piece of code to break out of the loop and end the program
	while (running)
	{
		if (serialPortReadAscii(&serialPort, asciiLine, sizeof(asciiLine), &asciiData) > 0)
		{
			printf("%s\n", asciiData);
		}
	}
}

