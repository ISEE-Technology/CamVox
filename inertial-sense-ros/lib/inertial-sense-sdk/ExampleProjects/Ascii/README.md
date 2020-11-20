# SDK: ASCII Communications Example Project

This [ISAsciiExample](https://github.com/inertialsense/InertialSenseSDK/tree/master/ExampleProjects/Ascii) project demonstrates ASCII NMEA communications with the <a href="https://inertialsense.com">InertialSense</a> products (uINS, uAHRS, and uIMU) using the Inertial Sense SDK.  See the [ASCII protocol](../protocol_ascii) section for details on the ASCII packet structures. 

## Files

#### Project Files

* [ISAsciiExample.c](https://github.com/inertialsense/InertialSenseSDK/tree/master/ExampleProjects/Ascii/ISAsciiExample.c)

#### SDK Files

* [data_sets.c](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/data_sets.c)
* [data_sets.h](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/data_sets.h)
* [ISComm.c](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/ISComm.c)
* [ISComm.h](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/ISComm.h)
* [ISConstants.h](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/ISConstants.h)
* [serialPort.c](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/serialPort.c)
* [serialPort.h](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/serialPort.h)
* [serialPortPlatform.c](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/serialPortPlatform.c)
* [serialPortPlatform.h](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/serialPortPlatform.h)

## Implementation

### Step 1: Add Includes

```C++
// Change these include paths to the correct paths for your project
#include "../../src/ISComm.h"
#include "../../src/serialPortPlatform.h"
```

### Step 2: Initialize and open serial port

```C++
	serial_port_t serialPort;

	// Initialize the serial port (Windows, MAC or Linux) - if using an embedded system like Arduino,
	//  you will need to handle the serial port creation, open and reads yourself. In this
	//  case, you do not need to include serialPort.h/.c and serialPortPlatform.h/.c in your project.
	serialPortPlatformInit(&serialPort);

	// Open serial, last parameter is a 1 which means a blocking read, you can set as 0 for non-blocking
	// you can change the baudrate to a supported baud rate (IS_BAUDRATE_*), make sure to reboot the uINS
	//  if you are changing baud rates, you only need to do this when you are changing baud rates.
	if (!serialPortOpen(&serialPort, argv[1], IS_BAUDRATE_3000000, 1))
	{
		printf("Failed to open serial port on com port %s\r\n", argv[1]);
		return -2;
	}
```

### Step 3: Enable message broadcasting

```C++
	// stop all broadcasts on the device, we don't want binary message coming through while we are doing ASCII
	if (!serialPortWriteAscii(&serialPort, "STPB", 4))
	{
		printf("Failed to encode stop broadcasts message\r\n");
		return -3;
	}

	// ASCII protocol is based on NMEA protocol https://en.wikipedia.org/wiki/NMEA_0183
	// turn on the INS message at a period of 100 milliseconds (10 hz)
	// serialPortWriteAscii takes care of the leading $ character, checksum and ending \r\n newline
	// ASCB message enables ASCII broadcasts
	// ASCB fields: 1:options, 2:PIMU, 3:PPIMU, 4:PINS1, 5:PINS2, 6:PGPSP, 7:reserved, 8:GPGGA, 9:GPGLL, 10:GPGSA, 11:GPRMC
	// options can be 0 for current serial port, 1 for serial 0, 2 for serial 1 or 3 for both serial ports
	// Instead of a 0 for a message, it can be left blank (,,) to not modify the period for that message
	// please see the user manual for additional updates and notes

	// Get PINS1 @ 10Hz on the connected serial port, leave all other broadcasts the same
	const char* asciiMessage = "ASCB,0,,,100,,,,,,,";

	// Get PIMU @ 50Hz, GPGGA @ 5Hz, both serial ports, set all other periods to 0
	// const char* asciiMessage = "ASCB,3,20,0,0,0,0,0,100,0,0,0";

	// Stop all messages / broadcasts
	// const char* asciiMessage = "STPB";
																				
	if (!serialPortWriteAscii(&serialPort, asciiMessage, (int)strnlen(asciiMessage, 128)))
	{
		printf("Failed to encode ASCII get INS message\r\n");
		return -4;
	}
```


### Step 4: Handle recieved data 

```C++
	// STEP 4: Handle received data
	unsigned char* asciiData;
	unsigned char asciiLine[512];

	// you can set running to false with some other piece of code to break out of the loop and end the program
	while (running)
	{
		if (serialPortReadAscii(&serialPort, asciiLine, sizeof(asciiLine), &asciiData) > 0)
		{
			printf("%s\n", asciiData);
		}
	}
```

## Compile & Run (Linux/Mac)

1. Create build directory
``` bash
$ cd InertialSenseSDK/ExampleProjects/Ascii
$ mkdir build
```
2. Run cmake from within build directory
``` bash
$ cd build
$ cmake ..
```
3. Compile using make
 ``` bash
 $ make
 ```
4. If necessary, add current user to the "dialout" group in order to read and write to the USB serial communication ports:
```bash
$ sudo usermod -a -G dialout $USER
$ sudo usermod -a -G plugdev $USER
(reboot computer)
```
5. Run executable
``` bash
$ ./bin/ISAsciiExample /dev/ttyUSB0
```
## Compile & Run (Windows MS Visual Studio)

1. Open Visual Studio solution file (InertialSenseSDK\ExampleProjects\Ascii\VS_project\ISAsciiExample.sln)
2. Build (F7)
3. Run executable
``` bash
C:\InertialSenseSDK\ExampleProjects\Ascii\VS_project\Release\ISAsciiExample.exe COM3
```

## Summary

That covers all the basic functionality you need to set up and talk to <a href="https://inertialsense.com">InertialSense</a> products.  If this doesn't cover everything you need, feel free to reach out to us on the <a href="https://github.com/inertialsense/InertialSenseSDK">InertialSenseSDK</a> GitHub repository, and we will be happy to help.
