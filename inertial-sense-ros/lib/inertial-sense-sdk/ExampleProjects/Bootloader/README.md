# SDK: Firmware Update (Bootloader) Example Project

This [ISBootloaderExample](https://github.com/inertialsense/InertialSenseSDK/tree/master/ExampleProjects/Bootloader) project demonstrates firmware update with the <a href="https://inertialsense.com">InertialSense</a> products (uINS, uAHRS, and uIMU) using the Inertial Sense SDK.

## Files

#### Project Files

* [ISBootloaderExample.c](https://github.com/inertialsense/InertialSenseSDK/tree/master/ExampleProjects/Bootloader/ISBootloaderExample.c)

#### SDK Files

* [data_sets.c](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/data_sets.c)
* [data_sets.h](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/data_sets.h)
* [inertialSenseBootLoader.c](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/inertialSenseBootLoader.c)
* [inertialSenseBootLoader.h](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/inertialSenseBootLoader.h)
* [ISComm.c](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/ISComm.c)
* [ISComm.h](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/ISComm.h)
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
#include "../../src/inertialSenseBootLoader.h"
```

### Step 2: Initialize and open serial port

```C++
	serial_port_t serialPort;

	// initialize the serial port (Windows, MAC or Linux) - if using an embedded system like Arduino,
	//  you will need to either bootload from Windows, MAC or Linux, or implement your own code that
	//  implements all the function pointers on the serial_port_t struct.
	serialPortPlatformInit(&serialPort);

	// set the port - the bootloader uses this to open the port and enable bootload mode, etc.
	serialPortSetPort(&serialPort, argv[1]);
```

### Step 3: Set bootloader parameters

```C++
	// bootloader parameters
	bootload_params_t param;
	// buffer to show any errors
	char errorBuffer[512];

	// very important - initialize the bootloader params to zeros
	memset(&param, 0, sizeof(param));

	// error buffer, useful if something fails
	param.error = errorBuffer;
	param.errorLength = sizeof(errorBuffer);

	// the file to bootload, *.hex
	param.fileName = argv[2];

	// the serial port
	param.port = &serialPort;

	// progress indicators
	param.uploadProgress = bootloaderUploadProgress;
	param.verifyProgress = bootloaderVerifyProgress;

	// enable verify to read back the firmware and ensure it is correct
	param.flags.bitFields.enableVerify = 1;

	// enable auto-baud, in the event that fast serial communications is not available,
	//  the bootloader will attempt to fall back to a slower speed
	param.flags.bitFields.enableAutoBaud = 1;
```

### Step 4: Run bootloader

```C++
	if (bootloadFileEx(&param))
	{
		printf("Bootloader success on port %s with file %s\n", serialPort.port, param.fileName);
		return 0;
	}
	else
	{
		printf("Bootloader failed! Error: %s\n", errorBuffer);
		return -1;
	}
```

## Compile & Run (Linux/Mac)

1. Create build directory
``` bash
$ cd InertialSenseSDK/ExampleProjects/Bootloader
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
$ ./bin/ISBootloaderExample /dev/ttyUSB0 IS_uINS-3.hex
```
## Compile & Run (Windows MS Visual Studio)

1. Open Visual Studio solution file (InertialSenseSDK\ExampleProjects\Bootloader\VS_project\ISBootloaderExample.sln)
2. Build (F7)
3. Run executable
``` bash
C:\InertialSenseSDK\ExampleProjects\Bootloader\VS_project\Release\ISBootloaderExample.exe COM3 IS_uINS-3.hex
```

## Summary

That covers all the basic functionality you need to set up and talk to <a href="https://inertialsense.com">InertialSense</a> products.  If this doesn't cover everything you need, feel free to reach out to us on the <a href="https://github.com/inertialsense/InertialSenseSDK">InertialSenseSDK</a> GitHub repository, and we will be happy to help.
