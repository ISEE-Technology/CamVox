# SDK Example Projects

The following example projects are provide with the SDK to demonstrate various capabilities of the <a href="https://inertialsense.com">InertialSense</a> products (uINS, uAHRS, and uIMU) and the Inertial Sense SDK.

## Example Projects

* [Ascii Communications](Ascii/README) - How to communicate using ASCII NMEA protocol using SDK.
* [Binary Communications](Communications/README) - How to communicate using InertialSense binary protocol using SDK. 
* [Bootloader](bootloader/README) - How to update firmware on the InertialSense products using SDK.
* [Logger ](Logger/README) - How to data log using using SDK.
* [CLTool](..CLTool/README) - A multipurpose command line tool capable most common functionalities, including those of all example projects.

## Compile & Run (Linux/Mac)

The following steps will build executables for all of the example projects.

1. Create build directory
``` bash
$ cd InertialSenseSDK/ExampleProjects
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
$ ./bin/[EXECUTABLE] /dev/ttyUSB0
```
## Compile & Run (Windows MS Visual Studio)

The following steps will build executables for all of the example projects.

1. Open Visual Studio solution file (InertialSenseSDK\ExampleProjects\VS_project\AllExampleProjects.sln)
2. Select the "Release" Solution Configuration.
3. Build (F7)
4. Run executable
``` bash
C:\InertialSenseSDK\ExampleProjects\Ascii\VS_project\Release\[EXECUTABLE.EXE] COM3
```

## Summary

That covers all the basic functionality you need to set up and talk to <a href="https://inertialsense.com">InertialSense</a> products.  If this doesn't cover everything you need, feel free to reach out to us on the <a href="https://github.com/inertialsense/InertialSenseSDK">InertialSenseSDK</a> github repository, and we will be happy to help.
