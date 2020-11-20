# logInspector

logInspector is an open source python utility for viewing and scrubbing InertialSense data log (.dat) files.

## Building

**Note** - logInspector requrires Python 3.

### Navigate to SDK directory

``` bash
pip3 install logInspector/ # (this will return an error message, but will install all the dependencies you need)
cd logInspector
python3 setup.py build_ext --inplace
```

### Create the config file.

```
C:\Users\[USER]\Documents\Inertial_Sense\config.yaml
```

### Add the following or similar contents to this file.

```
directory: C:\Users\<username>\Documents\Inertial_Sense\Logs\20181116_SKI\morning_run_1\back\20181116_175352
logs_directory: C:\Users\<username>\Documents\Inertial_Sense\Logs

serials: ["ALL"]
```

## Running

### To run logInspector open a shell and navigate to the logInspector directory and enter the folloing commands:
``` bash
python logInspector.py
```
## Usage
logInspector can open and plot [.dat PPD log files](https://docs.inertialsense.com/user-manual/application-notes/data_logging/#logging-ppd-in-evaltool). The lower left hand corner file browser allows you to enter a "working directory" in the directory field. The log directory can be selected from the directory tree.
Once the log is opened, the buttons in the upper left hand corner are used to graph various data sets.


![NED Map](assets/NEDMap.png "NED Map")

## Standard data sets
-----
* POS NED Map - Used to plot INS position data in NED frame.

* POS NED - INS position in NED frame.

* POS LLA - INS and GNSS position in LLA.

* GPS LLA - GNSS LLA position.

* Vel NED - Velocity in NED frame.

* Vel UVW - Velocity in body frame.

* Attitue - Euler angle attitude in degrees.

* Heading - Heading data from magnetometer, INS, and RTK.

* INS Status - Plots of status flags vs time.

* HDW Status - plots of hardware status flags vs time.


## Other Directory Contents
The *logInspector* also contains some example implementations for dealing with log files directly in python.

### logReader
This python module is responsible for loading the log file through a pybind11 interface.   All the data in the log is eventually put in the `log.data` array.

### logPlotter
This python module is responsible for creating plots.  Adding new plots is easy, data is directly accessed using the member `logReader` object.

### logInspector
A pyqt5 GUI which uses logPlotter to generate plots.

