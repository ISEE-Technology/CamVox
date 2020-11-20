/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef IS_UTILITIES_H
#define IS_UTILITIES_H

#include <stdlib.h>
#include <inttypes.h>
#include <stdio.h>
#include "data_sets.h"

// C++ API
#ifdef __cplusplus

#include <string>
#include <vector>

using namespace std;

/**
* Encode data as base64
* @param bytes_to_encode the data to encode
* @param in_len the number of bytes to encode
* @param return the base64 encoded data
*/
string base64Encode(const unsigned char* bytes_to_encode, unsigned int in_len);

/**
* Decode base64 data
* @param encoded_string the base64 encoded data
* @return the base64 decoded data - if error, this may be an incomplete set of data
*/
string base64Decode(const string& encoded_string);

/**
* Split string by delimiter
* @param s the string to split
* @param delimiter the delimiter to split on
* @param result cleared and then filled with split strings
* @return the number of items in result
*/
size_t splitString(const string& s, const string& delimiter, vector<string>& result);

/**
* Wraps the mutex functions below
*/
class cMutex
{
public:
	/**
	* Constructor - creates the mutex
	*/
	cMutex();

	/**
	* Destructor - frees the mutex
	*/
	virtual ~cMutex();

	/**
	* Lock the mutex
	*/
	void Lock();

	/**
	* Unlock the mutex
	*/
	void Unlock();

private:
	void* m_handle;
};

/**
* Locks / unlocks a mutex via constructor and destructor
*/
class cMutexLocker
{
public:
	/**
	* Constructor
	* @param mutex the mutex to lock
	*/
	cMutexLocker(cMutex* mutex);

	/**
	* Destructor - unlocks the mutex
	*/
	virtual ~cMutexLocker();

private:
	cMutex* m_mutex;
};

#endif

// C API...
#ifdef __cplusplus
extern "C" {
#endif

#if PLATFORM_IS_WINDOWS

void usleep(__int64 usec);

#define DEFAULT_COM_PORT "COM4"

#ifndef SLEEP_MS
#define SLEEP_MS(milliseconds) Sleep(milliseconds);
#endif

#ifndef SLEEP_US
#define SLEEP_US(timeUs) usleep(timeUs);
#endif

#else // LINUX

#include <unistd.h>
#include <sys/time.h>
#include <stdarg.h>

#define DEFAULT_COM_PORT "/dev/ttyUSB0"

#ifndef SLEEP_MS
#define SLEEP_MS(timeMs) usleep(timeMs * 1000);
#endif

#ifndef SLEEP_US
#define SLEEP_US(timeUs) usleep(timeUs);
#endif

#endif

int current_timeSec();
int current_timeMs();
int current_weekMs();
uint64_t current_weekUs();

uint64_t timerUsStart();
uint64_t timerUsEnd(uint64_t start);
uint64_t timerRawStart();
uint64_t timerRawEnd(uint64_t start);

uint64_t getTickCount(void);

int bootloadUploadProgress(const void* port, float percent);
int bootloadVerifyProgress(const void* port, float percent);
void bootloadStatusInfo(const void* port, const char* str);
float step_sinwave(float *sig_gen, float freqHz, float amplitude, float periodSec);

FILE* openFile(const char* path, const char* mode);
const char* tempPath(); // ends with dir separator

/**
* Return a pointer to 16 elements to map nibbles to for converting to hex
* @return pointer to 16 bytes for mapping nibbles to chars
*/
const unsigned char* getHexLookupTable();

/**
* Get numberic value of a hex code - no bounds check is done, so non-hex chars will return undefined values
* @param hex the hex digit, i.e. 'A'
* @return the numberic value
*/
uint8_t getHexValue(unsigned char hex);

/**
* Create a thread and execute a function
* @param function the function to execute in a background thread
* @param info the parameter to pass to the thread function
* @return the thread handle
*/
void* threadCreateAndStart(void(*function)(void* info), void* info);

/**
* Join a thread with this thread, waiting for it to finish, then free the thread
* @param handle the thread handle to join, wait for finish and then to free
*/
void threadJoinAndFree(void* handle);

/*
* Create a mutex which allows exclusive access to a shared resource
* @return the mutex handle
*/
void* mutexCreate(void);

/**
* Lock a mutex - mutex cannot be locked until mutexUnlock is called
* @param handle the mutex handle to lock
*/
void mutexLock(void* handle);

/**
* Unlock a mutex
* @param handle the mutex handle to unlock
*/
void mutexUnlock(void* handle);

/**
* Free a mutex
* @param handle the mutex handle to free
*/
void mutexFree(void* handle);

// taken from http://www.leapsecond.com/tools/gpsdate.c, uses UTC time
int32_t convertDateToMjd(int32_t year, int32_t month, int32_t day);
int32_t convertGpsToMjd(int32_t gpsWeek, int32_t gpsSeconds);
void convertMjdToDate(int32_t mjd, int32_t* year, int32_t* month, int32_t* day);
void convertGpsToHMS(int32_t gpsSeconds, int32_t* hour, int32_t* minutes, int32_t* seconds);
uint32_t dateToWeekDay(uint32_t ul_year, uint32_t ul_month, uint32_t ul_day);

gen_1axis_sensor_t gen1AxisSensorData(double time, const float val);
gen_3axis_sensor_t gen3AxisSensorData(double time, const float val[3]);
gen_dual_3axis_sensor_t genDual3AxisSensorData(double time, const float val1[3], const float val2[3]);
gen_3axis_sensord_t gen3AxisSensorDataD(double time, const double val[3]);

#ifdef __cplusplus
} // extern C
#endif

#endif // IS_UTILITIES_H
