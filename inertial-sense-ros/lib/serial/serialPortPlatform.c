/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "serialPort.h"
#include "serialPortPlatform.h"
#include "ISConstants.h"

#if PLATFORM_IS_LINUX || PLATFORM_IS_APPLE

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>

// cygwin defines FIONREAD in socket.h instead of ioctl.h
#ifndef FIONREAD
#include <sys/socket.h>
#endif

#if PLATFORM_IS_APPLE

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/serial/ioss.h>
#include <IOKit/IOBSD.h>

#endif

#ifndef error_message
#define error_message printf
#endif

#ifndef B460800
#define B460800 460800
#endif
#ifndef B921600
#define B921600 921600
#endif
#ifndef B1500000
#define B1500000 1500000
#endif
#ifndef B2000000
#define B2000000 2000000
#endif
#ifndef B2500000
#define B2500000 2500000
#endif
#ifndef B3000000
#define B3000000 3000000
#endif

#endif

typedef struct
{
	int blocking;

#if PLATFORM_IS_WINDOWS

	void* platformHandle;
	OVERLAPPED ovRead;
	OVERLAPPED ovWrite;

#else

	int fd;

#endif

} serialPortHandle;

#if PLATFORM_IS_WINDOWS

#define WINDOWS_OVERLAPPED_BUFFER_SIZE 8192

typedef struct
{
	OVERLAPPED ov;
	pfnSerialPortAsyncReadCompletion externalCompletion;
	serial_port_t* serialPort;
	unsigned char* buffer;
} readFileExCompletionStruct;

static void CALLBACK readFileExCompletion(DWORD errorCode, DWORD bytesTransferred, LPOVERLAPPED ov)
{
	readFileExCompletionStruct* c = (readFileExCompletionStruct*)ov;
	c->externalCompletion(c->serialPort, c->buffer, bytesTransferred, errorCode);
	free(c);
}

#else

static int get_baud_speed(int baudRate)
{
	switch (baudRate)
	{
	default:      return 0;
	case 300:     return B300;
	case 600:     return B600;
	case 1200:    return B1200;
	case 2400:    return B2400;
	case 4800:    return B4800;
	case 9600:    return B9600;
	case 19200:   return B19200;
	case 38400:   return B38400;
	case 57600:   return B57600;
	case 115200:  return B115200;
	case 230400:  return B230400;
	case 460800:  return B460800;
	case 921600:  return B921600;
	case 1500000: return B1500000;
	case 2000000: return B2000000;
	case 2500000: return B2500000;
	case 3000000: return B3000000;
	}
}

static int set_interface_attribs(int fd, int speed, int parity)
{
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0)
	{
		error_message("error %d from tcgetattr", errno);
		return -1;
	}

#if PLATFORM_IS_APPLE

	// set a valid speed for MAC, serial won't open otherwise
	cfsetospeed(&tty, 230400);
	cfsetispeed(&tty, 230400);

	// HACK: Set the actual speed, allows higher than 230400 baud
	if (ioctl(fd, IOSSIOSPEED, &speed) == -1)
	{
		error_message("error %d from ioctl IOSSIOSPEED", errno);
	}

#else

	speed = get_baud_speed(speed);
	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

#endif

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
													// disable IGNBRK for mismatched speed tests; otherwise receive break
													// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN] = 0;             // read doesn't block
	tty.c_cc[VTIME] = 0;            // no timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
									// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);  // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		error_message("error %d from tcsetattr", errno);
		return -1;
	}

	// re-open and remove additional flags
	memset(&tty, 0, sizeof(tty));

	// Check if the file descriptor is pointing to a TTY device or not.
	if (!isatty(fd))
	{
		return -1;
	}

	// Get the current configuration of the serial interface
	if (tcgetattr(fd, &tty) < 0)
	{
		return -1;
	}

	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	// config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	tty.c_iflag = 0;

	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	tty.c_oflag = 0;

	// No line processing
	//
	// echo off, echo newline off, canonical mode off, 
	// extended input processing off, signal chars off
	//
	// config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	tty.c_lflag = 0;

	// Turn off character processing
	//
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	tty.c_cflag &= ~(CSIZE | PARENB);
	tty.c_cflag |= CS8;

	// One input byte is enough to return from read()
	// Inter-character timer off
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 0;

	// Communication speed (simple version, using the predefined
	// constants)
	//
	// if(cfsetispeed(&config, B9600) < 0 || cfsetospeed(&config, B9600) < 0) 
	// 	return 0;

	// Finally, apply the configuration
	if (tcsetattr(fd, TCSAFLUSH, &tty) < 0)
	{
		return -1;
	}

	return 0;
}

#endif

static int serialPortOpenPlatform(serial_port_t* serialPort, const char* port, int baudRate, int blocking)
{
	if (serialPort->handle != 0)
	{
		// already open
		return 0;
	}

	serialPortSetPort(serialPort, port);

#if PLATFORM_IS_WINDOWS

	void* platformHandle = 0;
	platformHandle = CreateFileA(serialPort->port, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, blocking ? FILE_FLAG_OVERLAPPED : 0, 0);
	if (platformHandle == INVALID_HANDLE_VALUE)
	{
		// don't modify the originally requested port value, just create a new value that Windows needs for COM10 and above
		char tmpPort[MAX_SERIAL_PORT_NAME_LENGTH];
		sprintf_s(tmpPort, sizeof(tmpPort), "\\\\.\\%s", port);
		platformHandle = CreateFileA(tmpPort, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, blocking ? FILE_FLAG_OVERLAPPED : 0, 0);
		if (platformHandle == INVALID_HANDLE_VALUE)
		{
			return 0;
		}
	}

	DCB serialParams;
	serialParams.DCBlength = sizeof(DCB);
	if (GetCommState(platformHandle, &serialParams))
	{
		serialParams.BaudRate = baudRate;
		serialParams.ByteSize = DATABITS_8;
		serialParams.StopBits = ONESTOPBIT;
		serialParams.Parity = NOPARITY;
		serialParams.fBinary = 1;
		serialParams.fInX = 0;
		serialParams.fOutX = 0;
		serialParams.fAbortOnError = 0;
		serialParams.fNull = 0;
		serialParams.fErrorChar = 0;
		serialParams.fDtrControl = DTR_CONTROL_ENABLE;
		serialParams.fRtsControl = RTS_CONTROL_ENABLE;
		if (!SetCommState(platformHandle, &serialParams))
		{
			serialPortClose(serialPort);
			return 0;
		}
	}
	else
	{
		serialPortClose(serialPort);
		return 0;
	}

	DWORD timeout = (blocking ? 1 : 0);
	COMMTIMEOUTS timeouts = { MAXDWORD, timeout, timeout, 0, 0 };
	if (!SetCommTimeouts(platformHandle, &timeouts))
	{
		serialPortClose(serialPort);
		return 0;
	}

	serialPortHandle* handle = (serialPortHandle*)calloc(sizeof(serialPortHandle), 1);
	handle->blocking = blocking;
	handle->platformHandle = platformHandle;
	if (blocking)
	{
		handle->ovRead.hEvent = CreateEvent(0, 1, 0, 0);
		handle->ovWrite.hEvent = CreateEvent(0, 1, 0, 0);
	}
	serialPort->handle = handle;

#else

	int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd < 0 || set_interface_attribs(fd, baudRate, 0) != 0)
	{
		return 0;
	}
	serialPortHandle* handle = (serialPortHandle*)calloc(sizeof(serialPortHandle), 1);
	handle->fd = fd;
	handle->blocking = blocking;
	serialPort->handle = handle;

#endif

	return 1;	// success
}

static int serialPortIsOpenPlatform(serial_port_t* serialPort)
{

#if PLATFORM_IS_WINDOWS

	DCB serialParams;
	serialParams.DCBlength = sizeof(DCB);
	serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
	return GetCommState(handle->platformHandle, &serialParams);

#else

	struct stat sb;
	return (stat(serialPort->port, &sb) == 0);

#endif

}

static int serialPortClosePlatform(serial_port_t* serialPort)
{
	serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
	if (handle == 0)
	{
		// not open, no close needed
		return 0;
	}

#if PLATFORM_IS_WINDOWS

	CancelIo(handle->platformHandle);
	if (handle->blocking)
	{
		CloseHandle(handle->ovRead.hEvent);
		CloseHandle(handle->ovWrite.hEvent);
	}
	CloseHandle(handle->platformHandle);
	handle->platformHandle = 0;

#else

	close(handle->fd);
	handle->fd = 0;

#endif

	free(handle);
	serialPort->handle = 0;

	return 1;
}

static int serialPortFlushPlatform(serial_port_t* serialPort)
{
	serialPortHandle* handle = (serialPortHandle*)serialPort->handle;

#if PLATFORM_IS_WINDOWS

	if (!PurgeComm(handle->platformHandle, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR))
	{
		return 0;
	}

#else

	tcflush(handle->fd, TCIOFLUSH);

#endif

	return 1;
}

#if PLATFORM_IS_WINDOWS

static int serialPortReadTimeoutPlatformWindows(serialPortHandle* handle, unsigned char* buffer, int readCount, int timeoutMilliseconds)
{
	DWORD dwRead = 0;
	int totalRead = 0;
	ULONGLONG startTime = GetTickCount64();
	while (totalRead < readCount && GetTickCount64() - startTime <= timeoutMilliseconds)
	{
		if (ReadFile(handle->platformHandle, buffer + totalRead, readCount - totalRead, &dwRead, handle->blocking ? &handle->ovRead : 0))
		{
			if (handle->blocking)
			{
				GetOverlappedResult(handle->platformHandle, &handle->ovRead, &dwRead, 1);
			}
			totalRead += dwRead;
		}
		else if (handle->blocking)
		{
			DWORD dwRes = GetLastError();
			if (dwRes == ERROR_IO_PENDING)
			{
				dwRes = WaitForSingleObject(handle->ovRead.hEvent, timeoutMilliseconds);
				switch (dwRes)
				{
				case WAIT_OBJECT_0:
					if (!GetOverlappedResult(handle->platformHandle, &handle->ovRead, &dwRead, 0))
					{
						CancelIo(handle->platformHandle);
					}
					else
					{
						totalRead += dwRead;
					}
					break;

				default:
					CancelIo(handle->platformHandle);
					break;
				}
			}
			else
			{
				CancelIo(handle->platformHandle);
			}
		}
		if (!handle->blocking && totalRead == 0)
		{
			Sleep(2);
		}
	} 
	return totalRead;
}

#else

static int serialPortReadTimeoutPlatformLinux(serialPortHandle* handle, unsigned char* buffer, int readCount, int timeoutMilliseconds)
{
	int totalRead = 0;
	int dtMs;
	int n;
	struct timeval start, curr;
	if (timeoutMilliseconds > 0)
	{
		gettimeofday(&start, NULL);
	}

	while (1)
	{
		if (timeoutMilliseconds > 0)
		{
			struct pollfd fds[1];
			fds[0].fd = handle->fd;
			fds[0].events = POLLIN;
			int pollrc = poll(fds, 1, timeoutMilliseconds);
			if (pollrc <= 0 || !(fds[0].revents & POLLIN))
			{
				break;
			}
		}
		n = read(handle->fd, buffer + totalRead, readCount - totalRead);
		if (n < -1)
		{
			error_message("error %d from read, fd %d", errno, handle->fd);
			return 0;
		}
		else if (n != -1)
		{
			totalRead += n;
		}
		if (timeoutMilliseconds > 0 && totalRead < readCount)
		{
			gettimeofday(&curr, NULL);
			dtMs = ((curr.tv_sec - start.tv_sec) * 1000) + ((curr.tv_usec - start.tv_usec) / 1000);
			if (dtMs >= timeoutMilliseconds)
			{
				break;
			}

			// try for another loop around with a lower timeout
			timeoutMilliseconds = _MAX(0, timeoutMilliseconds - dtMs);
		}
		else
		{
			break;
		}
	}
	return totalRead;
}

#endif

static int serialPortReadTimeoutPlatform(serial_port_t* serialPort, unsigned char* buffer, int readCount, int timeoutMilliseconds)
{
	serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
	if (timeoutMilliseconds < 0)
	{
		timeoutMilliseconds = (handle->blocking ? SERIAL_PORT_DEFAULT_TIMEOUT : 0);
	}

#if PLATFORM_IS_WINDOWS

	return serialPortReadTimeoutPlatformWindows(handle, buffer, readCount, timeoutMilliseconds);

#else

	return serialPortReadTimeoutPlatformLinux(handle, buffer, readCount, timeoutMilliseconds);

#endif

}

static int serialPortAsyncReadPlatform(serial_port_t* serialPort, unsigned char* buffer, int readCount, pfnSerialPortAsyncReadCompletion completion)
{
	serialPortHandle* handle = (serialPortHandle*)serialPort->handle;

#if PLATFORM_IS_WINDOWS

	readFileExCompletionStruct* c = (readFileExCompletionStruct*)malloc(sizeof(readFileExCompletionStruct));
	c->externalCompletion = completion;
	c->serialPort = serialPort;
	c->buffer = buffer;
	memset(&c->ov, 0, sizeof(c->ov));

	if (!ReadFileEx(handle->platformHandle, buffer, readCount, (LPOVERLAPPED)c, readFileExCompletion))
	{
		return 0;
	}

#else

	// no support for async, just call the completion right away
	int n = read(handle->fd, buffer, readCount);
	completion(serialPort, buffer, (n < 0 ? 0 : n), (n >= 0 ? 0 : n));

#endif

	return 1;
}

static int serialPortWritePlatform(serial_port_t* serialPort, const unsigned char* buffer, int writeCount)
{
	serialPortHandle* handle = (serialPortHandle*)serialPort->handle;

#if PLATFORM_IS_WINDOWS

	DWORD dwWritten;
	if (!WriteFile(handle->platformHandle, buffer, writeCount, &dwWritten, handle->blocking ? &handle->ovWrite : 0))
	{
		DWORD result = GetLastError();
		if (result != ERROR_IO_PENDING)
		{
			CancelIo(handle->platformHandle);
			return 0;
		}
	}
	
	if (handle->blocking)
	{
		if (!GetOverlappedResult(handle->platformHandle, &handle->ovWrite, &dwWritten, 1))
		{
			CancelIo(handle->platformHandle);
			return 0;
		}
	}
	return dwWritten;
	

#else

	return write(handle->fd, buffer, writeCount);

	// if desired in the future, this will block until the data has been successfully written to the serial port
	/*
	int count = write(handle->fd, buffer, writeCount);
	int error = tcdrain(handle->fd);

	if (error != 0)
	{
	error_message("error %d from tcdrain", errno);
	return 0;
	}

	return count;
	*/

#endif

	if (serialPort->pfnWrite)
	{
		return serialPort->pfnWrite(serialPort, buffer, writeCount);
	}

	return 0;
}

static int serialPortGetByteCountAvailableToReadPlatform(serial_port_t* serialPort)
{
	serialPortHandle* handle = (serialPortHandle*)serialPort->handle;


#if PLATFORM_IS_WINDOWS

	COMSTAT commStat;
	if (ClearCommError(handle->platformHandle, 0, &commStat))
	{
		return commStat.cbInQue;
	}
	return 0;

#else

	int bytesAvailable;
	ioctl(handle->fd, FIONREAD, &bytesAvailable);
	return bytesAvailable;

#endif

}

static int serialPortGetByteCountAvailableToWritePlatform(serial_port_t* serialPort)
{
	// serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
	(void)serialPort;

	return 65536;

	/*
	int bytesUsed;
	struct serial_struct serinfo;
	memset(&serinfo, 0, sizeof(serial_struct));
	ioctl(handle->fd, TIOCGSERIAL, &serinfo);
	ioctl(handle->fd, TIOCOUTQ, &bytesUsed);
	return serinfo.xmit_fifo_size - bytesUsed;
	*/
}

static int serialPortSleepPlatform(serial_port_t* serialPort, int sleepMilliseconds)
{
	(void)serialPort;

#if PLATFORM_IS_WINDOWS

	Sleep(sleepMilliseconds);

#else

	usleep(sleepMilliseconds * 1000);

#endif

	return 1;
}

int serialPortPlatformInit(serial_port_t* serialPort)
{
	serialPort->pfnClose = serialPortClosePlatform;
	serialPort->pfnFlush = serialPortFlushPlatform;
	serialPort->pfnOpen = serialPortOpenPlatform;
	serialPort->pfnIsOpen = serialPortIsOpenPlatform;
	serialPort->pfnRead = serialPortReadTimeoutPlatform;
	serialPort->pfnAsyncRead = serialPortAsyncReadPlatform;
	serialPort->pfnWrite = serialPortWritePlatform;
	serialPort->pfnGetByteCountAvailableToRead = serialPortGetByteCountAvailableToReadPlatform;
	serialPort->pfnGetByteCountAvailableToWrite = serialPortGetByteCountAvailableToWritePlatform;
	serialPort->pfnSleep = serialPortSleepPlatform;
	return 0;
}
