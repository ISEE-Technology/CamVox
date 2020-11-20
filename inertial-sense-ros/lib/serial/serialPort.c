/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "serialPort.h"
#include <stdlib.h>

int SERIAL_PORT_DEFAULT_TIMEOUT = 2500;

void serialPortSetPort(serial_port_t* serialPort, const char* port)
{
	if (serialPort != 0 && port != 0 && port != serialPort->port)
	{
		int portLength = (int)strnlen(port, MAX_SERIAL_PORT_NAME_LENGTH);
		memcpy(serialPort->port, port, portLength);
		serialPort->port[portLength] = '\0';
	}
}

int serialPortOpen(serial_port_t* serialPort, const char* port, int baudRate, int blocking)
{
	if (serialPort == 0 || port == 0)
	{
		return 0;
	}
	else if (serialPort->pfnOpen != 0)
	{
		if (!serialPort->pfnOpen(serialPort, port, baudRate, blocking))
		{
			serialPortClose(serialPort);
			return 0;
		}
		return 1;
	}
	return 0;
}

int serialPortIsOpen(serial_port_t* serialPort)
{
	if (serialPort == 0 || serialPort->handle == 0)
	{
		return 0;
	}
	return (serialPort->pfnIsOpen ? serialPort->pfnIsOpen(serialPort) : 1);
}

int serialPortClose(serial_port_t* serialPort)
{
	if (serialPort != 0 && serialPort->pfnClose != 0)
	{
		return serialPort->pfnClose(serialPort);
	}
	return 0;
}

int serialPortFlush(serial_port_t* serialPort)
{
	if (serialPort == 0 || serialPort->handle == 0 || serialPort->pfnFlush == 0)
	{
		return 0;
	}
	return serialPort->pfnFlush(serialPort);
}

int serialPortRead(serial_port_t* serialPort, unsigned char* buffer, int readCount)
{
	return serialPortReadTimeout(serialPort, buffer, readCount, -1);
}

int serialPortReadTimeout(serial_port_t* serialPort, unsigned char* buffer, int readCount, int timeoutMilliseconds)
{
	if (serialPort == 0 || serialPort->handle == 0 || buffer == 0 || readCount < 1 || serialPort->pfnRead == 0)
	{
		return 0;
	}

	int count = serialPort->pfnRead(serialPort, buffer, readCount, timeoutMilliseconds);

	if (count < 0)
	{
		return 0;
	}

	return count;
}

int serialPortReadTimeoutAsync(serial_port_t* serialPort, unsigned char* buffer, int readCount, pfnSerialPortAsyncReadCompletion completion)
{
	if (serialPort == 0 || serialPort->handle == 0 || buffer == 0 || readCount < 1 || serialPort->pfnAsyncRead == 0 || completion == 0)
	{
		return 0;
	}

	return serialPort->pfnAsyncRead(serialPort, buffer, readCount, completion);
}

int serialPortReadLine(serial_port_t* serialPort, unsigned char** result)
{
	return serialPortReadLineTimeout(serialPort, result, SERIAL_PORT_DEFAULT_TIMEOUT);
}

int serialPortReadLineTimeout(serial_port_t* serialPort, unsigned char** result, int timeoutMilliseconds)
{
	if (serialPort == 0 || serialPort->handle == 0 || result == 0 || serialPort->pfnRead == 0)
	{
		return 0;
	}

	int prevCR = 0;
	unsigned int memIndex = 0;
	unsigned int memCapacity = 512;
	unsigned char b;
	unsigned char* mem = (unsigned char*)malloc(memCapacity);

	while (serialPortReadCharTimeout(serialPort, &b, timeoutMilliseconds) == 1)
	{
		if (memIndex == memCapacity)
		{
			memCapacity = (unsigned int)(memCapacity * 1.5);
			mem = (unsigned char*)realloc(mem, memCapacity);
		}
		mem[memIndex++] = b;
		if (b == '\n' && prevCR)
		{
			mem[memIndex -= 2] = '\0';
			*result = mem;
			return memIndex;
		}
		prevCR = (b == '\r');
	}
	free(mem);
	mem = 0;
	return -1;
}

int serialPortReadChar(serial_port_t* serialPort, unsigned char* c)
{
	return serialPortReadCharTimeout(serialPort, c, SERIAL_PORT_DEFAULT_TIMEOUT);
}

int serialPortReadCharTimeout(serial_port_t* serialPort, unsigned char* c, int timeoutMilliseconds)
{
	return serialPortReadTimeout(serialPort, c, 1, timeoutMilliseconds);
}

int serialPortWrite(serial_port_t* serialPort, const unsigned char* buffer, int writeCount)
{
	if (serialPort == 0 || serialPort->handle == 0 || buffer == 0 || writeCount < 1 || serialPort->pfnWrite == 0)
	{
		return 0;
	}

	int count = serialPort->pfnWrite(serialPort, buffer, writeCount);

	if (count < 0)
	{
		return 0;
	}

	return count;
}

int serialPortWriteLine(serial_port_t* serialPort, const unsigned char* buffer, int writeCount)
{
	if (serialPort == 0 || serialPort->handle == 0 || buffer == 0 || writeCount < 1)
	{
		return 0;
	}

	int count = serialPortWrite(serialPort, buffer, writeCount);
	count += serialPortWrite(serialPort, (unsigned char[2]) { '\r', '\n' }, 2);
	return count;
}

int serialPortWriteAscii(serial_port_t* serialPort, const char* buffer, int bufferLength)
{
	if (serialPort == 0 || serialPort->handle == 0 || buffer == 0 || bufferLength < 2)
	{
		return 0;
	}

	int checkSum = 0;
	const unsigned char* ptr = (const unsigned char*)buffer;
	const unsigned char* ptrEnd = ptr + bufferLength;
	unsigned char buf[8];

	int count = 0;

	if (*buffer == '$')
	{
		ptr++;
	}
	else
	{
		count += serialPortWrite(serialPort, (const unsigned char*)"$", 1);
	}
	count += serialPortWrite(serialPort, (const unsigned char*)buffer, bufferLength);

	while (ptr != ptrEnd)
	{
		checkSum ^= *ptr++;
	}

#ifdef _MSC_VER

#pragma warning(push)
#pragma warning(disable: 4996)

#endif

	sprintf((char*)buf, "*%.2x\r\n", checkSum);

#ifdef _MSC_VER

#pragma warning(pop)

#endif

	count += serialPortWrite(serialPort, buf, 5);

	return count;
}

int serialPortWriteAndWaitFor(serial_port_t* serialPort, const unsigned char* buffer, int writeCount, const unsigned char* waitFor, int waitForLength)
{
	return serialPortWriteAndWaitForTimeout(serialPort, buffer, writeCount, waitFor, waitForLength, SERIAL_PORT_DEFAULT_TIMEOUT);
}

int serialPortWriteAndWaitForTimeout(serial_port_t* serialPort, const unsigned char* buffer, int writeCount, const unsigned char* waitFor, int waitForLength, const int timeoutMilliseconds)
{
	if (serialPort == 0 || serialPort->handle == 0 || buffer == 0 || writeCount < 1 || waitFor == 0 || waitForLength < 1)
	{
		return 0;
	}

	int actuallyWrittenCount = serialPortWrite(serialPort, buffer, writeCount);

	if (actuallyWrittenCount != writeCount)
	{
		return 0;
	}

	return serialPortWaitForTimeout(serialPort, waitFor, waitForLength, timeoutMilliseconds);
}

int serialPortWaitFor(serial_port_t* serialPort, const unsigned char* waitFor, int waitForLength)
{
	return serialPortWaitForTimeout(serialPort, waitFor, waitForLength, SERIAL_PORT_DEFAULT_TIMEOUT);
}

int serialPortWaitForTimeout(serial_port_t* serialPort, const unsigned char* waitFor, int waitForLength, int timeoutMilliseconds)
{
	if (serialPort == 0 || serialPort->handle == 0 || waitFor == 0 || waitForLength < 1)
	{
		return 1;
	}
	else if (waitForLength > 128)
	{
		return 0;
	}

	unsigned char buf[128] = { 0 };
	int count = serialPortReadTimeout(serialPort, buf, waitForLength, timeoutMilliseconds);

	if (count == waitForLength && memcmp(buf, waitFor, waitForLength) == 0)
	{
		return 1;
	}
	return 0;
}

int serialPortGetByteCountAvailableToRead(serial_port_t* serialPort)
{
	if (serialPort == 0 || serialPort->handle == 0 || serialPort->pfnGetByteCountAvailableToRead == 0)
	{
		return 0;
	}

	return serialPort->pfnGetByteCountAvailableToRead(serialPort);
}

int serialPortGetByteCountAvailableToWrite(serial_port_t* serialPort)
{
	if (serialPort == 0 || serialPort->handle == 0 || serialPort->pfnGetByteCountAvailableToWrite == 0)
	{
		return 0;
	}

	return serialPort->pfnGetByteCountAvailableToWrite(serialPort);
}

int serialPortSleep(serial_port_t* serialPort, int sleepMilliseconds)
{
	if (serialPort == 0 || serialPort->handle == 0 || serialPort->pfnSleep == 0)
	{
		return 0;
	}

	return serialPort->pfnSleep(serialPort, sleepMilliseconds);
}
