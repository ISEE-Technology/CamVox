/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

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
    if (serialPort == 0 || port == 0 || serialPort->pfnOpen == 0)
	{
		return 0;
	}
    return serialPort->pfnOpen(serialPort, port, baudRate, blocking);
}

int serialPortOpenRetry(serial_port_t* serialPort, const char* port, int baudRate, int blocking)
{
    if (serialPort == 0 || port == 0 || serialPort->pfnOpen == 0)
    {
        return 0;
    }

    serialPortClose(serialPort);
    for (int retry = 0; retry < 30; retry++)
    {
        if (serialPortOpen(serialPort, port, baudRate, blocking))
        {
            return 1;
        }
        serialPortSleep(serialPort, 100);
    }
    serialPortClose(serialPort);
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

int serialPortReadLine(serial_port_t* serialPort, unsigned char* buffer, int bufferLength)
{
	return serialPortReadLineTimeout(serialPort, buffer, bufferLength, SERIAL_PORT_DEFAULT_TIMEOUT);
}

int serialPortReadLineTimeout(serial_port_t* serialPort, unsigned char* buffer, int bufferLength, int timeoutMilliseconds)
{
	if (serialPort == 0 || serialPort->handle == 0 || buffer == 0 || bufferLength < 8 || serialPort->pfnRead == 0)
	{
		return 0;
	}

	int prevCR = 0;
	int bufferIndex = 0;
	unsigned char c;
	while (bufferIndex < bufferLength && serialPortReadCharTimeout(serialPort, &c, timeoutMilliseconds) == 1)
	{
		buffer[bufferIndex++] = c;
		if (c == '\n' && prevCR)
		{
			// remove \r\n and null terminate and return count of chars
			buffer[bufferIndex -= 2] = '\0';
			return bufferIndex;
		}
		prevCR = (c == '\r');
	}
	return -1;
}

int serialPortReadAscii(serial_port_t* serialPort, unsigned char* buffer, int bufferLength, unsigned char** asciiData)
{
	return serialPortReadAsciiTimeout(serialPort, buffer, bufferLength, SERIAL_PORT_DEFAULT_TIMEOUT, asciiData);
}

int serialPortReadAsciiTimeout(serial_port_t* serialPort, unsigned char* buffer, int bufferLength, int timeoutMilliseconds, unsigned char** asciiData)
{
	int count = serialPortReadLineTimeout(serialPort, buffer, bufferLength, timeoutMilliseconds);
	unsigned char* ptr = buffer;
	unsigned char* ptrEnd = buffer + count;
	while (*ptr != '$' && ptr < ptrEnd)
	{
		ptr++;
	}

	// if at least 8 chars available
	if (ptrEnd - ptr > 7)
	{
		if (asciiData != 0)
		{
			*asciiData = ptr;
		}
		int checksum = 0;
		int existingChecksum;

		// calculate checksum, skipping leading $ and trailing *XX\r\n
		unsigned char* ptrEndNoChecksum = ptrEnd - 3;
		while (++ptr < ptrEndNoChecksum)
		{
			checksum ^= *ptr;
		}

		if (*ptr == '*')
		{
			// read checksum from buffer, skipping the * char
			existingChecksum = strtol((void*)++ptr, NULL, 16);
			if (existingChecksum == checksum)
			{
				return count;
			}
		}
	}

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
	count += serialPortWrite(serialPort, (unsigned char*)"\r\n", 2);
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
	int count = 0;

	if (*buffer == '$')
	{
		ptr++;
        bufferLength--;
	}
	else
	{
		count += serialPortWrite(serialPort, (const unsigned char*)"$", 1);
	}

    const unsigned char* ptrEnd = ptr + bufferLength;
    unsigned char buf[16];

	count += serialPortWrite(serialPort, (const unsigned char*)buffer, bufferLength);

	while (ptr != ptrEnd)
	{
		checkSum ^= *ptr++;
	}

#ifdef _MSC_VER

#pragma warning(push)
#pragma warning(disable: 4996)

#endif

	snprintf((char*)buf, sizeof(buf), "*%.2x\r\n", checkSum);

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
	if (serialPort == 0 || serialPort->pfnSleep == 0)
	{
		return 0;
	}

	return serialPort->pfnSleep(sleepMilliseconds);
}
